//! The carriage seam, closed: firmware pulse-out → STEP pin → a real
//! [`StepperMotor`] plant → its shaft → a real [`QuadratureEncoder`] → the A/B
//! pins → back into the firmware's encoder bank.
//!
//! This is the loop a consumer has had to hand-wire in Rust — subscribe to
//! `pulse_out::on_progress`, read `pulse_out::frequency`, integrate a plant by
//! hand, and write `encoder::set` — because `McuComponent` bridged serial only
//! and its motion channels had no pins. Every hop below is now a *system
//! description* instead: two channel tables, three harness lines, and the
//! components' own physics.
//!
//! What each assertion is really for:
//!
//! - **The commanded count crosses the wire exactly.** `commanded_steps()` on
//!   the far side of the harness must equal the pulse total the firmware asked
//!   for — not approximately, exactly. Encoder feedback is a closed loop, so
//!   an off-by-a-few step count is a silently wrong machine.
//! - **The encoder walks, it does not teleport.** `snapped_updates() == 0`
//!   proves the counts the firmware reads came from a real Gray-code walk on
//!   real pins, decoded by the bridge — not from a model writing the bank.
//! - **Direction agrees on both paths.** The drive latches its own `DIR` pin
//!   while the train carries a direction of its own; a machine whose two
//!   descriptions of "reverse" disagree is the classic inverted-axis bug.
//!
//! Its own binary: it owns the process-default peripheral banks and the
//! process-global virtual clock (`TESTING.md` rule 5).

use rstest::rstest;
use std::time::{Duration, Instant};

use embsim_board::mcu::{
    EncoderChannelConfig, GpioChannelConfig, GpioDirection, PulseOutChannelConfig,
};
use embsim_board::{Harness, Level, McuComponent, NetState, PulseDirection, System, SystemHandle};
use embsim_core::virtual_clock;
use embsim_models::machine::{quadrature_encoder, stepper_motor, QuadratureEncoder, StepperMotor};
use embsim_peripherals::{encoder, gpio, pulse_out};

/// GPIO channel 0: the drive's enable, open-collector — *active* pulls the pin
/// low, and the drive is enabled by a low `ENA`.
const ENA_CHANNEL: usize = 0;
/// GPIO channel 1: direction, active-high — and on this machine *active means
/// reverse*, which is stated twice below and asserted to agree.
const DIR_CHANNEL: usize = 1;

const GPIO_TABLE: [GpioChannelConfig; 2] = [
    GpioChannelConfig {
        pin: 6,
        active_low: true,
    },
    GpioChannelConfig {
        pin: 7,
        active_low: false,
    },
];
const PULSE_TABLE: [PulseOutChannelConfig; 1] = [PulseOutChannelConfig { pin: 8 }];
const ENCODER_TABLE: [EncoderChannelConfig; 1] = [EncoderChannelConfig {
    pin_a: 20,
    pin_b: 21,
}];

/// Steps in each leg of the move. Small enough that the encoder's Gray walk
/// (one engine transition per count — the encoder is *not* rate-carried) stays
/// cheap, large enough that a lost or doubled step is unmistakable.
const MOVE_STEPS: u32 = 100;
/// Step rate of each leg.
const STEP_HZ: u32 = 20_000;
/// Virtual duration of one `MOVE_STEPS` train at [`STEP_HZ`].
const TRAIN_US: u64 = MOVE_STEPS as u64 * 1_000_000 / STEP_HZ as u64;
/// Coast after the last pulse so the first-order lag (`tau_s = 0.5 ms`)
/// settles. ~20 τ.
const SETTLE_US: u64 = 10_000;

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

/// Home the encoder channel, exactly as firmware does after a datum move.
///
/// A quadrature counter has no absolute meaning at boot: the encoder drives
/// its own count-0 phase at attach while the pins idle high, so the bridge
/// faithfully counts the transitions between the two. That offset is the
/// physically correct answer, and homing is what turns it into a datum.
fn home_encoder(system: &SystemHandle) {
    assert!(
        wait_for(
            || system.net_state("ENC.A") == Some(NetState::Driven(Level::Low))
                && system.net_state("ENC.B") == Some(NetState::Driven(Level::Low)),
            Duration::from_secs(5)
        ),
        "the encoder must present its count-0 phase at attach; A = {:?} B = {:?}",
        system.net_state("ENC.A"),
        system.net_state("ENC.B")
    );
    // …and the bridge must have finished decoding it before the datum is set.
    const STABLE_POLLS: usize = 5;
    let deadline = Instant::now() + Duration::from_secs(5);
    let (mut last, mut stable) = (encoder::value(0), 0);
    while Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(2));
        let now = encoder::value(0);
        if now == last {
            stable += 1;
            if stable >= STABLE_POLLS {
                encoder::set(0, 0);
                return;
            }
        } else {
            (last, stable) = (now, 0);
        }
    }
    panic!("the encoder count never settled before homing (last {last})");
}

/// The whole axis, wired by description alone.
#[rstest]
fn the_carriage_seam_closes_through_real_pins() {
    virtual_clock::init(0.0, 1_000_000);
    gpio::init(GPIO_TABLE.len(), None);
    pulse_out::init(PULSE_TABLE.len());
    encoder::init(ENCODER_TABLE.len());

    let mcu = McuComponent::builder("p2")
        .gpio_table(GPIO_TABLE.to_vec())
        .bridge_gpio(ENA_CHANNEL, GpioDirection::Output)
        .bridge_gpio(DIR_CHANNEL, GpioDirection::Output)
        .pulse_out_table(PULSE_TABLE.to_vec())
        .bridge_pulse_out_with_direction(0, DIR_CHANNEL, PulseDirection::Reverse)
        .encoder_table(ENCODER_TABLE.to_vec())
        .bridge_encoder(0)
        .build()
        .expect("MCU builds from the channel tables");

    // One count per step and no load, so the assertions are about the loop
    // rather than about the plant's own parameters (which are unit-tested in
    // `embsim-models`). Lag is well under the 50 µs step interval so the
    // encoder tracks commanded counts; observe cadence is fine enough that
    // the Gray walk hits every count.
    let motor = StepperMotor::new(stepper_motor::Config {
        tau_s: 1e-6,
        load_loss: 0.0,
        // The machine's convention, stated on the drive: DIR high = reverse.
        dir_forward_level: Level::Low,
        // …and its open-collector enable: ENA low = enabled.
        enable_active_low: true,
        observe_interval_us: Some(200),
        ..stepper_motor::Config::new(1.0)
    })
    .expect("valid motor config");
    let shaft = motor.shaft();

    let encoder_model =
        QuadratureEncoder::new(quadrature_encoder::Config::new(1.0)).expect("valid encoder config");
    let input = encoder_model.input();
    {
        // The mechanical seam: the motor publishes millimetres, the encoder
        // applies its own counts/mm. Nothing here knows about the firmware.
        let input = input.clone();
        shaft.on_position_change(move |mm| input.set_position_mm(mm));
    }

    let harness = Harness::new()
        .connect_str("MCU.P8", "MOTOR.STEP")
        .expect("endpoint")
        .connect_str("MCU.P7", "MOTOR.DIR")
        .expect("endpoint")
        .connect_str("MCU.P6", "MOTOR.ENA")
        .expect("endpoint")
        .connect_str("ENC.A", "MCU.P20")
        .expect("endpoint")
        .connect_str("ENC.B", "MCU.P21")
        .expect("endpoint");

    let system = System::new()
        .component("MCU", Box::new(mcu))
        .component("MOTOR", Box::new(motor))
        .component("ENC", Box::new(encoder_model))
        .harness(harness)
        .start()
        .expect("live system starts");

    home_encoder(&system);

    // --- enable ---------------------------------------------------------
    gpio::set_active(ENA_CHANNEL, true);
    assert!(
        wait_for(|| shaft.enabled(), Duration::from_secs(5)),
        "an active (low) ENA must enable the drive"
    );

    // --- forward leg ----------------------------------------------------
    pulse_out::start(0, MOVE_STEPS, STEP_HZ);
    // Jump virtual time through the train and the lag; wall sleep does not
    // move the counter (and an unpaced run would otherwise race years ahead
    // while the plant was still mid-train).
    virtual_clock::wait_virtual_us(TRAIN_US + SETTLE_US);
    assert!(
        wait_for(
            || shaft.commanded_steps() == i64::from(MOVE_STEPS),
            Duration::from_secs(5)
        ),
        "the drive must reconstruct exactly {MOVE_STEPS} commanded steps, got {}",
        shaft.commanded_steps()
    );
    assert_eq!(
        u64::try_from(shaft.commanded_steps()).expect("forward travel is positive"),
        pulse_out::emitted(0),
        "and agree with the firmware's own emitted count"
    );
    assert!(
        wait_for(
            || encoder::value(0) == MOVE_STEPS as i32,
            Duration::from_secs(5)
        ),
        "the firmware's encoder bank must reach {MOVE_STEPS} counts, got {} \
         (commanded={} pos={:.3})",
        encoder::value(0),
        shaft.commanded_steps(),
        shaft.position_counts(),
    );
    assert_eq!(
        input.snapped_updates(),
        0,
        "every count must arrive as a real Gray transition on the pins"
    );

    // --- reverse leg ----------------------------------------------------
    gpio::set_active(DIR_CHANNEL, true);
    assert!(
        wait_for(|| !shaft.forward(), Duration::from_secs(5)),
        "the drive latches reverse from its own DIR pin"
    );
    assert_eq!(
        shaft.train().map(|train| train.direction),
        Some(PulseDirection::Reverse),
        "…and the train carries the same direction — the two descriptions of \
         'reverse' must agree, or the axis is silently inverted"
    );

    pulse_out::start(0, MOVE_STEPS, STEP_HZ);
    virtual_clock::wait_virtual_us(TRAIN_US + SETTLE_US);
    assert!(
        wait_for(|| shaft.commanded_steps() == 0, Duration::from_secs(5)),
        "{MOVE_STEPS} forward then {MOVE_STEPS} reverse is zero commanded \
         steps, got {}",
        shaft.commanded_steps()
    );
    assert!(
        wait_for(|| encoder::value(0) == 0, Duration::from_secs(5)),
        "…and the carriage is back at its datum; bank reads {}",
        encoder::value(0)
    );
    assert_eq!(input.snapped_updates(), 0);

    system.shutdown();
    pulse_out::reset();
    gpio::reset();
    encoder::reset();
}
