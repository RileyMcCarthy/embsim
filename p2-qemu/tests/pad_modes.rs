//! A pad is a Thevenin source at the strength its `WRPIN` word configured,
//! and a pad that only pulls its net reads what the net resolved to.
//!
//! The guest is fourteen instructions of PASM2, hand-assembled below and
//! checked against flexspin's listing, run on QEMU from cog 0 in place of
//! the boot ROM. It configures three pads and samples one:
//!
//! - `P0`: `P_HIGH_15K`, driven high — a 15 kΩ pull-up — on a net a bench
//!   pin holds **low** through 25 Ω. Rule 2 (`NODES.md` §2) makes the
//!   sink the node's source and the pad a pull that never contends, and
//!   the pad's `IN` must then read the **net**, not its own `OUT` bit:
//!   `testp #0` reads 0, and the guest writes `"0"` to the debug pin. This
//!   is the read an I2C master depends on for a slave's clock stretch and
//!   its ACK, and the sample the old `(dir & out) | (!dir & in)` rule got
//!   wrong.
//! - `P1`: the same word, driven high, on a net of its own — the net reads
//!   `Pulled(High, 15 000)`, the pad's own impedance in the path.
//! - `P2`: `P_HIGH_FLOAT`, driven high — released; the net floats.
//!
//! Every pad change reaches its net as its own publish (the float-mode
//! pad's `drvh` is none: released to released), and the test holds the
//! fight count at zero: a 15 kΩ pull against a 25 Ω sink is not contention.
//!
//! The bench supplies what the chip needs to run and to drive: `VDD` at
//! 1.8 V and `RESN` released, which open the package's START gate at the
//! build, and the `VIO_0_3` bank at 3.3 V, which is what `P0`–`P2` drive
//! high at (`embsim_boards::p2`, "Pads drive high at their bank's
//! supply").
//!
//! The bench carries two such chips, `P2` and `P2B`, each its own
//! `qemu-system-p2`, each pad held by a sink of its own: two P2s in one
//! process, each reading its own nets.
//!
//! Needs a `qemu-system-p2` (`embsim qemu install`), so it is `#[ignore]`d
//! in the workspace's tests; CI's `p2-qemu-boot` job runs it.

use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, AttachError, Component, ComponentNetIo, EndpointRef, Finding, Harness, Level,
    NetState, PinDecl, System,
};
use embsim_boards::p2::{P2Package, P2_PULL_15K_OHMS};
use embsim_core::virtual_clock;
use embsim_p2_qemu::{P2Qemu, P2QemuHandle};

/// The P2's debug transmit pin, where the guest writes its sample.
const DEBUG_TX: u8 = 62;

/// The guest, as flexspin 6.0.5 assembles it (`-2 -l`):
///
/// ```text
///         org     0
///         wrpin   ##P_HIGH_15K, #0      ' FF800008 FC0C0000  (AUGD, WRPIN)
///         drvh    #0                    ' FD640059
///         wrpin   ##P_HIGH_15K, #1      ' FF800008 FC0C0001
///         drvh    #1                    ' FD640259
///         wrpin   ##P_HIGH_FLOAT, #2    ' FF80001C FC0C0002
///         drvh    #2                    ' FD640459
///         testp   #0 wc                 ' FD740040
///   if_c  mov     pa, #"1"              ' C607EC31
///   if_nc mov     pa, #"0"              ' 3607EC30
///         wypin   pa, #62               ' FC27EC3E
///         jmp     #\13                  ' FD80000D  (to itself)
/// ```
///
/// `P_HIGH_15K` is `%0000_0000_000_0000000010000_00_00000_0` = `$1000`
/// (`HHH` = `%010` at bits 13:11) and `P_HIGH_FLOAT` `$3800` (`%111`);
/// each needs an `AUGD` for its upper 23 bits. The three `drvh` are one
/// pad change each; `testp` samples `IN`; the conditional `mov`s turn C
/// into a character; `wypin` puts it where the console tap records it.
const PROGRAM: [u32; 14] = [
    0xFF80_0008,
    0xFC0C_0000,
    0xFD64_0059,
    0xFF80_0008,
    0xFC0C_0001,
    0xFD64_0259,
    0xFF80_001C,
    0xFC0C_0002,
    0xFD64_0459,
    0xFD74_0040,
    0xC607_EC31,
    0x3607_EC30,
    0xFC27_EC3E,
    0xFD80_000D,
];

/// A bench pin holding its net low through the default 25 Ω from attach:
/// an I2C slave stretching the clock, a device asserting a line.
struct Sink {
    pins: [PinDecl; 1],
}

impl Sink {
    fn new() -> Self {
        Self {
            pins: [PinDecl::digital_out("A").with_idle(Some(digital_drive(Level::Low)))],
        }
    }
}

impl Component for Sink {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The bench supplies of the chip named `p2`: the core rail inside its
/// window and reset released (the START gate), and the bank the guest
/// drives pads in — each measured against the ground the bench holds at
/// 0 V, which is not implicit (`DESIGN.md` rule 6): a pin's sense is its
/// voltage against its reference pin, `GND`.
fn supplies(harness: Harness, p2: &str) -> Harness {
    harness
        .power(ep(&format!("BENCH.{p2}GND")), ep(&format!("{p2}.GND")), 0.0)
        .power(ep(&format!("BENCH.{p2}VDD")), ep(&format!("{p2}.VDD")), 1.8)
        .power(
            ep(&format!("BENCH.{p2}RESN")),
            ep(&format!("{p2}.RESN")),
            3.3,
        )
        .power(
            ep(&format!("BENCH.{p2}VIO_0_3")),
            ep(&format!("{p2}.VIO_0_3")),
            3.3,
        )
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    pred()
}

/// The guest's sample on the debug pin, the nets it left, its publishes,
/// as one chip's line in a failure message.
fn chip(system: &embsim_board::SystemHandle, name: &str, handle: &P2QemuHandle) -> String {
    let nets: Vec<String> = ["P0", "P1", "P2", "P3"]
        .iter()
        .map(|pad| {
            format!(
                "{name}.{pad}={:?}",
                system.net_state(&format!("{name}.{pad}"))
            )
        })
        .collect();
    format!(
        "{name}: console={:?} yields={} publishes={} slices={} halted={} failure={:?} nets={nets:?}",
        handle.console(DEBUG_TX),
        handle.yields(),
        handle.publishes(),
        handle.slices(),
        handle.halted(),
        handle.failure(),
    )
}

#[test]
#[ignore = "needs qemu-system-p2 (embsim qemu install); CI's p2-qemu-boot job runs it"]
fn a_pad_pulling_its_net_high_reads_the_sink_holding_it_low() {
    let image: Vec<u8> = PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect();
    let p2 = P2Qemu::with_boot_rom(&image, &[]).expect("qemu-system-p2 starts");
    let p2b = P2Qemu::with_boot_rom(&image, &[]).expect("a second qemu-system-p2 starts");
    let handles = [("P2", p2.handle()), ("P2B", p2b.handle())];
    assert_ne!(handles[0].1.pid(), handles[1].1.pid(), "one program each");

    virtual_clock::init(0.0, 160_000_000);

    let system = System::new()
        .component("P2", Box::new(P2Package::new(p2)))
        .component("P2B", Box::new(P2Package::new(p2b)))
        .component("SINK", Box::new(Sink::new()))
        .component("SINKB", Box::new(Sink::new()))
        .harness(supplies(
            supplies(
                Harness::new()
                    .connect_str("SINK.A", "P2.P0")
                    .expect("the pad is a bench endpoint")
                    .connect_str("SINKB.A", "P2B.P0")
                    .expect("the pad is a bench endpoint"),
                "P2",
            ),
            "P2B",
        ))
        .start()
        .expect("the bench starts");

    wait_for(
        || {
            handles
                .iter()
                .all(|(_, h)| !h.console(DEBUG_TX).is_empty() || h.halted())
        },
        Duration::from_secs(30),
    );

    for (name, handle) in &handles {
        let state = chip(&system, name, handle);
        // The sample: IN on a pulling pad is the net, and the net is the
        // sink's.
        assert_eq!(
            handle.console(DEBUG_TX),
            "0",
            "a pad pulling high through 15 kΩ reads the 25 Ω sink holding its net low; {state}"
        );

        // What each pad put on its net.
        assert_eq!(
            system.net_state(&format!("{name}.P0")),
            Some(NetState::Driven(Level::Low)),
            "the sink wins the node; the pull never contends: {state}"
        );
        assert_eq!(
            system.net_state(&format!("{name}.P1")),
            Some(NetState::Pulled(Level::High, P2_PULL_15K_OHMS)),
            "a 15 kΩ pull-up alone on its net is a pull through its own impedance: {state}"
        );
        assert_eq!(
            system.net_state(&format!("{name}.P2")),
            Some(NetState::Floating),
            "P_HIGH_FLOAT driven high is a released pad: {state}"
        );
        assert_eq!(
            system.net_state(&format!("{name}.P3")),
            Some(NetState::Floating),
            "an untouched pad stays released: {state}"
        );

        // Two pad changes, each its own publish — `drvh #2` on a float-mode
        // pad leaves it released, which is what its net was already told,
        // so it is no pad change at all.
        assert_eq!(
            handle.publishes(),
            2,
            "drvh on P0 and P1 are the guest's only pad changes; a float-mode pad driven high \
             is still released: {state}"
        );
        assert_eq!(handle.yields(), 2, "{state}");
    }
    let fights: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|f| matches!(f, Finding::Contention { .. }))
        .collect();
    assert_eq!(fights, Vec::<Finding>::new());
    assert_eq!(system.escalated_solves(), 0, "projections only");
    drop(system);
}
