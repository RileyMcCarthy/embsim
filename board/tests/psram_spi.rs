//! The APS6404L PSRAM as a LIVE part of the P2-EC32MB module, driven bit by
//! bit over the module's own nets: a Read ID from `U302` comes back as the
//! datasheet's manufacturer and known-good-die bytes — `NODES.md` §8 phase
//! 2's proof for `Psram`.
//!
//! The master is the bit-banging bench master `w25q128jv.rs` uses,
//! attached to the P2's own pins (`P56` the shared clock, `P57` the shared
//! chip enable, `P52`/`P53` `U302`'s serial in and out) with the processor
//! slot filled by the pin-only placeholder — so the four PSRAMs see exactly
//! what the module wires them to, shared enable and clock included, and the
//! other three answer nothing. Stepped mode per `TESTING.md` rule 9; its
//! own binary per rule 5.

mod machine_parts;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use embsim_board::{
    digital_drive, level_of, AttachError, Component, ComponentNetIo, Harness, IdleDrive, Level,
    NetState, PinDecl, PinHandle, PinKind, System,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::psram::{APS6404L_KGD_PASS, APS6404L_MF_ID};
use machine_parts::shipped_ec32mb_board;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

const MODULE: &str = "EC32MB";

// ============================================================
// The bit-banging master
// ============================================================

#[derive(Default)]
struct MasterPins {
    cs: Option<PinHandle>,
    clk: Option<PinHandle>,
    di: Option<PinHandle>,
    dout: Option<PinHandle>,
}

struct BitBangMaster {
    pins: [PinDecl; 4],
    handles: Arc<Mutex<MasterPins>>,
}

impl BitBangMaster {
    fn new(handles: Arc<Mutex<MasterPins>>) -> Self {
        let decl = |number, name, kind| PinDecl {
            number,
            name: Some(name),
            kind,
            stream: None,
            drive_impedance: None,
            idle: IdleDrive::KindDefault,
        };
        Self {
            pins: [
                decl("1", "CS", PinKind::DigitalOut),
                decl("2", "CLK", PinKind::DigitalOut),
                decl("3", "DI", PinKind::DigitalOut),
                decl("4", "DO", PinKind::DigitalIn),
            ],
            handles,
        }
    }
}

impl Component for BitBangMaster {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut slot = self.handles.lock().unwrap();
        slot.cs = Some(io.pin("CS")?);
        slot.clk = Some(io.pin("CLK")?);
        slot.di = Some(io.pin("DI")?);
        slot.dout = Some(io.pin("DO")?);
        Ok(())
    }
}

/// Drive a level and let the engine run: the drive is enqueued, resolved,
/// the part's sense delivered and its answering drive resolved in turn.
fn drive_and_settle(pin: &PinHandle, level: Level) {
    pin.set_drive(Some(digital_drive(level)));
    std::thread::sleep(Duration::from_millis(2));
}

fn send(pins: &MasterPins, byte: u8) {
    let (clk, di) = (pins.clk.as_ref().unwrap(), pins.di.as_ref().unwrap());
    for i in (0..8).rev() {
        let level = if (byte >> i) & 1 != 0 {
            Level::High
        } else {
            Level::Low
        };
        drive_and_settle(di, level);
        drive_and_settle(clk, Level::High);
        drive_and_settle(clk, Level::Low);
    }
}

/// Pulse, then sample — a bit-banging master's order.
fn recv(pins: &MasterPins) -> u8 {
    let (clk, dout) = (pins.clk.as_ref().unwrap(), pins.dout.as_ref().unwrap());
    let mut byte = 0u8;
    for _ in 0..8 {
        drive_and_settle(clk, Level::High);
        drive_and_settle(clk, Level::Low);
        byte = (byte << 1) | u8::from(level_of(dout.sense()) == Some(Level::High));
    }
    byte
}

// ============================================================
// Read ID over the module's nets
// ============================================================

#[rstest]
fn a_read_id_over_the_modules_nets_returns_the_datasheet_id() {
    behaviour!(Test {
        id: "psram.read-id-over-nets",
        covers: Some("models/src/psram.rs#Psram"),
        given: "the P2-EC32MB module with a bit-banging master on the first PSRAM's four \
                serial pins, issuing a Read ID with a dummy address",
    });
    expect!(
        "manufacturer-and-kgd",
        "the PSRAM answers 0D hex then 5D hex on its serial output: the manufacturer ID and \
         a passing known-good-die byte",
        "the datasheet's Read ID returns MF ID 0D, then KGD, and a part that shipped is \
         marked PASS"
    );
    expect!(
        "released-when-deselected",
        "the serial output floats before the part is selected and again after it is \
         deselected",
        "the output is high impedance until the part has data to present and after chip \
         enable rises"
    );
    expect!(
        "the-others-stay-silent",
        "the other three PSRAMs, which share the clock and chip enable, leave their serial \
         outputs floating throughout",
        "each shifts in its own serial input, and an input nothing drives is not a command"
    );

    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let harness = Harness::new()
        .connect_str("MASTER.CS", &format!("{MODULE}.U100.P57"))
        .expect("endpoints parse")
        .connect_str("MASTER.CLK", &format!("{MODULE}.U100.P56"))
        .expect("endpoints parse")
        .connect_str("MASTER.DI", &format!("{MODULE}.U100.P52"))
        .expect("endpoints parse")
        .connect_str("MASTER.DO", &format!("{MODULE}.U100.P53"))
        .expect("endpoints parse");
    let system = System::new()
        .board(MODULE, shipped_ec32mb_board())
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .harness(harness)
        .start()
        .expect("the module starts");
    std::thread::sleep(Duration::from_millis(20));

    let so = format!("{MODULE}.P2_IO53");
    let others = ["P2_IO49", "P2_IO45", "P2_IO41"].map(|n| format!("{MODULE}.{n}"));

    let pins = handles.lock().unwrap();
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);
    assert_eq!(
        system.net_state(&so),
        Some(NetState::Floating),
        "deselected: released"
    );

    drive_and_settle(pins.cs.as_ref().unwrap(), Level::Low);
    send(&pins, 0x9F);
    for _ in 0..3 {
        send(&pins, 0x00);
    }
    let id = [recv(&pins), recv(&pins)];
    assert!(
        matches!(system.net_state(&so), Some(NetState::Driven(_))),
        "selected and answering: driven"
    );
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);

    assert_eq!(
        id,
        [APS6404L_MF_ID, APS6404L_KGD_PASS],
        "MF ID 0Dh then KGD 5Dh, over four nets and an engine round trip per edge"
    );
    assert_eq!(id, [0x0D, 0x5D]);
    assert_eq!(
        system.net_state(&so),
        Some(NetState::Floating),
        "deselected again"
    );
    for other in &others {
        assert_eq!(
            system.net_state(other),
            Some(NetState::Floating),
            "{other}: a PSRAM whose serial input nothing drives answers nothing"
        );
    }
    drop(pins);
    drop(system);
}
