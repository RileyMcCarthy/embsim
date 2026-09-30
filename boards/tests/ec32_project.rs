//! The EC32 carrier project is the module the ROM boot runs on, written as
//! a file. This test loads that file. The core is held in reset, so the
//! claim is the carrier's supply on `VIN_Edge` and ground on `GND`, which
//! is what the file's wires say.

use embsim_board::project::Project;
use embsim_board::NetState;
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, ClockMode};

#[test]
fn the_carrier_project_puts_five_volts_on_the_module() {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let project = Project::parse(include_str!("../projects/ec32-carrier.toml"))
        .expect("ec32-carrier.toml is a project");
    let system = project
        .instantiate(&StandardCatalog)
        .expect("the catalog builds the module")
        .hold_time()
        .start()
        .expect("the system starts");

    match system.net_state("EC32.VIN_Edge") {
        Some(NetState::Analog(volts)) => {
            assert!(
                (volts - 5.0).abs() < 1e-6,
                "J203's 5 V fingers are VIN_Edge; the file sources them at 5 V, got {volts}"
            );
        }
        other => panic!("VIN_Edge should be the carrier's 5 V, got {other:?}"),
    }
    match system.net_state("EC32.GND") {
        Some(NetState::Analog(volts)) => {
            assert!(
                volts.abs() < 1e-6,
                "J203's ground fingers are GND; the file sources them at 0 V, got {volts}"
            );
        }
        other => panic!("GND should be the carrier's 0 V, got {other:?}"),
    }
}
