//! Named simulation profiles — three **independent** axes, one process.
//!
//! embsim does not require every backend to be linked. Cargo features decide
//! what is **in the binary**; a [`SimProfile`] (or the three enums on their
//! own) decides what **this run** uses.
//!
//! | Axis | Type | Who applies it |
//! |---|---|---|
//! | Clock | [`ClockMode`] | `embsim-runtime` (`virtual_clock::init_mode`) |
//! | CPU | [`CpuBackend`] | consumer MCU `Component` (native HAL vs ISS); runtime rejects ISS until an ISS MCU is registered |
//! | Analog | [`AnalogBackend`] | `embsim-board` `System` (or a custom cluster solver) |
//!
//! The three can be mixed freely: native + no analog + stepped, ISS + spice
//! `.op` + free-running, etc. [`SimProfile::LIVE`] / [`SimProfile::SIL`] /
//! [`SimProfile::FULLSIM`] are **optional named bundles** for a CLI preset —
//! not the API.
//!
//! A digital-only consumer never depends on `embsim-board`'s `spice` feature
//! and never constructs [`AnalogBackend::SpiceOp`].

use crate::virtual_clock::ClockMode;
use std::fmt;
use std::str::FromStr;

/// How the MCU executes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CpuBackend {
    /// Host-compiled firmware + HAL trampolines. The playground default.
    #[default]
    Native,
    /// Instruction-set simulator (`IssCore`). Not implemented yet.
    Iss,
}

/// How analog clusters are solved.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum AnalogBackend {
    /// ngspice DC operating point. Cheap enough for the playground.
    #[default]
    SpiceOp,
    /// Event-driven ngspice `.tran` from `now` to the next engine deadline.
    /// Not implemented yet; `System::start` fails loudly if this is selected.
    SpiceTran {
        /// Maximum analog window (virtual µs) if the next digital event is later.
        max_step_us: u64,
    },
    /// No analog solver. Escalated clusters publish Floating. Use this when
    /// spice is not linked, or when the system has no analog content.
    Off,
}

/// One process's backend bundle. Apply at emulator/system start.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimProfile {
    /// Virtual-time mode.
    pub clock: ClockMode,
    /// MCU execution backend.
    pub cpu: CpuBackend,
    /// Analog cluster solver.
    pub analog: AnalogBackend,
}

impl SimProfile {
    /// Playground / Playwright: native firmware, spice `.op`, scaled wall time.
    pub const LIVE: Self = Self {
        clock: ClockMode::FreeRunning { speed: 1.0 },
        cpu: CpuBackend::Native,
        analog: AnalogBackend::SpiceOp,
    };

    /// Deterministic native SIL (D1 goldens, PR CI): native + `.op` + stepped.
    pub const SIL: Self = Self {
        clock: ClockMode::Stepped,
        cpu: CpuBackend::Native,
        analog: AnalogBackend::SpiceOp,
    };

    /// Full simulation: ISS + spice `.op` + stepped. ISS construction fails
    /// until the ISS crate exists; analog still runs `.op`.
    pub const FULLSIM: Self = Self {
        clock: ClockMode::Stepped,
        cpu: CpuBackend::Iss,
        analog: AnalogBackend::SpiceOp,
    };

    /// Override free-running speed. No-op on a stepped profile.
    pub fn with_speed(mut self, speed: f64) -> Self {
        if let ClockMode::FreeRunning { speed: s } = &mut self.clock {
            *s = speed;
        }
        self
    }

    /// Canonical name (`live`, `sil`, `fullsim`), or `custom` if the fields
    /// do not match a preset (e.g. LIVE with speed 5).
    pub fn name(self) -> &'static str {
        match self {
            Self::LIVE => "live",
            Self::SIL => "sil",
            Self::FULLSIM => "fullsim",
            _ => "custom",
        }
    }
}

impl Default for SimProfile {
    fn default() -> Self {
        Self::LIVE
    }
}

impl fmt::Display for SimProfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} (cpu={:?}, analog={:?}, clock={:?})",
            self.name(),
            self.cpu,
            self.analog,
            self.clock
        )
    }
}

impl fmt::Display for AnalogBackend {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AnalogBackend::SpiceOp => write!(f, "spice-op"),
            AnalogBackend::SpiceTran { max_step_us } => {
                write!(f, "spice-tran(max_step_us={max_step_us})")
            }
            AnalogBackend::Off => write!(f, "off"),
        }
    }
}

/// Unknown profile name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseProfileError {
    /// The token that did not match `live` / `sil` / `fullsim`.
    pub found: String,
}

impl fmt::Display for ParseProfileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "unknown simulation profile {:?}; expected live, sil, or fullsim",
            self.found
        )
    }
}

impl std::error::Error for ParseProfileError {}

impl FromStr for SimProfile {
    type Err = ParseProfileError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "live" => Ok(Self::LIVE),
            "sil" => Ok(Self::SIL),
            "fullsim" | "full" => Ok(Self::FULLSIM),
            other => Err(ParseProfileError {
                found: other.to_string(),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[rstest]
    #[case::live("live", SimProfile::LIVE)]
    #[case::sil("sil", SimProfile::SIL)]
    #[case::fullsim("fullsim", SimProfile::FULLSIM)]
    #[case::full("full", SimProfile::FULLSIM)]
    #[case::caps("LIVE", SimProfile::LIVE)]
    fn parse_presets(#[case] input: &str, #[case] expected: SimProfile) {
        assert_eq!(input.parse::<SimProfile>().unwrap(), expected);
    }

    #[rstest]
    fn unknown_name_is_an_error() {
        let err = "fast".parse::<SimProfile>().unwrap_err();
        assert_eq!(err.found, "fast");
        assert!(err.to_string().contains("live"));
    }

    #[rstest]
    fn with_speed_only_affects_free_running() {
        let live = SimProfile::LIVE.with_speed(5.0);
        assert_eq!(live.clock, ClockMode::FreeRunning { speed: 5.0 });
        assert_eq!(live.name(), "custom");
        let sil = SimProfile::SIL.with_speed(5.0);
        assert_eq!(sil, SimProfile::SIL);
    }

    #[rstest]
    fn default_is_live() {
        assert_eq!(SimProfile::default(), SimProfile::LIVE);
    }

    #[rstest]
    fn presets_keep_spice_op() {
        for p in [SimProfile::LIVE, SimProfile::SIL, SimProfile::FULLSIM] {
            assert_eq!(p.analog, AnalogBackend::SpiceOp);
        }
        assert_eq!(SimProfile::FULLSIM.cpu, CpuBackend::Iss);
        assert_eq!(SimProfile::LIVE.cpu, CpuBackend::Native);
    }
}
