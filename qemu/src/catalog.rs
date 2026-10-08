//! The host computer as bench component kinds a project can name.
//!
//! [`VmCatalog`] is a [`Catalog`] of two bench component kinds, and
//! [`register`] adds it to a set, as the `embsim` command's set has it:
//!
//! ```
//! use embsim_board::Catalog;
//! use embsim_boards::catalog::CatalogSet;
//!
//! let mut set = CatalogSet::new();
//! embsim_qemu::catalog::register(&mut set).expect("two kinds, spelled as kinds are");
//! let kinds: Vec<String> = set
//!     .component_kinds()
//!     .into_iter()
//!     .map(|kind| kind.name.into_owned())
//!     .filter(|name| name.ends_with("-vm"))
//!     .collect();
//! assert_eq!(kinds, ["qemu-vm", "chrome-vm"]);
//! ```
//!
//! - **`qemu-vm`**: a virtual machine on the host's own system QEMU — any
//!   machine, accelerator and image it runs — its serial port on the
//!   component's pins.
//! - **`chrome-vm`**: the Chrome guest `guest/chrome/build.sh` builds, a
//!   headless Chromium whose Web Serial reaches the board through the same
//!   port, its DevTools forwarded to a host port a harness attaches to.
//!
//! Both are a [`QemuNode`]: the pins of a host's serial line at its own
//! rail (`TX`, `RX`, `VIO`, `GND`, as `host-serial` has them), the guest
//! metered by the board's clock a quantum at a time. Building the project
//! checks what can be checked without launching anything — the binary, the
//! image, the firmware — and the VM boots at the node's first slice, a
//! quantum after the run starts, with the board's clock held there while it
//! does; so `embsim check` boots nothing. Each kind reports what it runs and
//! how its guest was metered, and a guest that fails stops the run
//! ([`Report::failure`]).
//!
//! ```toml
//! [[component]]
//! name = "PC"
//! kind = "chrome-vm"
//! [component.options]
//! baud = 2000000
//! devtools_port = 9222
//! ```

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use embsim_board::report::instant;
use embsim_board::{
    Catalog, Component, ComponentRequest, ComponentSpec, KindInfo, PartOptions, ProjectError,
    Report, Reports,
};
use embsim_boards::catalog::CatalogSet;

use crate::chrome::{arch_of, resolve_on_path, uefi_firmware};
use crate::{
    default_image, free_port, Accel, ChromeGuest, Guest, NodeStats, QemuNode, QemuSpec,
    SerialDevice, DEFAULT_QUANTUM, DEFAULT_WARMUP_TIMEOUT, MAX_QUANTUM,
};

/// The catalog's name, as an error naming two catalogs prints it.
pub const NAME: &str = "embsim-qemu";

/// Add the two VM kinds to `set`.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    set.add(VmCatalog)
}

/// The `qemu-vm` and `chrome-vm` bench component kinds (module docs).
#[derive(Debug, Default, Clone, Copy)]
pub struct VmCatalog;

impl Catalog for VmCatalog {
    fn name(&self) -> &str {
        NAME
    }

    fn component_kinds(&self) -> Vec<KindInfo> {
        vec![
            KindInfo::new(
                "qemu-vm",
                "a virtual machine on the host's QEMU, its serial port on the wire and its \
                 clock metered by the board's",
            )
            .requires("baud", "115200", "the serial line's rate, framed 8N1"),
            KindInfo::new(
                "chrome-vm",
                "the Chrome guest in a virtual machine metered by the board's clock, its Web \
                 Serial on the wire and its DevTools on a host port",
            )
            .requires("baud", "2000000", "the serial line's rate, framed 8N1"),
        ]
    }

    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        match request.spec.kind.as_str() {
            "qemu-vm" => qemu_vm(request),
            "chrome-vm" => chrome_vm(request),
            other => Err(ProjectError::message(format!(
                "component {}: unknown kind {other:?}; the kinds of {NAME} are \"qemu-vm\" and \
                 \"chrome-vm\"",
                request.spec.name
            ))),
        }
    }
}

/// The entry being built: its name and kind, for its errors, and the
/// project's directory, for the files its options name.
struct Entry<'a> {
    spec: &'a ComponentSpec,
    dir: &'a Path,
    reports: &'a Reports,
}

impl Entry<'_> {
    /// An error about this entry: `component PC (kind "qemu-vm"): …`.
    fn error(&self, message: impl std::fmt::Display) -> ProjectError {
        ProjectError::message(format!(
            "component {} (kind {:?}): {message}",
            self.spec.name, self.spec.kind
        ))
    }

    /// A file option relative to the project, which must exist.
    fn existing_file(&self, name: &str, file: &str) -> Result<PathBuf, ProjectError> {
        let path = self.dir.join(file);
        if path.is_file() {
            Ok(path)
        } else {
            Err(self.error(format!("options.{name}: no file at {}", path.display())))
        }
    }
}

/// Split a request into its entry and its options.
fn split(request: ComponentRequest<'_>) -> (Entry<'_>, PartOptions) {
    let ComponentRequest {
        spec,
        options,
        dir,
        reports,
        ..
    } = request;
    (Entry { spec, dir, reports }, options)
}

/// The options both kinds take: the line, the metering, and the machine
/// QEMU emulates.
struct Common {
    baud: u32,
    quantum_ns: u64,
    binary: PathBuf,
    arch: String,
    accel: Accel,
    firmware: Option<PathBuf>,
    memory: Option<String>,
    cpus: Option<u32>,
}

impl Common {
    /// Read the options both kinds share, `memory` and `cpus` defaulting to
    /// `defaults`'.
    fn read(
        request: &Entry<'_>,
        options: &mut PartOptions,
        defaults: (Option<&str>, Option<u32>),
    ) -> Result<Self, ProjectError> {
        let baud = options.integer("baud")?.ok_or_else(|| {
            request.error(
                "options.baud is the serial line's rate, framed 8N1; a host names its rate, \
                 and the kind invents none (baud = 115200)",
            )
        })?;
        let baud = u32::try_from(baud)
            .ok()
            .filter(|baud| *baud > 0)
            .ok_or_else(|| request.error(format!("options.baud = {baud} is not a rate")))?;
        let quantum_ns = options
            .duration("quantum")?
            .unwrap_or(DEFAULT_QUANTUM.as_nanos() as u64);
        if quantum_ns == 0 {
            return Err(request.error(
                "options.quantum is how much virtual time passes between two slices of the \
                 guest; it is more than 0",
            ));
        }
        let max_ns = MAX_QUANTUM.as_nanos() as u64;
        if quantum_ns > max_ns {
            return Err(request.error(format!(
                "options.quantum is at most {}: the guest lags the board by up to a quantum, \
                 and each slice holds the board for one",
                instant(max_ns)
            )));
        }
        let binary = match options.string("qemu")? {
            Some(name) if name.contains('/') => request.dir.join(name),
            Some(name) => resolve_on_path(Path::new(&name)),
            None => resolve_on_path(Path::new(&format!(
                "qemu-system-{}",
                std::env::consts::ARCH
            ))),
        };
        if !binary.is_file() {
            return Err(request.error(format!(
                "no {} on PATH or at that path: the VM runs on the host's own system QEMU \
                 (Homebrew's qemu on macOS; qemu-system-arm or qemu-system-x86 on Debian), \
                 or options.qemu names one",
                binary.display()
            )));
        }
        let arch = arch_of(Some(&binary));
        let accel = match options.choice("accel", &["auto", "hvf", "kvm", "tcg"])? {
            None | Some("auto") => Accel::Auto,
            Some("hvf") => Accel::Hvf,
            Some("kvm") => Accel::Kvm,
            Some(_) => Accel::Tcg,
        };
        let firmware = options
            .string("firmware")?
            .map(|file| request.dir.join(file));
        if let Some(firmware) = &firmware {
            if !firmware.is_file() {
                return Err(request.error(format!(
                    "options.firmware: no file at {}",
                    firmware.display()
                )));
            }
        }
        let memory = options
            .string("memory")?
            .or_else(|| defaults.0.map(str::to_string));
        let cpus = match options.integer("cpus")? {
            None => defaults.1,
            Some(n) => Some(u32::try_from(n).ok().filter(|n| *n > 0).ok_or_else(|| {
                request.error(format!("options.cpus = {n} is not a number of vCPUs"))
            })?),
        };
        Ok(Self {
            baud,
            quantum_ns,
            binary,
            arch,
            accel,
            firmware,
            memory,
            cpus,
        })
    }
}

/// `qemu-vm`: any image on the host's QEMU.
fn qemu_vm(request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
    let (request, mut options) = split(request);
    let common = Common::read(&request, &mut options, (None, None))?;
    let image = match options.string("image")? {
        Some(file) => Some(request.existing_file("image", &file)?),
        None => None,
    };
    let machine = options.string("machine")?;
    let serial = match options.choice("serial", &["usb-ftdi", "uart"])? {
        None | Some("usb-ftdi") => SerialDevice::UsbFtdi,
        Some(_) => SerialDevice::Uart,
    };
    let agent = match options.value("agent") {
        None => false,
        Some(toml::Value::Boolean(agent)) => agent,
        Some(other) => {
            return Err(options.error(format!(
                "options.agent is true or false; {other} is a {}",
                other.type_str()
            )))
        }
    };
    let args = match options.value("args") {
        None => Vec::new(),
        Some(toml::Value::Array(items)) => items
            .into_iter()
            .map(|item| match item {
                toml::Value::String(arg) => Ok(arg),
                _ => Err(()),
            })
            .collect::<Result<Vec<String>, ()>>()
            .map_err(|()| {
                options.error("options.args is a list of QEMU's arguments, each a string")
            })?,
        Some(_) => {
            return Err(options.error("options.args is a list of QEMU's arguments, each a string"))
        }
    };
    let warmup_ns = options.duration("warmup")?;
    options.finish()?;

    // The machine QEMU emulates. On an aarch64 guest the GIC stays in QEMU
    // (`kernel-irqchip=off`): with Hypervisor.framework's own GIC, a vCPU
    // that executes WFI with no timer pending parks inside `hv_vcpu_run`
    // and QEMU's kick does not bring it back, so a `stop` waits forever
    // (measured: guests wedged minutes into a run, with exactly those
    // stacks), and the node stops a guest every quantum.
    let machine = machine.unwrap_or_else(|| match common.arch.as_str() {
        "aarch64" => "virt,kernel-irqchip=off".to_string(),
        _ => "q35".to_string(),
    });
    let firmware = match (&common.firmware, &image) {
        (Some(firmware), _) => Some(firmware.clone()),
        // An aarch64 disk boots UEFI, which `virt` does not carry.
        (None, Some(_)) if common.arch == "aarch64" => {
            Some(uefi_firmware(&common.binary).ok_or_else(|| {
                request.error(
                    "an aarch64 image boots UEFI, and no edk2-aarch64-code.fd was found beside \
                     QEMU or in the usual places; options.firmware names one",
                )
            })?)
        }
        (None, _) => None,
    };
    let mut spec = QemuSpec::new(&common.binary)
        .serial(serial)
        .agent(agent)
        .args(["-M", &machine])
        .args(common.accel.args(&common.arch));
    if let Some(memory) = &common.memory {
        spec = spec.args(["-m", memory]);
    }
    if let Some(cpus) = common.cpus {
        spec = spec.args(["-smp", &cpus.to_string()]);
    }
    if let Some(firmware) = &firmware {
        spec = spec.arg("-bios").arg(firmware);
    }
    if let Some(image) = &image {
        spec = spec.overlay_of(image);
    }
    spec = spec.args(&args);

    let warmup = warmup_ns.map(Duration::from_nanos);
    let boot: crate::Boot = Box::new(move || {
        let mut vm = spec.spawn().map_err(|e| e.to_string())?;
        if let Some(warmup) = warmup {
            vm.run_for(warmup)
                .map_err(|e| format!("warming the guest up: {e}"))?;
        }
        Ok(Box::new(vm) as Box<dyn Guest>)
    });
    let node =
        QemuNode::booting(boot, common.baud).with_quantum(Duration::from_nanos(common.quantum_ns));
    request.reports.add(VmReport {
        subject: request.spec.name.clone(),
        what: format!(
            "QEMU VM on {}{}",
            common.binary.display(),
            image
                .as_ref()
                .map(|image| format!(", booting {}", image.display()))
                .unwrap_or_default()
        ),
        devtools: None,
        baud: common.baud,
        quantum_ns: common.quantum_ns,
        stats: node.stats(),
        said: Said::default(),
    });
    Ok(Box::new(node))
}

/// `chrome-vm`: the Chrome guest, DevTools forwarded to a host port.
fn chrome_vm(request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
    let (request, mut options) = split(request);
    let common = Common::read(&request, &mut options, (Some("2G"), Some(2)))?;
    let image = match options.string("image")? {
        Some(file) => request.existing_file("image", &file)?,
        None => default_image()
            .filter(|image| image.is_file())
            .ok_or_else(|| {
                request.error(format!(
                    "no Chrome guest image at {}: qemu/guest/chrome/build.sh in embsim's \
                     source builds it there (minutes, once), or options.image names \
                     one",
                    default_image()
                        .map(|image| image.display().to_string())
                        .unwrap_or_else(|| "the cache (no HOME)".to_string())
                ))
            })?,
    };
    let port = match options.integer("devtools_port")? {
        None => free_port()
            .map_err(|e| request.error(format!("cannot find a free port for DevTools: {e}")))?,
        Some(port) => u16::try_from(port)
            .ok()
            .filter(|port| *port > 0)
            .ok_or_else(|| {
                request.error(format!("options.devtools_port = {port} is not a TCP port"))
            })?,
    };
    let warmup_timeout = options
        .duration("warmup_timeout")?
        .map_or(DEFAULT_WARMUP_TIMEOUT, Duration::from_nanos);
    options.finish()?;

    let mut chrome = ChromeGuest::new(&image)
        .binary(&common.binary)
        .accel(common.accel)
        .devtools_port(port)
        .warmup_timeout(warmup_timeout);
    if let Some(memory) = &common.memory {
        chrome = chrome.memory(memory);
    }
    if let Some(cpus) = common.cpus {
        chrome = chrome.cpus(cpus);
    }
    if let Some(firmware) = &common.firmware {
        chrome = chrome.firmware(firmware);
    }
    chrome.check().map_err(|e| request.error(e))?;

    let boot: crate::Boot = Box::new(move || {
        chrome
            .spawn()
            .map(|vm| Box::new(vm) as Box<dyn Guest>)
            .map_err(|e| e.to_string())
    });
    let node =
        QemuNode::booting(boot, common.baud).with_quantum(Duration::from_nanos(common.quantum_ns));
    request.reports.add(VmReport {
        subject: request.spec.name.clone(),
        what: format!(
            "Chrome guest {} on {}",
            image.display(),
            common.binary.display()
        ),
        devtools: Some(format!("http://127.0.0.1:{port}")),
        baud: common.baud,
        quantum_ns: common.quantum_ns,
        stats: node.stats(),
        said: Said::default(),
    });
    Ok(Box::new(node))
}

/// What a report has said so far.
#[derive(Debug, Default)]
struct Said {
    start: bool,
    boot: bool,
}

/// What a VM kind says in a run: what it runs at the first look, when its
/// guest booted, and at the end how the guest was metered and what crossed
/// the line.
struct VmReport {
    subject: String,
    what: String,
    devtools: Option<String>,
    baud: u32,
    quantum_ns: u64,
    stats: Arc<NodeStats>,
    said: Said,
}

impl Report for VmReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        let mut lines = Vec::new();
        if !std::mem::replace(&mut self.said.start, true) {
            lines.push(format!(
                "{}, {} baud 8N1, metered every {} of virtual time; it boots at the first slice",
                self.what,
                self.baud,
                instant(self.quantum_ns)
            ));
        }
        if let Some((took, at_ns)) = self.stats.booted() {
            if !std::mem::replace(&mut self.said.boot, true) {
                lines.push(format!(
                    "the guest booted in {:.3} s of host time, the board's clock held at {}; \
                     it runs only while the board's clock advances",
                    took.as_secs_f64(),
                    instant(at_ns)
                ));
                if let Some(url) = &self.devtools {
                    lines.push(format!("DevTools at {url}"));
                }
            }
        }
        lines
    }

    fn summary(&self) -> Vec<String> {
        let stats = &self.stats;
        let skew_us = stats.skew_ns() / 1_000;
        let apart = match skew_us {
            0 => "level with the board at the last slice".to_string(),
            us if us > 0 => format!("{us} µs behind the board at the last slice"),
            us => format!("{} µs ahead of the board at the last slice", -us),
        };
        let mut lines = vec![format!(
            "{} slices: the guest lived {} of the board's {} ({} booked from its own clock), \
             {apart}",
            stats.slices(),
            instant(stats.guest_ns()),
            instant(stats.virtual_ns()),
            stats.clocked(),
        )];
        lines.push(format!(
            "{} bytes from the guest, {} to it, {} framing errors",
            stats.from_guest(),
            stats.to_guest(),
            stats.framing_errors()
        ));
        if stats.from_guest() > 0 {
            lines.push(
                "the guest sent during the run: its bytes entered the line in the slice it sent \
                 them in, so this run is reproducible in what it sent, not in when"
                    .to_string(),
            );
        }
        if stats.shed() > 0 {
            lines.push(format!(
                "{} bytes were shed: the guest's line was unpowered (VIO read no voltage), or a \
                 side stopped reading",
                stats.shed()
            ));
        }
        if let Some(failure) = stats.failure() {
            lines.push(format!("failed: {failure}"));
        }
        lines
    }

    fn failure(&self) -> Option<String> {
        self.stats.failure()
    }
}
