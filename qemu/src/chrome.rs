//! A guest that is a browser.
//!
//! [`ChromeGuest`] launches the image that `guest/chrome/build.sh` produces
//! — a Linux with a headless Chromium listening for DevTools, a relay that
//! exposes that port to the host, the agent that answers
//! [`Guest::clock_ns`](crate::Guest::clock_ns), and a managed policy that
//! grants the host's web app its serial port without a picker — and warms it
//! up on host time until DevTools answers, then freezes it. What comes out
//! is a [`Guest`] for a [`QemuNode`](crate::QemuNode) plus the address a
//! test harness attaches to with `connectOverCDP`.
//!
//! The image's conventions (what listens where) are documented next to its
//! build script; this module only knows them.

use std::fmt;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::os::fd::RawFd;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::guest::Guest;
use crate::vm::{QemuSpec, QemuVm, SerialDevice, SpawnError};

/// The port the image's DevTools relay listens on inside the guest.
pub const GUEST_DEVTOOLS_PORT: u16 = 9223;

/// How long a fresh boot may take before Chrome answers (default).
pub const DEFAULT_WARMUP_TIMEOUT: Duration = Duration::from_secs(180);

/// Why [`ChromeGuest::spawn`] failed.
#[derive(Debug)]
pub enum ChromeError {
    /// The image file does not exist.
    NoImage(PathBuf),
    /// No UEFI firmware for this guest architecture was found; pass one with
    /// [`ChromeGuest::firmware`].
    NoFirmware,
    /// QEMU could not be launched.
    Spawn(SpawnError),
    /// The guest booted but Chrome never answered, or QEMU died meanwhile.
    Warmup(io::Error),
}

impl fmt::Display for ChromeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoImage(p) => write!(f, "no guest image at {}", p.display()),
            Self::NoFirmware => write!(
                f,
                "no UEFI firmware found for the guest; set ChromeGuest::firmware"
            ),
            Self::Spawn(e) => write!(f, "{e}"),
            Self::Warmup(e) => write!(f, "warming the guest up: {e}"),
        }
    }
}

impl std::error::Error for ChromeError {}

impl From<SpawnError> for ChromeError {
    fn from(e: SpawnError) -> Self {
        Self::Spawn(e)
    }
}

/// Where a harness attaches to the guest's Chrome.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DevTools {
    /// Host-side TCP port forwarded to the guest's DevTools relay.
    pub port: u16,
}

impl DevTools {
    /// The URL for `connectOverCDP` / `/json/version`.
    pub fn url(&self) -> String {
        format!("http://127.0.0.1:{}", self.port)
    }

    /// Whether Chrome answers `/json/version` right now (only while the
    /// guest runs).
    pub fn is_up(&self) -> bool {
        probe_devtools(self.port)
    }
}

/// The accelerator to run the guest under.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Accel {
    /// Pick by host: HVF on macOS, KVM on Linux when `/dev/kvm` is usable,
    /// else TCG.
    Auto,
    /// Apple's Hypervisor.framework.
    Hvf,
    /// Linux KVM.
    Kvm,
    /// Software emulation; slow, works anywhere.
    Tcg,
}

impl Accel {
    fn resolve(self) -> Accel {
        match self {
            Accel::Auto => {
                if cfg!(target_os = "macos") {
                    Accel::Hvf
                } else if cfg!(target_os = "linux")
                    && std::fs::OpenOptions::new()
                        .read(true)
                        .write(true)
                        .open("/dev/kvm")
                        .is_ok()
                {
                    Accel::Kvm
                } else {
                    Accel::Tcg
                }
            }
            other => other,
        }
    }

    fn args(self, arch: &str) -> Vec<String> {
        match self {
            Accel::Hvf => vec!["-accel".into(), "hvf".into(), "-cpu".into(), "host".into()],
            Accel::Kvm => vec!["-accel".into(), "kvm".into(), "-cpu".into(), "host".into()],
            Accel::Tcg => {
                let cpu = if arch == "aarch64" {
                    "cortex-a72"
                } else {
                    "max"
                };
                vec!["-accel".into(), "tcg".into(), "-cpu".into(), cpu.into()]
            }
            Accel::Auto => unreachable!("resolved before use"),
        }
    }
}

/// How to launch the browser image.
#[derive(Debug, Clone)]
pub struct ChromeGuest {
    image: PathBuf,
    binary: Option<PathBuf>,
    firmware: Option<PathBuf>,
    accel: Accel,
    memory: String,
    cpus: u32,
    devtools_port: u16,
    warmup_timeout: Duration,
}

impl ChromeGuest {
    /// Launch the image at `image` (built by `guest/chrome/build.sh`).
    ///
    /// Defaults: the host's architecture, `qemu-system-<arch>` from `PATH`,
    /// the accelerator picked by host, 2 GiB, 2 vCPUs, a free DevTools port,
    /// [`DEFAULT_WARMUP_TIMEOUT`].
    pub fn new(image: impl Into<PathBuf>) -> Self {
        Self {
            image: image.into(),
            binary: None,
            firmware: None,
            accel: Accel::Auto,
            memory: "2G".into(),
            cpus: 2,
            devtools_port: 0,
            warmup_timeout: DEFAULT_WARMUP_TIMEOUT,
        }
    }

    /// The `qemu-system-*` binary (default: `qemu-system-<host arch>`).
    pub fn binary(mut self, binary: impl Into<PathBuf>) -> Self {
        self.binary = Some(binary.into());
        self
    }

    /// UEFI firmware image for an aarch64 guest (default: found next to the
    /// binary or in the usual system locations).
    pub fn firmware(mut self, firmware: impl Into<PathBuf>) -> Self {
        self.firmware = Some(firmware.into());
        self
    }

    /// The accelerator (default: [`Accel::Auto`]).
    pub fn accel(mut self, accel: Accel) -> Self {
        self.accel = accel;
        self
    }

    /// Guest memory, in QEMU's `-m` syntax (default `2G`).
    pub fn memory(mut self, memory: impl Into<String>) -> Self {
        self.memory = memory.into();
        self
    }

    /// Guest vCPUs (default 2).
    pub fn cpus(mut self, cpus: u32) -> Self {
        self.cpus = cpus.max(1);
        self
    }

    /// Host port to forward to the guest's DevTools (default: any free port).
    pub fn devtools_port(mut self, port: u16) -> Self {
        self.devtools_port = port;
        self
    }

    /// How long to let the guest boot before giving up (default
    /// [`DEFAULT_WARMUP_TIMEOUT`]).
    pub fn warmup_timeout(mut self, timeout: Duration) -> Self {
        self.warmup_timeout = timeout;
        self
    }

    /// The guest architecture, from the binary name or the host.
    fn arch(&self) -> String {
        self.binary
            .as_ref()
            .and_then(|b| b.file_name())
            .and_then(|n| n.to_str())
            .and_then(|n| n.strip_prefix("qemu-system-"))
            .map(str::to_string)
            .unwrap_or_else(|| std::env::consts::ARCH.to_string())
    }

    fn find_firmware(&self, binary: &Path, arch: &str) -> Option<PathBuf> {
        if let Some(fw) = &self.firmware {
            return Some(fw.clone());
        }
        if arch != "aarch64" {
            return None; // x86_64 boots SeaBIOS by default; nothing to pass
        }
        let mut candidates = Vec::new();
        if let Some(prefix) = binary.parent().and_then(Path::parent) {
            candidates.push(prefix.join("share/qemu/edk2-aarch64-code.fd"));
        }
        candidates.extend(
            [
                "/opt/homebrew/share/qemu/edk2-aarch64-code.fd",
                "/usr/local/share/qemu/edk2-aarch64-code.fd",
                "/usr/share/qemu/edk2-aarch64-code.fd",
                "/usr/share/AAVMF/AAVMF_CODE.fd",
            ]
            .into_iter()
            .map(PathBuf::from),
        );
        candidates.into_iter().find(|p| p.is_file())
    }

    /// Launch the guest, let it boot until Chrome answers, and freeze it.
    pub fn spawn(self) -> Result<ChromeVm, ChromeError> {
        if !self.image.is_file() {
            return Err(ChromeError::NoImage(self.image.clone()));
        }
        let arch = self.arch();
        let binary = self
            .binary
            .clone()
            .unwrap_or_else(|| PathBuf::from(format!("qemu-system-{arch}")));
        let binary = resolve_on_path(&binary);
        let port = if self.devtools_port == 0 {
            free_port()?
        } else {
            self.devtools_port
        };

        let mut spec = QemuSpec::new(&binary)
            .overlay_of(&self.image)
            .agent(true)
            .serial(SerialDevice::UsbFtdi)
            .args(["-m", &self.memory, "-smp", &self.cpus.to_string()])
            .args(self.accel.resolve().args(&arch))
            // No IPv6 on the guest's NIC: slirp's router advertisements land a
            // SLAAC address on the interface minutes after boot, Chrome sees a
            // network change and aborts every request in flight
            // (net::ERR_NETWORK_CHANGED) — a page mid-load never finishes.
            .args([
                "-netdev",
                &format!(
                    "user,id=embsim-net,ipv6=off,hostfwd=tcp:127.0.0.1:{port}-:{GUEST_DEVTOOLS_PORT}"
                ),
                "-device",
                "virtio-net-pci,netdev=embsim-net",
            ])
            // The guest's console, for a post-mortem: `console.log` next to
            // `qemu.log` in the VM's working directory.
            .args(["-serial", "file:console.log"]);
        spec = match arch.as_str() {
            "aarch64" => {
                let fw = self
                    .find_firmware(&binary, &arch)
                    .ok_or(ChromeError::NoFirmware)?;
                // Interrupts stay in QEMU, not in Hypervisor.framework. With
                // the in-framework GIC (the default on macOS 15+), a vCPU
                // that executes WFI with no timer pending parks inside
                // `hv_vcpu_run` (`VcpuStateManager::wait_for_interrupt`) and
                // QEMU's `hv_vcpus_exit` kick does not bring it back, so a
                // `stop` waits in `pause_all_vcpus` until some interrupt
                // arrives — which, with QEMU's main loop stuck there, never
                // comes. Measured: guests wedged minutes into a run, every
                // time, with exactly those stacks. With the GIC in QEMU a WFI
                // exits to QEMU's own wait, which its kick interrupts.
                spec.args(["-M", "virt,kernel-irqchip=off"])
                    .arg("-bios")
                    .arg(fw)
            }
            _ => spec.args(["-M", "q35"]),
        };

        let mut vm = spec.spawn()?;
        let devtools = DevTools { port };
        let booted = vm
            .run_until(|| devtools.is_up(), self.warmup_timeout)
            .map_err(ChromeError::Warmup)?;
        tracing::info!(
            ?booted,
            url = devtools.url(),
            "Chrome guest ready and frozen"
        );
        Ok(ChromeVm { vm, devtools })
    }
}

/// A frozen, warmed-up browser guest.
pub struct ChromeVm {
    vm: QemuVm,
    devtools: DevTools,
}

impl fmt::Debug for ChromeVm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ChromeVm")
            .field("vm", &self.vm)
            .field("devtools", &self.devtools)
            .finish()
    }
}

impl ChromeVm {
    /// Where to attach.
    pub fn devtools(&self) -> &DevTools {
        &self.devtools
    }

    /// The underlying VM.
    pub fn vm(&self) -> &QemuVm {
        &self.vm
    }

    /// The underlying VM, mutably.
    pub fn vm_mut(&mut self) -> &mut QemuVm {
        &mut self.vm
    }

    /// Take the VM out.
    pub fn into_vm(self) -> QemuVm {
        self.vm
    }
}

impl Guest for ChromeVm {
    fn resume(&mut self) -> io::Result<()> {
        self.vm.resume()
    }

    fn pause(&mut self) -> io::Result<()> {
        self.vm.pause()
    }

    fn serial_fd(&self) -> RawFd {
        self.vm.serial_fd()
    }

    fn serial_attached(&self) -> bool {
        self.vm.serial_attached()
    }

    fn set_serial_attached(&mut self, attached: bool) -> io::Result<()> {
        self.vm.set_serial_attached(attached)
    }

    fn clock_ns(&mut self) -> Option<u64> {
        self.vm.clock_ns()
    }
}

/// `GET /json/version` and look for the browser's WebSocket URL.
fn probe_devtools(port: u16) -> bool {
    let addr: SocketAddr = ([127, 0, 0, 1], port).into();
    let Ok(mut s) = TcpStream::connect_timeout(&addr, Duration::from_millis(500)) else {
        return false;
    };
    let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
    let _ = s.set_write_timeout(Some(Duration::from_secs(2)));
    // Chrome's DevTools server wants HTTP/1.1 with the port in `Host`, and
    // does not always close after answering: read until the marker shows
    // up, not until EOF.
    let request = format!(
        "GET /json/version HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nConnection: close\r\n\r\n"
    );
    if s.write_all(request.as_bytes()).is_err() {
        return false;
    }
    let mut body = Vec::new();
    let mut chunk = [0u8; 4096];
    loop {
        match s.read(&mut chunk) {
            Ok(0) | Err(_) => break,
            Ok(n) => {
                body.extend_from_slice(&chunk[..n]);
                if body.windows(20).any(|w| w == b"webSocketDebuggerUrl") {
                    return true;
                }
            }
        }
    }
    false
}

fn free_port() -> Result<u16, ChromeError> {
    let listener = TcpListener::bind(("127.0.0.1", 0)).map_err(ChromeError::Warmup)?;
    let port = listener.local_addr().map_err(ChromeError::Warmup)?.port();
    Ok(port)
}

/// A bare binary name is fine for `Command`, but the firmware search wants
/// to know where the installation lives.
fn resolve_on_path(binary: &Path) -> PathBuf {
    if binary.components().count() > 1 {
        return binary.to_path_buf();
    }
    std::env::var_os("PATH")
        .map(|path| {
            std::env::split_paths(&path)
                .map(|dir| dir.join(binary))
                .find(|candidate| candidate.is_file())
        })
        .unwrap_or(None)
        .unwrap_or_else(|| binary.to_path_buf())
}
