//! Serial PTY — creates a PTY pair for host ↔ firmware communication.
//!
//! Uses `openpty()` to create a master/slave PTY pair. The master FD is used
//! by the serial peripheral for firmware I/O. The slave path is symlinked to
//! a well-known location so host software can connect to it.
//!
//! The link is the only thing at that path a `Pty` ever removes: it replaces
//! a symlink it finds there (one an earlier run left), refuses a path that
//! holds anything else, and on drop removes the path only while it is still
//! the link this `Pty` made.

use nix::pty::{openpty, OpenptyResult};
use nix::sys::termios::{self, InputFlags, LocalFlags, OutputFlags, SetArg};
use std::fs;
use std::os::fd::{AsRawFd, OwnedFd};
use std::path::Path;
use tracing::info;

/// Holds the PTY pair file descriptors and paths.
pub struct Pty {
    /// Master FD — emulator reads/writes this.
    pub master: OwnedFd,
    /// Slave FD — kept open so the PTY stays alive.
    _slave: OwnedFd,
    /// Symlink path the host connects to (e.g. `/tmp/tty.sim_client`).
    pub symlink_path: String,
    /// The slave's device path, what the link points at: how `Drop` knows
    /// the link at `symlink_path` is still this PTY's.
    slave_path: String,
}

impl Pty {
    /// Create a new PTY pair and symlink the slave to `symlink_path`.
    ///
    /// A symlink already at `symlink_path` is replaced. Anything else there
    /// — a file, a directory — is refused with
    /// [`std::io::ErrorKind::AlreadyExists`] and left as it is: a PTY link
    /// never deletes what a path held.
    pub fn new(symlink_path: &str) -> std::io::Result<Self> {
        let link = Path::new(symlink_path);
        refuse_non_link(link)?;
        let OpenptyResult { master, slave } = openpty(None, None).map_err(std::io::Error::other)?;

        let master_fd = master;
        let slave_fd = slave;

        // Configure the slave for raw mode (no echo, no line buffering)
        let mut termios_config = termios::tcgetattr(&slave_fd).map_err(std::io::Error::other)?;
        termios::cfmakeraw(&mut termios_config);
        termios_config.local_flags &= !(LocalFlags::ECHO | LocalFlags::ICANON | LocalFlags::ISIG);
        termios_config.input_flags &= !(InputFlags::IXON | InputFlags::IXOFF | InputFlags::ICRNL);
        termios_config.output_flags &= !OutputFlags::OPOST;
        termios::tcsetattr(&slave_fd, SetArg::TCSANOW, &termios_config)
            .map_err(std::io::Error::other)?;

        // Get the slave device path
        let slave_path = get_slave_path(&slave_fd)?;

        // Set master FD to non-blocking
        set_nonblocking(&master_fd)?;

        // Create the symlink: replace a symlink already there (checked
        // again, in case the path changed since the check above), never
        // anything else. A file that appears in between makes `symlink`
        // fail rather than be replaced.
        refuse_non_link(link)?;
        if link.is_symlink() {
            let _ = fs::remove_file(link);
        }
        if let Some(parent) = link.parent() {
            let _ = fs::create_dir_all(parent);
        }
        std::os::unix::fs::symlink(&slave_path, link)?;

        info!(
            "PTY created: master_fd={}, slave={}",
            master_fd.as_raw_fd(),
            slave_path
        );
        info!("PTY symlinked: {} → {}", symlink_path, slave_path);

        Ok(Pty {
            master: master_fd,
            _slave: slave_fd,
            symlink_path: symlink_path.to_string(),
            slave_path,
        })
    }
}

impl Drop for Pty {
    /// Remove the link, while it is still this PTY's: a path that has since
    /// been replaced (another run's link, a file) is not this PTY's to
    /// remove.
    fn drop(&mut self) {
        let ours = fs::read_link(&self.symlink_path)
            .is_ok_and(|target| target == Path::new(&self.slave_path));
        if ours {
            let _ = fs::remove_file(&self.symlink_path);
            info!("PTY symlink removed: {}", self.symlink_path);
        }
    }
}

/// Refuse `link` when it holds anything but a symlink.
fn refuse_non_link(link: &Path) -> std::io::Result<()> {
    match fs::symlink_metadata(link) {
        Ok(meta) if !meta.file_type().is_symlink() => Err(std::io::Error::new(
            std::io::ErrorKind::AlreadyExists,
            format!(
                "{} exists and is not a symlink; a PTY link replaces only a symlink",
                link.display()
            ),
        )),
        _ => Ok(()),
    }
}

/// Get the device path of a slave PTY FD.
fn get_slave_path(fd: &OwnedFd) -> std::io::Result<String> {
    match nix::unistd::ttyname(fd) {
        Ok(path) => Ok(path.to_string_lossy().to_string()),
        Err(e) => Err(std::io::Error::other(e)),
    }
}

/// Set a file descriptor to non-blocking mode.
fn set_nonblocking(fd: &OwnedFd) -> std::io::Result<()> {
    use nix::fcntl::{fcntl, FcntlArg, OFlag};
    // nix 0.31's `fcntl` takes `impl AsFd`; `&OwnedFd` borrows the fd for the
    // call without taking ownership.
    let flags = fcntl(fd, FcntlArg::F_GETFL).map_err(std::io::Error::other)?;
    let mut flags = OFlag::from_bits_truncate(flags);
    flags.insert(OFlag::O_NONBLOCK);
    fcntl(fd, FcntlArg::F_SETFL(flags)).map_err(std::io::Error::other)?;
    Ok(())
}
