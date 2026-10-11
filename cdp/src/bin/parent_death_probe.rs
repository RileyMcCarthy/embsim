//! Proves [`embsim_cdp::arm_parent_death`]: a parent that is SIGKILLed takes
//! the process group it started with it, and the profile directory goes too.
//!
//! A library test runs inside the harness's threads, so this probe is its
//! own process. The fork below happens before any other thread exists.

fn main() {
    #[cfg(target_os = "linux")]
    {
        if let Err(error) = linux::probe() {
            eprintln!("{error}");
            std::process::exit(1);
        }
    }
    #[cfg(not(target_os = "linux"))]
    eprintln!("parent-death is proved on Linux");
}

#[cfg(target_os = "linux")]
mod linux {
    use std::fs;
    use std::io;
    use std::process::{Command, Stdio};
    use std::time::{Duration, Instant};

    pub fn probe() -> io::Result<()> {
        let scratch = std::env::temp_dir().join(format!("embsim-cdp-death-{}", std::process::id()));
        let _ = fs::remove_dir_all(&scratch);
        fs::create_dir_all(&scratch)?;
        let profile = scratch.join("profile");
        fs::create_dir_all(&profile)?;
        fs::write(profile.join("marker"), b"alive")?;
        let pidfile = scratch.join("supervisor");

        let parent = unsafe { libc::fork() };
        if parent < 0 {
            return Err(io::Error::last_os_error());
        }
        if parent == 0 {
            let code = child(&profile, &pidfile);
            unsafe { libc::_exit(code) };
        }

        let mut status = 0;
        unsafe { libc::waitpid(parent, &mut status, 0) };
        let text = fs::read_to_string(&pidfile)?;
        let supervisor: i32 = text
            .trim()
            .parse()
            .map_err(|error| io::Error::other(format!("supervisor pid {text:?}: {error}")))?;

        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            let group_gone = unsafe { libc::kill(-supervisor, 0) } != 0;
            if group_gone && !profile.exists() {
                let _ = fs::remove_dir_all(&scratch);
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        Err(io::Error::other(format!(
            "supervisor {supervisor} still in its group or profile remains at {}",
            profile.display()
        )))
    }

    fn child(profile: &std::path::Path, pidfile: &std::path::Path) -> i32 {
        let mut command = Command::new("/bin/sleep");
        command
            .arg("30")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        embsim_cdp::arm_parent_death(&mut command, profile);
        let child = match command.spawn() {
            Ok(child) => child,
            Err(error) => {
                let _ = fs::write(pidfile, format!("spawn {error}"));
                return 1;
            }
        };
        if fs::write(pidfile, child.id().to_string()).is_err() {
            return 1;
        }
        unsafe { libc::raise(libc::SIGKILL) };
        0
    }
}
