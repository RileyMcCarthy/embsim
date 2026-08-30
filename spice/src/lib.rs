//! In-process ngspice shared-library session.
//!
//! libngspice is process-global and not re-entrant, so every call takes one
//! mutex. The board engine is already a single writer; cargo-test parallelism
//! is serialized here. This crate runs **operating-point** (`.op`) analysis:
//! a deck in, node voltages out. Transient windows are a later slice.

use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_short, c_void};
use std::ptr;
use std::sync::{Mutex, Once};

// ============================================================
// FFI (sharedspice.h — the subset we call)
// ============================================================

#[repr(C)]
struct VectorInfo {
    v_name: *mut c_char,
    v_type: c_int,
    v_flags: c_short,
    v_realdata: *mut f64,
    _v_compdata: *mut c_void,
    v_length: c_int,
}

extern "C" {
    fn ngSpice_Init(
        printfcn: Option<extern "C" fn(*mut c_char, c_int, *mut c_void) -> c_int>,
        statfcn: Option<extern "C" fn(*mut c_char, c_int, *mut c_void) -> c_int>,
        ngexit: Option<extern "C" fn(c_int, bool, bool, c_int, *mut c_void) -> c_int>,
        sdata: Option<extern "C" fn(*mut c_void, c_int, c_int, *mut c_void) -> c_int>,
        sinitdata: Option<extern "C" fn(*mut c_void, c_int, *mut c_void) -> c_int>,
        bgtrun: Option<extern "C" fn(bool, c_int, *mut c_void) -> c_int>,
        user_data: *mut c_void,
    ) -> c_int;
    fn ngSpice_Command(command: *mut c_char) -> c_int;
    fn ngSpice_Circ(circarray: *mut *mut c_char) -> c_int;
    fn ngGet_Vec_Info(vecname: *mut c_char) -> *mut VectorInfo;
    fn ngSpice_CurPlot() -> *mut c_char;
    fn ngSpice_AllVecs(plotname: *mut c_char) -> *mut *mut c_char;
}

// ============================================================
// Callbacks — never panic; never call back into ngspice.
// ============================================================

extern "C" fn send_char(msg: *mut c_char, _id: c_int, _user: *mut c_void) -> c_int {
    if msg.is_null() {
        return 0;
    }
    // SAFETY: ngspice owns `msg` for the duration of the callback.
    let text = unsafe { CStr::from_ptr(msg) }.to_string_lossy();
    // ngspice prints `Error on line...` / `fatal error`; a missing spinit is
    // a Warning and must not trip this.
    let lower = text.to_ascii_lowercase();
    if lower.contains("error on") || lower.contains("fatal") || lower.contains("stderr error") {
        if let Ok(mut err) = LAST_ERROR.lock() {
            err.push(text.into_owned());
        }
    }
    0
}

extern "C" fn send_stat(_msg: *mut c_char, _id: c_int, _user: *mut c_void) -> c_int {
    0
}

extern "C" fn controlled_exit(
    status: c_int,
    _immediate: bool,
    _quit: bool,
    _id: c_int,
    _user: *mut c_void,
) -> c_int {
    if let Ok(mut err) = LAST_ERROR.lock() {
        err.push(format!("ngspice controlled_exit status={status}"));
    }
    0
}

// ============================================================
// Session
// ============================================================

static INIT: Once = Once::new();
static SESSION: Mutex<()> = Mutex::new(());
static LAST_ERROR: Mutex<Vec<String>> = Mutex::new(Vec::new());

/// Failure of an operating-point run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SpiceError {
    /// Deck contained a NUL byte (not a valid SPICE card).
    NulInDeck {
        /// Offending card.
        card: String,
    },
    /// ngspice rejected the circuit or `.op` failed.
    Analysis {
        /// Lines captured from ngspice's error callbacks.
        messages: Vec<String>,
    },
}

impl std::fmt::Display for SpiceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpiceError::NulInDeck { card } => write!(f, "NUL byte in SPICE card {card:?}"),
            SpiceError::Analysis { messages } => {
                write!(f, "ngspice .op failed: {}", messages.join("; "))
            }
        }
    }
}

impl std::error::Error for SpiceError {}

fn lock_session() -> std::sync::MutexGuard<'static, ()> {
    SESSION.lock().unwrap_or_else(|p| p.into_inner())
}

fn take_errors() -> Vec<String> {
    LAST_ERROR
        .lock()
        .map(|mut v| std::mem::take(&mut *v))
        .unwrap_or_default()
}

fn ensure_init() {
    INIT.call_once(|| {
        // SAFETY: first (and only) ngspice init in this process. Callbacks
        // never panic and never re-enter. Do not call `ngSpice_nospinit`:
        // Homebrew's libngspice 47 segfaults there; a missing `spinit` is a
        // warning, not a crash.
        unsafe {
            ngSpice_Init(
                Some(send_char),
                Some(send_stat),
                Some(controlled_exit),
                None,
                None,
                None,
                ptr::null_mut(),
            );
        }
        let _ = command("set nomoremode");
        take_errors();
    });
}

fn command(cmd: &str) -> i32 {
    let c = CString::new(cmd).expect("ngspice command is UTF-8 without NUL");
    // SAFETY: `c` lives for the call; the command is executed immediately.
    unsafe { ngSpice_Command(c.as_ptr() as *mut c_char) }
}

/// Run a DC operating-point analysis on `cards` (element lines, no title/`.end`).
///
/// Returns a map of **node name → voltage**. Internal source nodes (`vs*`)
/// and branch currents are omitted. Ground (`0`) is not in the map (it is 0 V).
pub fn operating_point(cards: &[&str]) -> Result<HashMap<String, f64>, SpiceError> {
    let _guard = lock_session();
    ensure_init();
    take_errors();

    let mut lines: Vec<CString> = Vec::with_capacity(cards.len() + 2);
    lines.push(cstring("embsim cluster")?);
    for card in cards {
        lines.push(cstring(card)?);
    }
    lines.push(cstring(".end")?);

    let mut ptrs: Vec<*mut c_char> = lines.iter().map(|s| s.as_ptr() as *mut c_char).collect();
    ptrs.push(ptr::null_mut());

    let _ = command("destroy all");
    take_errors();

    // SAFETY: `ptrs` is NULL-terminated; `lines` owns the strings for the call.
    let circ_rc = unsafe { ngSpice_Circ(ptrs.as_mut_ptr()) };
    let mut errors = take_errors();
    if circ_rc != 0 {
        if errors.is_empty() {
            errors.push(format!("ngSpice_Circ returned {circ_rc}"));
        }
        return Err(SpiceError::Analysis { messages: errors });
    }

    let op_rc = command("op");
    errors = take_errors();
    if op_rc != 0 {
        if errors.is_empty() {
            errors.push(format!("ngSpice_Command(\"op\") returned {op_rc}"));
        }
        return Err(SpiceError::Analysis { messages: errors });
    }
    if !errors.is_empty() {
        return Err(SpiceError::Analysis { messages: errors });
    }

    Ok(read_node_voltages())
}

fn cstring(s: &str) -> Result<CString, SpiceError> {
    CString::new(s).map_err(|_| SpiceError::NulInDeck {
        card: s.to_string(),
    })
}

fn read_node_voltages() -> HashMap<String, f64> {
    let mut out = HashMap::new();
    // SAFETY: ngspice owns the plot-name string for the session.
    let plot = unsafe { ngSpice_CurPlot() };
    if plot.is_null() {
        return out;
    }
    // SAFETY: NULL-terminated array of ngspice-owned strings.
    let vecs = unsafe { ngSpice_AllVecs(plot) };
    if vecs.is_null() {
        return out;
    }
    let mut i = 0isize;
    loop {
        let p = unsafe { *vecs.offset(i) };
        if p.is_null() {
            break;
        }
        let name = unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned();
        i += 1;
        if name.contains("#branch") || name.eq_ignore_ascii_case("time") {
            continue;
        }
        if let Some(v) = vec_last_real(&name) {
            if v.is_finite() {
                out.insert(name, v);
            }
        }
    }
    out
}

fn vec_last_real(name: &str) -> Option<f64> {
    let c = CString::new(name).ok()?;
    // SAFETY: ngspice owns the vector for the current plot; we only read.
    let info = unsafe { ngGet_Vec_Info(c.as_ptr() as *mut c_char) };
    if info.is_null() {
        return None;
    }
    let info = unsafe { &*info };
    if info.v_realdata.is_null() || info.v_length <= 0 {
        return None;
    }
    let last = (info.v_length as usize).saturating_sub(1);
    // SAFETY: v_realdata has v_length entries.
    Some(unsafe { *info.v_realdata.add(last) })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[rstest]
    fn divider_1k_1k_at_2v_is_1v() {
        let v = operating_point(&["R1 n1 n2 1k", "R2 n2 0 1k", "V1 n1 0 2.0"]).unwrap();
        let mid = v
            .get("n2")
            .copied()
            .or_else(|| v.get("v(n2)").copied())
            .unwrap_or_else(|| panic!("n2 missing from {v:?}"));
        assert!((mid - 1.0).abs() < 1e-6, "n2={mid}, all={v:?}");
        let top = v.get("n1").copied().or_else(|| v.get("v(n1)").copied());
        if let Some(top) = top {
            assert!((top - 2.0).abs() < 1e-6, "n1={top}");
        }
    }

    #[rstest]
    fn wheatstone_unbalanced_matches_ratiometric_equation() {
        let r = 350.0;
        let delta = 0.004;
        let v_exc = 3.3;
        let cards = [
            format!("R1 nexc nsigp {r}"),
            format!("R2 nsigp ngnd {}", r * (1.0 + delta)),
            format!("R3 nexc nsign {r}"),
            format!("R4 nsign ngnd {r}"),
            format!("Vexc nexc ngnd {v_exc}"),
            "Vgnd ngnd 0 0".to_string(),
        ];
        let refs: Vec<&str> = cards.iter().map(String::as_str).collect();
        let v = operating_point(&refs).unwrap();
        let pick = |k: &str| {
            v.get(k)
                .copied()
                .or_else(|| v.get(&format!("v({k})")).copied())
                .unwrap_or_else(|| panic!("{k} missing from {v:?}"))
        };
        let sigp = pick("nsigp");
        let sign = pick("nsign");
        let expected_p = v_exc * (1.0 + delta) / (2.0 + delta);
        assert!(
            (sigp - expected_p).abs() < 1e-6,
            "sigp={sigp} expected={expected_p}"
        );
        assert!((sign - v_exc / 2.0).abs() < 1e-6, "sign={sign}");
    }

    #[rstest]
    fn nul_card_is_a_named_error() {
        let err = operating_point(&["R1 n1 n2 1k\0oops"]).unwrap_err();
        assert!(matches!(err, SpiceError::NulInDeck { .. }));
    }
}
