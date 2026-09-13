//! ISS-vs-silicon goldens.
//!
//! Silicon runs a program and prints observations. Those lines are the
//! specification. An instruction-set simulator must emit the same text for
//! the same image (report goldens) or the same `CASE`/`IN`/`OUT` record
//! (one-instruction probe goldens).
//!
//! This crate is ISA-agnostic: it does not know GETBYTE or loadp2. A CPU
//! adapter supplies the binary, the loader, and the ISS. The format is the
//! shared piece.

use std::collections::BTreeMap;
use std::fmt;

/// One probe record: named input map → named output map.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Record {
    pub name: String,
    pub input: BTreeMap<String, String>,
    pub output: BTreeMap<String, String>,
}

/// Line-oriented report (PASS/FAIL/RESULT/DUMP/…). Comments (`#`) dropped.
pub fn parse_report(text: &str) -> Vec<String> {
    text.replace('\r', "\n")
        .lines()
        .map(str::trim)
        .filter(|line| {
            !line.is_empty()
                && !line.starts_with('#')
                && (line.starts_with("PASS ")
                    || line.starts_with("FAIL ")
                    || line.starts_with("RESULT ")
                    || line.starts_with("DUMP ")
                    || line.starts_with("HUB ")
                    || line.starts_with("LOCKS")
                    || line.starts_with("COGS")
                    || line.starts_with("P2CORE-HW")
                    || line.starts_with("CLKFREQ ")
                    || line.starts_with("PROBE")
                    || line.starts_with("READY"))
        })
        .map(str::to_string)
        .collect()
}

/// Parse `CASE` / `IN` / `OUT` records. Unknown keys are kept as strings.
pub fn parse_records(text: &str) -> Result<Vec<Record>, String> {
    let mut records = Vec::new();
    let mut cur: Option<Record> = None;
    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(name) = line.strip_prefix("CASE ") {
            if let Some(r) = cur.take() {
                records.push(r);
            }
            cur = Some(Record {
                name: name.to_string(),
                input: BTreeMap::new(),
                output: BTreeMap::new(),
            });
        } else if let Some(rest) = line.strip_prefix("IN ") {
            let r = cur.as_mut().ok_or("IN without CASE")?;
            parse_kv(rest, &mut r.input);
        } else if let Some(rest) = line.strip_prefix("OUT ") {
            let r = cur.as_mut().ok_or("OUT without CASE")?;
            parse_kv(rest, &mut r.output);
        } else {
            return Err(format!("unrecognized golden line: {line}"));
        }
    }
    if let Some(r) = cur {
        records.push(r);
    }
    Ok(records)
}

fn parse_kv(line: &str, into: &mut BTreeMap<String, String>) {
    for tok in line.split_whitespace() {
        if let Some((k, v)) = tok.split_once('=') {
            into.insert(k.to_string(), v.to_string());
        }
    }
}

/// Format records in the on-disk golden layout.
pub fn format_records(records: &[Record]) -> String {
    let mut s = String::new();
    for r in records {
        s.push_str("CASE ");
        s.push_str(&r.name);
        s.push('\n');
        s.push_str("IN ");
        s.push_str(&format_kv(&r.input));
        s.push('\n');
        s.push_str("OUT ");
        s.push_str(&format_kv(&r.output));
        s.push('\n');
    }
    s
}

fn format_kv(map: &BTreeMap<String, String>) -> String {
    map.iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mismatch {
    pub name: String,
    pub iss: String,
    pub silicon: String,
}

impl fmt::Display for Mismatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}:\n  iss     {}\n  silicon {}",
            self.name, self.iss, self.silicon
        )
    }
}

/// Line-by-line report diff. Equal length and content required.
pub fn diff_report(iss: &[String], silicon: &[String]) -> Vec<Mismatch> {
    let n = iss.len().max(silicon.len());
    let mut out = Vec::new();
    for i in 0..n {
        let a = iss.get(i).map(String::as_str).unwrap_or("<missing>");
        let b = silicon.get(i).map(String::as_str).unwrap_or("<missing>");
        if a != b {
            out.push(Mismatch {
                name: format!("line {i}"),
                iss: a.to_string(),
                silicon: b.to_string(),
            });
        }
    }
    out
}

/// Per-case output map diff. Inputs are not compared (ISS is given them).
pub fn diff_records(iss: &[Record], silicon: &[Record]) -> Vec<Mismatch> {
    let mut out = Vec::new();
    let n = iss.len().max(silicon.len());
    for i in 0..n {
        match (iss.get(i), silicon.get(i)) {
            (None, Some(s)) => out.push(Mismatch {
                name: s.name.clone(),
                iss: "<missing>".into(),
                silicon: format!("{:?}", s.output),
            }),
            (Some(a), None) => out.push(Mismatch {
                name: a.name.clone(),
                iss: format!("{:?}", a.output),
                silicon: "<missing>".into(),
            }),
            (Some(a), Some(s)) => {
                if a.name != s.name {
                    out.push(Mismatch {
                        name: a.name.clone(),
                        iss: a.name.clone(),
                        silicon: s.name.clone(),
                    });
                    continue;
                }
                if a.output != s.output {
                    out.push(Mismatch {
                        name: a.name.clone(),
                        iss: format_kv(&a.output),
                        silicon: format_kv(&s.output),
                    });
                }
            }
            (None, None) => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[test]
    fn report_drops_comments_and_keeps_pass() {
        let lines = parse_report("# hi\nPASS foo 1\n\nRESULT 0 PASS\n");
        assert_eq!(lines, ["PASS foo 1", "RESULT 0 PASS"]);
    }

    #[test]
    fn records_roundtrip() {
        let text = "\
CASE add_reg
IN  enc=f103c1e1 din=00000002
OUT d=00000005 c=0
";
        let recs = parse_records(text).unwrap();
        assert_eq!(recs.len(), 1);
        assert_eq!(recs[0].name, "add_reg");
        assert_eq!(recs[0].input["enc"], "f103c1e1");
        assert_eq!(recs[0].output["d"], "00000005");
        let again = parse_records(&format_records(&recs)).unwrap();
        assert_eq!(recs, again);
    }

    #[rstest]
    #[case::match_ok(&["PASS a"], &["PASS a"], 0)]
    #[case::mismatch(&["PASS a"], &["PASS b"], 1)]
    fn report_diff(#[case] iss: &[&str], #[case] si: &[&str], #[case] n: usize) {
        let a: Vec<String> = iss.iter().map(|s| (*s).to_string()).collect();
        let b: Vec<String> = si.iter().map(|s| (*s).to_string()).collect();
        assert_eq!(diff_report(&a, &b).len(), n);
    }
}
