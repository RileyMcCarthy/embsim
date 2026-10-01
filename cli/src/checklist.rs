//! What a netlist asks of a project, and the project that answers it as far
//! as the catalog can: `embsim survey` and `embsim new`.
//!
//! Both survey the netlist the way a project surveys a `kind = "netlist"`
//! board — through [`Project::survey`], with the catalog's base registry —
//! so what they list is what `embsim check` will refuse until it is
//! answered.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::io::Write as IoWrite;
use std::path::{Component as PathPart, Path, PathBuf};

use embsim_board::{
    is_connector, kinds_without_a_model, BoardSurvey, Classification, ConnectorReport, Fit,
    KeyField, KindGuide, PassiveKind, PinSite, Project, SurveyedPart, UnmodelledPart,
};
use embsim_boards::catalog::CatalogSet;

/// The widest a line of references gets before it wraps.
const LINE_WIDTH: usize = 100;

/// A string as TOML writes it: quoted, escaped.
pub fn toml_string(text: &str) -> String {
    toml::Value::String(text.to_string()).to_string()
}

/// The board name a netlist file gives by default: its file stem, with
/// every dot and space made an underscore (a name is the first word of an
/// endpoint).
fn default_name(netlist: &Path) -> String {
    let stem = netlist
        .file_stem()
        .map(|stem| stem.to_string_lossy().into_owned())
        .unwrap_or_default();
    let name: String = stem
        .chars()
        .map(|c| {
            if c == '.' || c.is_whitespace() {
                '_'
            } else {
                c
            }
        })
        .collect();
    if name.is_empty() {
        "BOARD".to_string()
    } else {
        name
    }
}

/// Survey `netlist` as the board `name` of a project whose one board is
/// that netlist, with the set's base registrations.
fn survey_netlist(set: &CatalogSet, netlist: &Path, name: &str) -> Result<BoardSurvey, String> {
    let file = netlist
        .file_name()
        .ok_or_else(|| format!("{} names no netlist file", netlist.display()))?;
    let dir = netlist
        .parent()
        .filter(|dir| !dir.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let text = format!(
        "[[board]]\nname = {}\nkind = \"netlist\"\nnetlist = {}\n",
        toml_string(name),
        toml_string(&file.to_string_lossy())
    );
    let project = Project::parse(&text)
        .map_err(|err| err.to_string())?
        .relative_to(dir);
    project.survey(set, name).map_err(|err| err.to_string())
}

// ============================================================
// Reading a survey
// ============================================================

/// What a class makes a part, as a column heading.
fn class_label(class: &Classification) -> &'static str {
    match class {
        Classification::Passive { kind, .. } => match kind {
            PassiveKind::Resistor => "resistor",
            PassiveKind::Capacitor => "capacitor",
            PassiveKind::Inductor => "inductor",
            PassiveKind::Diode => "diode",
            PassiveKind::Led => "LED",
        },
        Classification::Boundary => "connector",
        Classification::Jumper { .. } => "jumper",
        Classification::Switch { .. } => "switch",
        Classification::Pwl { .. } => "element",
        Classification::Probe => "test point",
        Classification::Mechanical => "mechanical",
        Classification::Registered => "model",
    }
}

/// Which of a part's fields `key` is: the one the registry reached it by.
fn key_field(part: &SurveyedPart, key: &str) -> KeyField {
    if part.part == key {
        KeyField::Part
    } else if part.mpn.as_deref() == Some(key) {
        KeyField::Mpn
    } else {
        KeyField::Value
    }
}

/// The field and key a new `[[board.model]]` for an unmodelled part is
/// keyed by: its manufacturer part number, else its symbol's part name,
/// else its value — the most specific key it carries.
fn stub_key(part: &UnmodelledPart) -> (KeyField, &str) {
    match &part.mpn {
        Some(mpn) => (KeyField::Mpn, mpn),
        None if !part.part.is_empty() => (KeyField::Part, &part.part),
        None => (KeyField::Value, &part.value),
    }
}

/// How strong a fit is: a part number names the part itself, a family
/// name its family.
fn strength(fit: Fit) -> u8 {
    match fit {
        Fit::Number(_) => 2,
        Fit::Family(_) => 1,
    }
}

/// The kinds a part with these keys is, one fit per kind, and only the
/// strongest kind of fit any kind makes: the kinds whose part numbers name
/// it, else the kinds whose part family its keys name. Pins name no kind.
fn candidates<'g>(guide: &'g [KindGuide], keys: &[&str]) -> Vec<(&'g KindGuide, Fit)> {
    let fits: Vec<(&KindGuide, Fit)> = guide
        .iter()
        .filter_map(|kind| kind.fit(keys).map(|fit| (kind, fit)))
        .collect();
    let best = fits.iter().map(|(_, fit)| strength(*fit)).max();
    fits.into_iter()
        .filter(|(_, fit)| Some(strength(*fit)) == best)
        .collect()
}

/// `kind (why)` for one candidate.
fn fit_phrase(kind: &KindGuide, fit: Fit) -> String {
    match fit {
        Fit::Number(number) => format!("{} (part number {number})", kind.name),
        Fit::Family(family) => format!("{} (part family {family})", kind.name),
    }
}

/// The distinct nets a part's pins sit on.
fn net_count(pins: &[PinSite]) -> usize {
    pins.iter()
        .map(|site| site.net.as_str())
        .filter(|net| !net.is_empty())
        .collect::<BTreeSet<_>>()
        .len()
}

/// What a part that needs a model could be, as a sentence: the kinds whose
/// number or family it carries, else the kinds without a model its
/// designator, symbol, name or nets allow, else that it needs a model.
fn candidates_sentence(candidates: &[(&KindGuide, Fit)], part: &UnmodelledPart) -> String {
    if !candidates.is_empty() {
        return format!(
            "could be: {}",
            candidates
                .iter()
                .map(|(kind, fit)| fit_phrase(kind, *fit))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let plain = kinds_without_a_model(
        &part.reference,
        &part.part,
        &part.value,
        net_count(&part.pins),
    );
    if plain.is_empty() {
        return "no catalog kind is for this part: it needs a model (PROJECTS.md §7)".to_string();
    }
    format!(
        "no catalog model is for this part; it may be {}",
        plain
            .iter()
            .map(|(kind, why)| format!("{kind} ({why})"))
            .collect::<Vec<_>>()
            .join(", ")
    )
}

/// Letters and digits, upper-cased, with a symbol name's lower-case `x` —
/// a placeholder for any one character (`AM26LV32xD`) — kept as `None`.
fn stem_with_placeholders(text: &str) -> Vec<Option<char>> {
    text.chars()
        .filter(char::is_ascii_alphanumeric)
        .map(|c| (c != 'x').then(|| c.to_ascii_uppercase()))
        .collect()
}

/// The fewest leading letters and digits two part numbers share for one to
/// be read as a variant of the other's family: a vendor prefix and a series.
const SHARED_FAMILY_STEM: usize = 4;

/// When a part's symbol names one part and its manufacturer part number
/// another of the same family (`AM26LV32xD` and `AM26LS32CD`), the sentence
/// that says so: a model belongs to one part, and the netlist does not say
/// which the board carries.
fn names_disagree(part: &UnmodelledPart) -> Option<String> {
    let mpn = part.mpn.as_deref()?;
    let (a, b) = (
        stem_with_placeholders(&part.part),
        stem_with_placeholders(mpn),
    );
    if a.len() < SHARED_FAMILY_STEM || b.len() < SHARED_FAMILY_STEM {
        return None;
    }
    let same = |x: &Option<char>, y: &Option<char>| x.is_none() || y.is_none() || x == y;
    let shared = a.iter().zip(&b).take_while(|(x, y)| same(x, y)).count();
    if shared < SHARED_FAMILY_STEM || shared == a.len().min(b.len()) {
        return None;
    }
    Some(format!(
        "its symbol names {:?} and its mpn {mpn:?}: two parts; give it the model of the one \
         the board carries",
        part.part
    ))
}

/// A part's pins, comma-separated.
fn pin_list(pins: &[&str]) -> String {
    pins.join(", ")
}

/// `words` after `lead`, one space apart with `separator`'s mark (a comma)
/// after all but the last, wrapped at [`LINE_WIDTH`]; a line after the
/// first starts with `continuation`.
fn wrapped(lead: &str, words: &[&str], continuation: &str, separator: &str) -> String {
    let mark = separator.trim_end();
    let mut out = lead.to_string();
    let mut width = lead.chars().count();
    let mut line_empty = true;
    for (index, word) in words.iter().enumerate() {
        let piece = if index + 1 < words.len() {
            format!("{word}{mark}")
        } else {
            (*word).to_string()
        };
        let length = piece.chars().count();
        if !line_empty && width + 1 + length > LINE_WIDTH {
            out.push('\n');
            out.push_str(continuation);
            width = continuation.chars().count();
            line_empty = true;
        }
        if !line_empty {
            out.push(' ');
            width += 1;
        }
        out.push_str(&piece);
        width += length;
        line_empty = false;
    }
    out
}

/// A part's keys as a clause, `gap` between them: `part "X"  value "Y"
/// mpn "Z"`.
fn keys_clause(part: &str, value: &str, mpn: Option<&str>, gap: &str) -> String {
    let mut keys = Vec::new();
    if !part.is_empty() {
        keys.push(format!("part {part:?}"));
    }
    keys.push(format!("value {value:?}"));
    if let Some(mpn) = mpn {
        keys.push(format!("mpn {mpn:?}"));
    }
    keys.join(gap)
}

/// The mismatched parts grouped by what placed them: the key, the model
/// and the netlist's pins, each with the references it covers.
struct PinTableGroup<'s> {
    field: KeyField,
    key: &'s str,
    model: &'s str,
    declared: &'s [String],
    netlist: Vec<&'s str>,
    references: Vec<&'s str>,
}

fn pin_table_groups(survey: &BoardSurvey) -> Vec<PinTableGroup<'_>> {
    let mut groups: Vec<PinTableGroup<'_>> = Vec::new();
    for mismatch in &survey.mismatched {
        let part = survey
            .parts()
            .find(|part| part.reference == mismatch.reference)
            .expect("a mismatched part is a surveyed part");
        let key = part.key.as_deref().unwrap_or(&mismatch.value);
        let netlist: Vec<&str> = mismatch.netlist.iter().map(String::as_str).collect();
        if let Some(group) = groups.iter_mut().find(|group| {
            group.key == key && group.model == mismatch.model && group.netlist == netlist
        }) {
            group.references.push(&mismatch.reference);
            continue;
        }
        groups.push(PinTableGroup {
            field: key_field(part, key),
            key,
            model: &mismatch.model,
            declared: &mismatch.declared,
            netlist,
            references: vec![&mismatch.reference],
        });
    }
    groups
}

/// The catalog kind a key places parts as, when the key is one of the
/// numbers the catalog places a kind by.
fn kind_placed_by<'g>(guide: &'g [KindGuide], key: &str) -> Option<&'g KindGuide> {
    guide.iter().find(|kind| kind.numbers.contains(&key))
}

/// The option table of `kind` that declares exactly `pins`.
fn option_table_for<'g>(kind: &'g KindGuide, pins: &[&str]) -> Option<&'g str> {
    kind.table_with(pins)
        .filter(|table| table.option)
        .map(|table| table.name)
}

// ============================================================
// embsim survey
// ============================================================

/// `embsim survey <netlist>`.
pub fn survey(set: &CatalogSet, netlist: &Path, out: &mut dyn IoWrite) -> Result<(), String> {
    let name = default_name(netlist);
    let survey = survey_netlist(set, netlist, &name)?;
    let _ = write!(
        out,
        "{}",
        survey_report(&netlist.display().to_string(), &survey, &set.guide())
    );
    Ok(())
}

/// `embsim survey --kind <board kind>`: a board a catalog ships, surveyed
/// with the registry it builds with, as a project's `[[board]]` of that
/// kind surveys it — every part it places, the ones it leaves to the
/// project, and every connector pin with its name and net.
pub fn survey_kind(set: &CatalogSet, kind: &str, out: &mut dyn IoWrite) -> Result<(), String> {
    if kind == "netlist" {
        return Err(
            "--kind netlist is a board read from a file; give the file: embsim survey board.net"
                .to_string(),
        );
    }
    let text = format!(
        "[[board]]\nname = \"BOARD\"\nkind = {}\n",
        toml_string(kind)
    );
    let project = Project::parse(&text).map_err(|err| err.to_string())?;
    let survey = project
        .survey(set, "BOARD")
        .map_err(|err| err.to_string())?;
    let _ = write!(
        out,
        "{}",
        survey_report(&format!("kind {kind:?}"), &survey, &set.guide())
    );
    Ok(())
}

/// The checklist, as `embsim survey` prints it.
fn survey_report(source: &str, survey: &BoardSurvey, guide: &[KindGuide]) -> String {
    let mut out = String::new();
    let populated = survey.modelled - survey.mismatched.len();
    let _ = writeln!(out, "{source}: {} parts", survey.part_count);
    let _ = writeln!(out, "  {populated} populated by the catalog");
    let _ = writeln!(out, "  {} need a model", survey.needs_model.len());
    let _ = writeln!(
        out,
        "  {} placed with a pin table the netlist does not use",
        survey.mismatched.len()
    );
    let _ = writeln!(out, "  {} refused", survey.refused.len());
    let _ = writeln!(
        out,
        "  {}",
        count(survey.connectors.len(), "connector", "connectors")
    );

    let mismatched: Vec<&str> = survey
        .mismatched
        .iter()
        .map(|part| part.reference.as_str())
        .collect();
    let placed: Vec<&SurveyedPart> = survey
        .parts()
        .filter(|part| part.class.is_some() && !mismatched.contains(&part.reference.as_str()))
        .collect();

    // By class: the symbol or the reference designator says what it is.
    let mut by_class: Vec<(&str, Vec<&str>)> = Vec::new();
    for part in placed.iter().filter(|part| part.key.is_none()) {
        let label = class_label(part.class.as_ref().expect("placed parts have a class"));
        match by_class.iter_mut().find(|(seen, _)| *seen == label) {
            Some((_, references)) => references.push(&part.reference),
            None => by_class.push((label, vec![&part.reference])),
        }
    }
    if !by_class.is_empty() {
        let _ = writeln!(
            out,
            "\npopulated by class (the symbol or reference designator says what it is):"
        );
        for (label, references) in &by_class {
            let lead = format!("  {label:<11} {:>4}  ", references.len());
            let continuation = " ".repeat(lead.len());
            let _ = writeln!(out, "{}", wrapped(&lead, references, &continuation, " "));
        }
    }

    // By key: an entry of the catalog's registry reached it.
    let mut by_key: Vec<(String, String, Vec<&str>)> = Vec::new();
    for part in placed.iter().filter(|part| part.key.is_some()) {
        let key = part.key.as_deref().expect("filtered on a key");
        let field = key_field(part, key);
        let what = part
            .model
            .clone()
            .unwrap_or_else(|| class_label(part.class.as_ref().expect("placed")).to_string());
        let by = format!("{field} {key:?}");
        match by_key.iter_mut().find(|(b, w, _)| *b == by && *w == what) {
            Some((_, _, references)) => references.push(&part.reference),
            None => by_key.push((by, what, vec![&part.reference])),
        }
    }
    if !by_key.is_empty() {
        let _ = writeln!(out, "\npopulated by a catalog model, by the key shown:");
        for (by, what, references) in &by_key {
            let _ = writeln!(out, "  {by}: {what}");
            let _ = writeln!(out, "{}", wrapped("      ", references, "      ", ", "));
        }
    }

    if !survey.needs_model.is_empty() {
        let _ = writeln!(out, "\nneed a model:");
    }
    for part in &survey.needs_model {
        let pins: Vec<&str> = part.pins.iter().map(|site| site.pin.as_str()).collect();
        let _ = writeln!(
            out,
            "  {}  {}",
            part.reference,
            keys_clause(&part.part, &part.value, part.mpn.as_deref(), "  ")
        );
        if pins.is_empty() {
            let _ = writeln!(out, "      no pins");
        } else if is_connector(&part.reference, &part.part) {
            // A connector once it is one: its pins are where a wire or a
            // mate will land, so they are listed as a connector's are.
            let _ = writeln!(out, "      {}:", count(pins.len(), "pin", "pins"));
            let as_connector = ConnectorReport {
                reference: part.reference.clone(),
                value: part.value.clone(),
                pins: part.pins.clone(),
            };
            let _ = write!(
                out,
                "{}",
                pin_table(&as_connector, "        ", "pin", |pin| pin.to_string())
            );
        } else {
            let lead = format!("      {}: ", count(pins.len(), "pin", "pins"));
            let _ = writeln!(out, "{}", wrapped(&lead, &pins, "        ", ", "));
        }
        if let Some(disagree) = names_disagree(part) {
            let _ = writeln!(out, "      {disagree}");
        }
        let keys = [
            part.part.as_str(),
            part.mpn.as_deref().unwrap_or(""),
            part.value.as_str(),
        ];
        let found = candidates(guide, &keys);
        let _ = writeln!(out, "      {}", candidates_sentence(&found, part));
    }

    let groups = pin_table_groups(survey);
    if !groups.is_empty() {
        let _ = writeln!(out, "\nplaced with a pin table the netlist does not use:");
    }
    for group in &groups {
        let _ = writeln!(
            out,
            "  {}  {} {:?}: {}",
            group.references.join(", "),
            group.field,
            group.key,
            group.model
        );
        let declared: Vec<&str> = group.declared.iter().map(String::as_str).collect();
        let _ = writeln!(out, "      declares {}", pin_list(&declared));
        let _ = writeln!(out, "      the netlist has {}", pin_list(&group.netlist));
        let fix = kind_placed_by(guide, group.key)
            .and_then(|kind| option_table_for(kind, &group.netlist).map(|table| (kind, table)));
        match fix {
            Some((_, table)) => {
                let _ = writeln!(out, "      pins = {table:?} declares the netlist's pins");
            }
            None => {
                let _ = writeln!(
                    out,
                    "      no pin table of this model declares the netlist's pins"
                );
            }
        }
    }

    if !survey.refused.is_empty() {
        let _ = writeln!(out, "\nrefused:");
    }
    for refusal in &survey.refused {
        let _ = writeln!(out, "  {refusal}");
    }

    if !survey.connectors.is_empty() {
        let _ = writeln!(
            out,
            "\nconnectors (a wire's board end is Board.Connector.Pin):"
        );
    }
    for connector in &survey.connectors {
        let _ = writeln!(
            out,
            "  {}  value {:?}  {}",
            connector.reference,
            connector.value,
            count(connector.pins.len(), "pin", "pins")
        );
        let _ = write!(
            out,
            "{}",
            pin_table(connector, "    ", "pin", |pin| pin.to_string())
        );
    }
    out
}

/// A connector's pins as aligned rows: pin (as `endpoint` spells it), its
/// name, its net.
fn pin_table(
    connector: &ConnectorReport,
    indent: &str,
    heading: &str,
    endpoint: impl Fn(&str) -> String,
) -> String {
    let rows: Vec<(String, &str, &str)> = connector
        .pins
        .iter()
        .map(|site| {
            (
                endpoint(&site.pin),
                site.pinfunction.as_deref().unwrap_or("-"),
                site.net.as_str(),
            )
        })
        .collect();
    let first = rows
        .iter()
        .map(|row| row.0.len())
        .max()
        .unwrap_or(0)
        .max(heading.len());
    let second = rows.iter().map(|row| row.1.len()).max().unwrap_or(0).max(4);
    let mut out = String::new();
    let _ = writeln!(out, "{indent}{heading:<first$}  {:<second$}  net", "name");
    for (pin, name, net) in rows {
        let _ = writeln!(out, "{indent}{pin:<first$}  {name:<second$}  {net}");
    }
    out
}

// ============================================================
// embsim new
// ============================================================

/// `embsim new <netlist> [--name N] [-o project.toml] [--force]`.
pub fn new_project(
    set: &CatalogSet,
    netlist: &Path,
    name: Option<&str>,
    output: Option<&Path>,
    force: bool,
    out: &mut dyn IoWrite,
) -> Result<(), String> {
    let name = name.map_or_else(|| default_name(netlist), str::to_string);
    let survey = survey_netlist(set, netlist, &name)?;

    // The netlist, relative to where the project will live.
    let project_dir = match output {
        Some(path) => path
            .parent()
            .filter(|dir| !dir.as_os_str().is_empty())
            .map_or_else(|| PathBuf::from("."), Path::to_path_buf),
        None => PathBuf::from("."),
    };
    let relative = relative_path(netlist, &project_dir)?;
    let source = netlist.file_name().map_or_else(
        || netlist.display().to_string(),
        |file| file.to_string_lossy().into_owned(),
    );
    let text = starter_project(
        &source,
        &name,
        &relative.to_string_lossy(),
        &survey,
        &set.guide(),
    );
    // What `new` writes is a project; a text that is not one is this
    // command's own error.
    Project::parse(&text)
        .map_err(|err| format!("the starter project does not parse (embsim's error): {err}"))?;

    let Some(path) = output else {
        let _ = write!(out, "{text}");
        return Ok(());
    };
    if path.exists() && !force {
        return Err(format!(
            "{} exists; pass --force to replace it",
            path.display()
        ));
    }
    std::fs::write(path, &text).map_err(|err| format!("cannot write {}: {err}", path.display()))?;
    let stubs = stub_groups(&survey).len();
    let _ = writeln!(out, "wrote {}", path.display());
    let _ = writeln!(
        out,
        "  board {name}: {} parts, {} populated by the catalog, {} pin table{} chosen",
        survey.part_count,
        survey.modelled - survey.mismatched.len(),
        survey.mismatched.len(),
        if survey.mismatched.len() == 1 {
            ""
        } else {
            "s"
        }
    );
    if stubs > 0 {
        let _ = writeln!(
            out,
            "  {stubs} model stub{} to fill in, for {} part{} that need a model",
            if stubs == 1 { "" } else { "s" },
            survey.needs_model.len(),
            if survey.needs_model.len() == 1 {
                ""
            } else {
                "s"
            }
        );
    }
    let _ = writeln!(
        out,
        "  {} connector{} to wire; then `embsim check {}`",
        survey.connectors.len(),
        if survey.connectors.len() == 1 {
            ""
        } else {
            "s"
        },
        path.display()
    );
    Ok(())
}

/// `target` relative to the directory `base`, both resolved on disk.
fn relative_path(target: &Path, base: &Path) -> Result<PathBuf, String> {
    let target = target
        .canonicalize()
        .map_err(|err| format!("{}: {err}", target.display()))?;
    let base = base
        .canonicalize()
        .map_err(|err| format!("{}: {err}", base.display()))?;
    let target_parts: Vec<PathPart<'_>> = target.components().collect();
    let base_parts: Vec<PathPart<'_>> = base.components().collect();
    let common = target_parts
        .iter()
        .zip(&base_parts)
        .take_while(|(a, b)| a == b)
        .count();
    // Two paths that share only the root are clearer written whole.
    if common <= 1 {
        return Ok(target);
    }
    let mut relative = PathBuf::new();
    for _ in common..base_parts.len() {
        relative.push("..");
    }
    for part in &target_parts[common..] {
        relative.push(part.as_os_str());
    }
    Ok(relative)
}

/// The parts that need a model grouped by the key their stub uses.
fn stub_groups(survey: &BoardSurvey) -> Vec<(KeyField, &str, Vec<&UnmodelledPart>)> {
    let mut groups: Vec<(KeyField, &str, Vec<&UnmodelledPart>)> = Vec::new();
    for part in &survey.needs_model {
        let (field, key) = stub_key(part);
        match groups.iter_mut().find(|(f, k, _)| *f == field && *k == key) {
            Some((_, _, parts)) => parts.push(part),
            None => groups.push((field, key, vec![part])),
        }
    }
    groups
}

/// A comment block: every line of `text` wrapped and prefixed `# `, an
/// empty line a bare `#`.
fn comment(out: &mut String, text: &str) {
    for paragraph in text.split('\n') {
        if paragraph.is_empty() {
            let _ = writeln!(out, "#");
            continue;
        }
        let words: Vec<&str> = paragraph.split(' ').collect();
        let _ = writeln!(out, "{}", wrapped("# ", &words, "# ", " "));
    }
}

/// The starter project `embsim new` writes.
fn starter_project(
    source: &str,
    name: &str,
    netlist: &str,
    survey: &BoardSurvey,
    guide: &[KindGuide],
) -> String {
    let mut out = String::new();
    let populated = survey.modelled - survey.mismatched.len();
    comment(
        &mut out,
        &format!(
            "An embsim project for {source}, started by `embsim new`.\n\nThe netlist, surveyed \
             with every model the catalog places by part number: {} parts; {populated} \
             populated; {} placed with a pin table this file chooses; {} need a model; {} \
             refused; {} connectors.\n\nTo finish it: uncomment each stub under \"Parts that \
             need a model\" and give it a kind, then wire the connectors. `embsim check` on \
             this file says what is still missing; `embsim run` runs it.",
            survey.part_count,
            survey.mismatched.len(),
            survey.needs_model.len(),
            survey.refused.len(),
            survey.connectors.len()
        ),
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "[[board]]");
    let _ = writeln!(out, "name = {}", toml_string(name));
    let _ = writeln!(out, "kind = \"netlist\"");
    if Path::new(netlist).is_absolute() {
        let _ = writeln!(out, "netlist = {}", toml_string(netlist));
    } else {
        let _ = writeln!(
            out,
            "netlist = {}    # relative to this file",
            toml_string(netlist)
        );
    }

    let groups = pin_table_groups(survey);
    if !groups.is_empty() {
        let _ = writeln!(out);
        section(&mut out, "Pin tables");
        comment(
            &mut out,
            "The catalog places these parts by their part number with its model's default pin \
             table, the datasheet's numbered one; the netlist names their pins otherwise. Each \
             entry below picks the table that has the netlist's pins.",
        );
    }
    for group in &groups {
        let _ = writeln!(out);
        let fix = kind_placed_by(guide, group.key)
            .and_then(|kind| option_table_for(kind, &group.netlist).map(|table| (kind, table)));
        let declared: Vec<&str> = group.declared.iter().map(String::as_str).collect();
        match fix {
            Some((kind, table)) => {
                comment(
                    &mut out,
                    &format!(
                        "{}: {} declares {}; the netlist has {}.",
                        group.references.join(", "),
                        group.model,
                        pin_list(&declared),
                        pin_list(&group.netlist)
                    ),
                );
                let _ = writeln!(out, "[[board.model]]");
                let _ = writeln!(out, "{} = {}", group.field, toml_string(group.key));
                let _ = writeln!(out, "kind = {}", toml_string(kind.name));
                let _ = writeln!(out, "[board.model.options]");
                let _ = writeln!(out, "pins = {}", toml_string(table));
            }
            None => comment(
                &mut out,
                &format!(
                    "{}: {} declares {}; the netlist has {}, and no pin table of the model has \
                     them. Give the part another model: a [[board.model]] {} = {:?} with a kind \
                     whose pins are the netlist's.",
                    group.references.join(", "),
                    group.model,
                    pin_list(&declared),
                    pin_list(&group.netlist),
                    group.field,
                    group.key
                ),
            ),
        }
    }

    let stubs = stub_groups(survey);
    if !stubs.is_empty() {
        let _ = writeln!(out);
        section(&mut out, "Parts that need a model");
        let kinds: Vec<String> = guide
            .iter()
            .map(|kind| format!("{:?}", kind.name))
            .collect();
        comment(
            &mut out,
            &format!(
                "Uncomment each stub and give it a kind: {}. `embsim survey` lists what each \
                 kind is.",
                kinds.join(", ")
            ),
        );
    }
    for (field, key, parts) in &stubs {
        let _ = writeln!(out);
        let first = parts[0];
        let pins: Vec<&str> = first.pins.iter().map(|site| site.pin.as_str()).collect();
        let references: Vec<&str> = parts.iter().map(|part| part.reference.as_str()).collect();
        let keys = [
            first.part.as_str(),
            first.mpn.as_deref().unwrap_or(""),
            first.value.as_str(),
        ];
        let found = candidates(guide, &keys);
        let pins_clause = if pins.is_empty() {
            "no pins".to_string()
        } else {
            format!("{}: {}", count(pins.len(), "pin", "pins"), pin_list(&pins))
        };
        let disagree = names_disagree(first)
            .map(|text| format!("\n{}.", capitalized(&text)))
            .unwrap_or_default();
        comment(
            &mut out,
            &format!(
                "{}: {}; {pins_clause}.{disagree}\n{}.",
                references.join(", "),
                keys_clause(&first.part, &first.value, first.mpn.as_deref(), ", "),
                capitalized(&candidates_sentence(&found, first))
            ),
        );
        // The kind is written in only where a part number names it: a
        // family says the model is for the part's series, and the part's
        // own number is for the author to check against it.
        let numbered: Vec<&KindGuide> = found
            .iter()
            .filter(|(_, fit)| matches!(fit, Fit::Number(_)))
            .map(|(kind, _)| *kind)
            .collect();
        let chosen = match numbered.as_slice() {
            [only] if disagree.is_empty() => Some(*only),
            _ => None,
        };
        let _ = writeln!(out, "# [[board.model]]");
        let _ = writeln!(out, "# {field} = {}", toml_string(key));
        let Some(kind) = chosen else {
            let _ = writeln!(out, "# kind = \"\"");
            continue;
        };
        let _ = writeln!(out, "# kind = {}", toml_string(kind.name));
        // Each required option under a comment saying what it is, which
        // stays a comment when the stub is uncommented.
        let mut options = String::new();
        for option in &kind.required {
            let mut said = String::new();
            comment(&mut said, &capitalized(&format!("{}.", option.means)));
            for line in said.lines() {
                let _ = writeln!(options, "# {line}");
            }
            let _ = writeln!(options, "# {} = {}", option.name, option.example);
        }
        if let Some(table) = option_table_for(kind, &pins) {
            if kind.tables.first().map(|first| first.name) != Some(table) {
                let _ = writeln!(options, "# pins = {}", toml_string(table));
            }
        }
        if !options.is_empty() {
            let _ = writeln!(out, "# [board.model.options]");
            out.push_str(&options);
        }
    }

    if !survey.connectors.is_empty() {
        let _ = writeln!(out);
        section(&mut out, "Connectors: where a wire may land");
        comment(
            &mut out,
            "A [[wire]] joins a connector pin, Board.Connector.Pin, to a pin on another board's \
             connector, a bench component's pin, or a supply of its own: a name no board or \
             component has, which the `from` of a wire with `volts` makes. For example:",
        );
        let example = &survey.connectors[0];
        let _ = writeln!(out, "#");
        let _ = writeln!(out, "# [[wire]]");
        let _ = writeln!(out, "# from = \"BENCH.SUPPLY\"");
        let _ = writeln!(
            out,
            "# to = {}",
            toml_string(&format!(
                "{name}.{}.{}",
                example.reference,
                example.pins.first().map_or("1", |site| site.pin.as_str())
            ))
        );
        let _ = writeln!(out, "# volts = 0.0    # the supply's voltage");
    }
    for connector in &survey.connectors {
        let _ = writeln!(out, "#");
        let _ = writeln!(
            out,
            "# {} {:?}, {}:",
            connector.reference,
            connector.value,
            count(connector.pins.len(), "pin", "pins")
        );
        let prefix = format!("{name}.{}.", connector.reference);
        let _ = write!(
            out,
            "{}",
            pin_table(connector, "#   ", "endpoint", |pin| format!(
                "{prefix}{pin}"
            ))
        );
    }
    out
}

/// A section heading comment, ruled out to 78 columns.
fn section(out: &mut String, title: &str) {
    let rule = "-".repeat(78usize.saturating_sub(title.len() + 8).max(4));
    let _ = writeln!(out, "# ---- {title} {rule}");
}

/// `1 pin`, `8 pins`.
fn count(n: usize, one: &str, many: &str) -> String {
    format!("{n} {}", if n == 1 { one } else { many })
}

fn capitalized(text: &str) -> String {
    let mut chars = text.chars();
    match chars.next() {
        Some(first) => first.to_uppercase().chain(chars).collect(),
        None => String::new(),
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::stem("boards/netlists/p2_ec32mb.net", "p2_ec32mb")]
    #[case::dots("a/board.rev.b.net", "board_rev_b")]
    #[case::spaces("my board.net", "my_board")]
    fn a_netlist_names_its_board(#[case] path: &str, #[case] name: &str) {
        assert_eq!(default_name(Path::new(path)), name);
    }

    #[rstest]
    fn references_wrap_at_the_line_width_under_their_indent() {
        let words: Vec<String> = (1..=40).map(|n| format!("C{n}")).collect();
        let words: Vec<&str> = words.iter().map(String::as_str).collect();
        let text = wrapped("    ", &words, "    ", " ");
        assert!(text.lines().count() > 1, "{text}");
        for line in text.lines() {
            assert!(line.len() <= LINE_WIDTH, "{line:?}");
            assert!(line.starts_with("    C"), "{line:?}");
        }
    }
}
