//! The guides' quoted code: a fenced block a guide marks
//! `<!-- quoted from PATH -->` on the line above it is that file's text,
//! line for line. The worked example's crate is compiled by every gate, so
//! a quotation of it is code that builds; this test keeps the quotation
//! and the file the same. Code a guide does not quote runs as a doc test
//! (`PROJECTS.md`, through `embsim-boards`) or is marked as a design. Build
//! only: nothing here starts an engine.

use std::path::{Path, PathBuf};

use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The guides that may quote a file, relative to the workspace root.
const GUIDES: &[&str] = &[
    "README.md",
    "PROJECTS.md",
    "TESTING.md",
    "MIGRATING-MAD.md",
    "examples/custom-project/README.md",
];

/// The marker a quotation carries on the line above its fence.
const MARKER: (&str, &str) = ("<!-- quoted from ", " -->");

/// The workspace root.
fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate sits in the workspace")
        .to_path_buf()
}

/// One quotation: the guide, the line its marker is on, the file it
/// quotes, and the quoted text, each line ending in a newline.
struct Quote {
    guide: &'static str,
    line: usize,
    file: String,
    text: String,
}

/// Every quotation in `guide`. A marker not followed by a fenced block is
/// a quotation of nothing, and fails here with the line it is on.
fn quotes(guide: &'static str) -> Vec<Quote> {
    let path = root().join(guide);
    let body = std::fs::read_to_string(&path)
        .unwrap_or_else(|err| panic!("{} cannot be read: {err}", path.display()));
    let lines: Vec<&str> = body.lines().collect();
    let mut found = Vec::new();
    let mut at = 0;
    while at < lines.len() {
        let Some(file) = lines[at]
            .trim()
            .strip_prefix(MARKER.0)
            .and_then(|rest| rest.strip_suffix(MARKER.1))
        else {
            at += 1;
            continue;
        };
        let marker = at + 1;
        let fence = lines.get(at + 1).map(|line| line.trim_start());
        assert!(
            fence.is_some_and(|line| line.starts_with("```")),
            "{guide}:{marker}: a quotation marker is followed by a fenced block"
        );
        let indent = lines[at + 1].len() - lines[at + 1].trim_start().len();
        let mut text = String::new();
        at += 2;
        while at < lines.len() && lines[at].trim_start() != "```" {
            let line = lines[at];
            text.push_str(line.get(indent.min(line.len())..).unwrap_or(""));
            text.push('\n');
            at += 1;
        }
        assert!(
            at < lines.len(),
            "{guide}:{marker}: the quoted block is closed"
        );
        found.push(Quote {
            guide,
            line: marker,
            file: file.to_string(),
            text,
        });
        at += 1;
    }
    found
}

#[rstest]
fn every_quotation_in_the_guides_is_the_text_of_the_file_it_names() {
    behaviour!(Test {
        id: "guides.quotations",
        covers: Some("PROJECTS.md#10-extending-embsim-from-a-project"),
        given: "the guides' code blocks marked as quoted from a file of the repository, beside \
                the files they name",
    });
    expect!(
        "verbatim",
        "every quoted block appears in the file it names, line for line",
        "the quoted files are the worked example's crate and project, which every gate \
         compiles and runs, so a quotation that matches is code that builds"
    );
    expect!(
        "example-quoted",
        "the projects guide quotes the worked example's registration function and the \
         example's own binary over the command's library"
    );

    let all: Vec<Quote> = GUIDES.iter().flat_map(|guide| quotes(guide)).collect();
    for quote in &all {
        let path = root().join(&quote.file);
        let file = std::fs::read_to_string(&path).unwrap_or_else(|err| {
            panic!(
                "{}:{} quotes {}, which cannot be read: {err}",
                quote.guide,
                quote.line,
                path.display()
            )
        });
        assert!(
            file.contains(&quote.text),
            "{}:{}: the block quoted from {} is not that file's text; the file has \
             changed, so quote it again:\n{}",
            quote.guide,
            quote.line,
            quote.file,
            quote.text
        );
    }

    let quoted = |file: &str, needle: &str| {
        all.iter().any(|quote| {
            quote.guide == "PROJECTS.md" && quote.file == file && quote.text.contains(needle)
        })
    };
    assert!(
        quoted(
            "examples/custom-project/catalog/src/lib.rs",
            "pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {"
        ),
        "PROJECTS.md quotes the example's registration function"
    );
    assert!(
        quoted(
            "examples/custom-project/catalog/examples/own_binary.rs",
            "embsim_cli::main_with(set)"
        ),
        "PROJECTS.md quotes the example's own binary"
    );
}
