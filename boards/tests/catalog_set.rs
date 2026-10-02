//! Catalogs composed into one: what a set refuses when a catalog joins it,
//! what a project that names a kind two catalogs provide is told, the check
//! that a kind seats only on a part it is — made by the project for every
//! catalog's kinds — and a board kind's own entries. Build only: every case
//! is decided before an engine exists.

use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use embsim_board::{
    netlist, Assignment, BoardSpec, Catalog, CatalogBoard, Classification, KindGuide, KindInfo,
    ModelSpec, Named, PartOptions, PartRegistry, Project, ProjectError, RequiredOption,
};
use embsim_boards::catalog::CatalogSet;
use embsim_boards::ec32mb::dip_switch_poles;
use embsim_boards::p2::{CoreCatalog, CoreCtor, HeldInReset, P2Core};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The directory the shipped example projects live in.
fn projects() -> PathBuf {
    [env!("CARGO_MANIFEST_DIR"), "projects"].iter().collect()
}

/// The P2-EC32MB's transcribed netlist as a `netlist` board, plus `rest`.
fn ec32(rest: &str) -> Project {
    Project::parse(&format!(
        "[[board]]\nname = \"EC32\"\nkind = \"netlist\"\nnetlist = \"../netlists/p2_ec32mb.net\"\n\
         {rest}"
    ))
    .expect("the text is a project")
    .relative_to(projects())
}

/// A catalog of whatever kinds a case needs, counting the entries it is
/// asked to register.
struct Kinds {
    name: &'static str,
    boards: Vec<KindInfo>,
    parts: Vec<KindGuide>,
    components: Vec<KindInfo>,
    /// Keys its base registrations place as mechanical parts.
    base: Vec<&'static str>,
    registered: Arc<AtomicUsize>,
}

impl Kinds {
    fn named(name: &'static str) -> Self {
        Self {
            name,
            boards: Vec::new(),
            parts: Vec::new(),
            components: Vec::new(),
            base: Vec::new(),
            registered: Arc::new(AtomicUsize::new(0)),
        }
    }

    fn part(mut self, kind: KindGuide) -> Self {
        self.parts.push(kind);
        self
    }
}

impl Catalog for Kinds {
    fn name(&self) -> &str {
        self.name
    }

    fn board_kinds(&self) -> Vec<KindInfo> {
        self.boards.clone()
    }

    fn register_base(&self, registry: &mut PartRegistry) {
        for key in &self.base {
            registry.register_mechanical(*key);
        }
    }

    fn part_kinds(&self) -> Vec<KindGuide> {
        self.parts.clone()
    }

    /// Every kind here is the P2-EC32MB's four-pole option switch.
    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        self.registered.fetch_add(1, Ordering::SeqCst);
        options.finish()?;
        registry.register_switch(assignment.key, dip_switch_poles());
        Ok(())
    }

    fn component_kinds(&self) -> Vec<KindInfo> {
        self.components.clone()
    }
}

/// A part kind for the option switch, by the name `name`.
fn switch_kind(name: &'static str) -> KindGuide {
    KindGuide::new(name, "the module's four-pole option switch", Named::Switch)
}

/// A core catalog of one core kind that holds the chip in reset.
struct Cores {
    name: &'static str,
    core: &'static str,
}

impl CoreCatalog for Cores {
    fn name(&self) -> &str {
        self.name
    }

    fn core_kinds(&self) -> Vec<KindInfo> {
        vec![KindInfo::new(self.core, "the chip held in reset")]
    }

    fn seat(
        &self,
        _core: &str,
        _assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<CoreCtor, ProjectError> {
        options.finish()?;
        Ok(Box::new(|_| Ok(Box::new(HeldInReset) as Box<dyn P2Core>)))
    }
}

fn assert_says(message: &str, needles: &[&str]) {
    for needle in needles {
        assert!(
            message.contains(needle),
            "{needle:?} missing from:\n{message}"
        );
    }
}

#[rstest]
fn a_part_kind_two_catalogs_provide_is_refused_only_where_a_project_names_it() {
    behaviour!(Test {
        id: "catalogs.part-kind-clash",
        covers: Some("boards/src/set.rs#CatalogSet::kind_clash"),
        given: "a set holding two catalogs that each provide a part kind of the same name, and \
                the P2-EC32MB's netlist in a project",
    });
    expect!(
        "named-refused",
        "an entry that names the kind is refused, the error naming both catalogs",
        "a kind means one thing in every project, so neither catalog's model is taken for it"
    );
    expect!(
        "unnamed-builds",
        "a project that does not name the kind is surveyed as if the clash were not there",
        "a catalog a project wrote keeps working when another later ships a kind of the same \
         name, until a project names that kind"
    );
    let mut set = CatalogSet::new();
    set.add(Kinds::named("catalog-a").part(switch_kind("twin-switch")))
        .expect("the first catalog joins");
    set.add(Kinds::named("catalog-b").part(switch_kind("twin-switch")))
        .expect("the second joins too: the clash is decided where a project names the kind");

    let named = ec32("[[board.model]]\nmpn = \"218-4LPSTJR\"\nkind = \"twin-switch\"\n");
    let message = named
        .survey(&set, "EC32")
        .expect_err("the kind is two catalogs'")
        .to_string();
    assert_says(
        &message,
        &[
            "kind \"twin-switch\" is provided by 2 catalogs, catalog-a and catalog-b",
            "[[board.model]] mpn = \"218-4LPSTJR\"",
        ],
    );

    let unnamed = ec32(
        "[[board.model]]\nmpn = \"218-4LPSTJR\"\nkind = \"switch\"\n[board.model.options]\n\
         poles = [[\"1_ON\", \"1_OFF\"], [\"2_ON\", \"2_OFF\"], [\"3_ON\", \"3_OFF\"], \
         [\"4_ON\", \"4_OFF\"]]\n",
    );
    let survey = unnamed
        .survey(&set, "EC32")
        .expect("the standard switch kind is one catalog's");
    assert!(matches!(
        survey.class_of("S301"),
        Some(Classification::Switch { .. })
    ));
}

#[rstest]
fn a_core_two_catalogs_provide_is_refused_naming_both() {
    behaviour!(Test {
        id: "catalogs.core-kind-clash",
        covers: Some("boards/src/p2.rs#register_p2"),
        given: "a set holding two core catalogs that each provide a processor core of the same \
                name, and a project seating that core in the P2-EC32MB's processor",
    });
    expect!(
        "refused-naming-both",
        "the processor's entry is refused, the error naming the core and both catalogs"
    );
    let mut set = CatalogSet::new();
    set.add_cores(Cores {
        name: "cores-a",
        core: "twin-core",
    })
    .expect("the first core catalog joins");
    set.add_cores(Cores {
        name: "cores-b",
        core: "twin-core",
    })
    .expect("the second joins too");
    let project = Project::parse(
        "[[board]]\nname = \"EC32\"\nkind = \"p2-ec32mb\"\n[[board.model]]\nvalue = \
         \"P2X8C4M64P\"\nkind = \"p2\"\n[board.model.options]\ncore = \"twin-core\"\n",
    )
    .expect("the text is a project");
    let message = project
        .survey(&set, "EC32")
        .expect_err("the core is two catalogs'")
        .to_string();
    assert_says(
        &message,
        &["core \"twin-core\" is provided by 2 catalogs, cores-a and cores-b"],
    );
}

#[rstest]
fn a_part_number_two_catalogs_place_is_refused_on_a_board_that_carries_it() {
    behaviour!(Test {
        id: "catalogs.base-key-clash",
        covers: Some("board/src/project.rs#prepare"),
        given: "a set holding two catalogs whose base registrations both place the P2-EC32MB's \
                bill-of-materials line by its part number, and the module's netlist in a \
                project",
    });
    expect!(
        "carried-refused",
        "the board is refused, naming the part, the part number and both catalogs"
    );
    expect!(
        "entry-decides",
        "an entry that gives that part number a kind itself settles it, and the board is \
         surveyed",
        "the project then says which model the part is, so neither catalog's is guessed"
    );
    let mut set = CatalogSet::new();
    for name in ["catalog-a", "catalog-b"] {
        let mut kinds = Kinds::named(name);
        kinds.base.push("300-64002");
        set.add(kinds)
            .expect("a catalog with base registrations joins");
    }
    let message = ec32("")
        .survey(&set, "EC32")
        .expect_err("two catalogs place the part")
        .to_string();
    assert_says(
        &message,
        &[
            "is placed by \"300-64002\", and 2 catalogs place parts by that key, catalog-a and \
             catalog-b",
        ],
    );
    ec32("[[board.model]]\nmpn = \"300-64002\"\nkind = \"mechanical\"\n")
        .survey(&set, "EC32")
        .expect("the entry says what the part is");
}

#[rstest]
#[case::netlist(Kinds { boards: vec![KindInfo::new("netlist", "a board")], ..Kinds::named("bad-catalog") }, "\"netlist\" is the board kind every project has")]
#[case::spelling(Kinds::named("bad-catalog").part(switch_kind("Big_Switch")), "a kind is lowercase letters, digits and hyphens")]
#[case::two_sorts(Kinds { components: vec![KindInfo::new("thing", "a thing")], ..Kinds::named("bad-catalog").part(switch_kind("thing")) }, "both a part kind and a component kind of this catalog")]
#[case::same_name(
    Kinds::named("embsim-boards"),
    "the set already holds a catalog named \"embsim-boards\""
)]
fn what_is_wrong_whatever_a_project_names_is_refused_when_a_catalog_joins(
    #[case] catalog: Kinds,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "catalogs.refused-on-joining",
        covers: Some("boards/src/set.rs#CatalogSet::add"),
        given: "a catalog joining a set while providing the board kind every project has, a \
                kind with capitals or underscores, one name as two sorts, or a catalog name \
                the set holds",
    });
    expect!(
        "refused-saying-why",
        "the catalog is refused as it joins, before any project is read, with the reason"
    );
    let mut set = CatalogSet::new();
    let message = set
        .add(catalog)
        .expect_err("the catalog is refused")
        .to_string();
    assert_says(&message, &["catalog ", says]);
}

#[rstest]
fn an_added_kind_seats_only_on_a_part_it_is_whichever_catalog_it_is_from() {
    behaviour!(Test {
        id: "catalogs.kind-checks-the-part",
        covers: Some("board/src/kind.rs#KindGuide::check"),
        given: "a catalog a project added whose part kind is for a serial flash, assigned to \
                the P2-EC32MB's option switch",
    });
    expect!(
        "refused",
        "the entry is refused, naming the switch and the part family the kind is for",
        "a kind says what a part is (DESIGN.md rule 1), for every catalog's kinds"
    );
    expect!(
        "catalog-not-asked",
        "the catalog is never asked to register the kind",
        "the project checks the part before any catalog sees the entry"
    );
    let catalog = Kinds::named("project-catalog").part(KindGuide::new(
        "my-flash",
        "a serial flash",
        Named::family(["W25Q128JV"]),
    ));
    let registered = Arc::clone(&catalog.registered);
    let mut set = CatalogSet::new();
    set.add(catalog).expect("the catalog joins");
    let message = ec32("[[board.model]]\nmpn = \"218-4LPSTJR\"\nkind = \"my-flash\"\n")
        .survey(&set, "EC32")
        .expect_err("the switch is no flash")
        .to_string();
    assert_says(
        &message,
        &[
            "S301 is not the part this kind says it is",
            "kind \"my-flash\" is for a part whose part name, mpn or value contains W25Q128JV",
        ],
    );
    assert_eq!(registered.load(Ordering::SeqCst), 0);
}

/// A strip of two connectors and a resistor, bundled: `J1` a header drawn
/// with a symbol of the strip's own library, `J2` a pair of pads on the
/// signal net.
const STRIP: &str = r#"(export (version "E")
  (components
    (comp (ref "J1") (value "Strip")
      (libsource (lib "Strip") (part "Strip_Header")))
    (comp (ref "J2") (value "Pads")
      (libsource (lib "Strip") (part "Strip_Pads")))
    (comp (ref "R1") (value "10k")
      (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "SIG")
      (node (ref "J1") (pin "1"))
      (node (ref "J2") (pin "1"))
      (node (ref "J2") (pin "2"))
      (node (ref "R1") (pin "1")))
    (net (code "2") (name "GND")
      (node (ref "J1") (pin "2"))
      (node (ref "R1") (pin "2")))))"#;

/// A catalog whose one board kind is the strip, bringing entries for its
/// own two connectors.
struct StripCatalog;

impl Catalog for StripCatalog {
    fn name(&self) -> &str {
        "strip-catalog"
    }

    fn board_kinds(&self) -> Vec<KindInfo> {
        vec![KindInfo::new("strip", "a strip of two connectors")]
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        let netlist = netlist::parse(STRIP)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
        Ok(CatalogBoard::from_base(netlist)
            .with_model(ModelSpec::by_part("Strip_Header", "boundary"))
            .with_model(ModelSpec::by_part("Strip_Pads", "mechanical")))
    }
}

#[rstest]
fn a_board_kinds_own_entries_seat_standard_kinds_and_a_project_entry_replaces_one() {
    behaviour!(Test {
        id: "catalogs.board-kind-entries",
        covers: Some("board/src/project.rs#prepare"),
        given: "an added catalog's board kind whose bundled header and pads are drawn with \
                symbols of its own library, and which brings entries making them a connector and \
                a mechanical part",
    });
    expect!(
        "seated",
        "the survey places every part on the board, with nothing left to assign",
        "a board kind's entries go through the standard kinds with every check a project \
         file's entry gets"
    );
    expect!(
        "project-replaces",
        "a project entry for the pads by the same key makes them a connector",
        "a project entry with the key of the board's own replaces it, as a project's flash \
         image replaces a module's blank flash"
    );
    let mut set = CatalogSet::new();
    set.add(StripCatalog).expect("the catalog joins");
    let strip = |rest: &str| {
        Project::parse(&format!(
            "[[board]]\nname = \"STRIP\"\nkind = \"strip\"\n{rest}"
        ))
        .expect("the text is a project")
    };
    let survey = strip("").survey(&set, "STRIP").expect("the strip surveys");
    assert!(survey.compliant(), "{survey}");
    assert_eq!(survey.class_of("J1"), Some(&Classification::Boundary));
    assert_eq!(survey.class_of("J2"), Some(&Classification::Mechanical));

    let replaced = strip("[[board.model]]\npart = \"Strip_Pads\"\nkind = \"boundary\"\n")
        .survey(&set, "STRIP")
        .expect("the project's entry replaces the board's");
    assert!(replaced.compliant(), "{replaced}");
    assert_eq!(replaced.class_of("J2"), Some(&Classification::Boundary));
}

#[rstest]
fn an_option_taking_a_core_kind_is_described_with_every_core_the_set_holds() {
    behaviour!(Test {
        id: "catalogs.core-kind-option",
        covers: Some("boards/src/set.rs#CatalogSet::part_kinds"),
        given: "a set holding a project's catalog whose part kind declares a required option \
                taking one of the set's core kinds, and a core catalog adding one core",
    });
    expect!(
        "lists-every-core",
        "the set describes that option with what the kind says it means, then every core kind \
         the set holds, the standard catalog's first, each with what it is",
        "the set treats every catalog's kinds alike: any kind may take a core, and none is \
         named in the set"
    );
    expect!(
        "p2-alike",
        "the standard catalog's P2 package describes its core option the same way"
    );
    let package = KindGuide::new(
        "my-package",
        "a processor package of the project's",
        Named::family(["MY-PACKAGE"]),
    )
    .requires_option(
        RequiredOption::new("core", "\"held-in-reset\"", "what runs inside it")
            .one_of_the_core_kinds(),
    );
    let mut set = CatalogSet::new();
    set.add(Kinds::named("project-catalog").part(package))
        .expect("the catalog joins");
    set.add_cores(Cores {
        name: "project-cores",
        core: "my-core",
    })
    .expect("the core catalog joins");
    let guides = set.part_kinds();
    let means = |kind: &str| {
        guides
            .iter()
            .find(|guide| guide.name() == kind)
            .and_then(|guide| guide.info.required.iter().find(|o| o.name == "core"))
            .map(|option| option.means.to_string())
            .unwrap_or_else(|| panic!("{kind} has a core option"))
    };
    let listed = "\"held-in-reset\", the chip before it runs, or \"my-core\", the chip held in \
                  reset";
    assert_eq!(
        means("my-package"),
        format!("what runs inside it: {listed}")
    );
    assert_eq!(
        means("p2"),
        format!("what runs inside the package: {listed}")
    );
}
