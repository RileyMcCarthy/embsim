//! Catalogs composed into one: [`CatalogSet`].
//!
//! A project is built with one [`Catalog`]. A set is that catalog made of
//! several: the standard catalog, which every set starts with; the QEMU
//! core (`embsim_p2_qemu::catalog::register`); and whatever a project adds
//! — its own boards, part models, P2 cores and bench components. The set
//! answers each kind from the catalog that provides it.
//!
//! ```
//! use embsim_board::{Catalog, ComponentRequest, Component, KindInfo, ProjectError};
//! use embsim_boards::catalog::CatalogSet;
//!
//! /// A catalog of one bench component kind.
//! struct Bench;
//!
//! impl Catalog for Bench {
//!     fn name(&self) -> &str {
//!         "bench-catalog"
//!     }
//!     fn component_kinds(&self) -> Vec<KindInfo> {
//!         vec![KindInfo::new("bench-lamp", "a lamp on the bench")]
//!     }
//!     fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
//!         Err(request.error("this example builds no lamp"))
//!     }
//! }
//!
//! let mut set = CatalogSet::new();
//! set.add(Bench).expect("its kind is spelled as a kind is");
//! assert_eq!(set.catalogs(), ["embsim-boards", "bench-catalog"]);
//! assert!(set.component_kinds().iter().any(|kind| kind.name == "bench-lamp"));
//! ```
//!
//! # Names
//!
//! Board, part, component and core kinds share one namespace, and a kind
//! means the same thing in every project that names it, so no catalog
//! replaces another's kind. Two catalogs in a set may still both provide a
//! name: the set holds both, and a project that **names** that kind is
//! refused, the error naming both catalogs ([`Catalog::kind_clash`]). A
//! project that does not name it builds. So a catalog a project wrote does
//! not stop building when embsim later ships a kind of the same name;
//! only the projects that name the kind are asked to choose, and a project
//! catalog that puts its project's name in front of each of its kinds
//! (`mad-machine`) is never asked. A key two catalogs' base registrations
//! place parts by is the same ([`Catalog::base_key_clash`]): a board that
//! carries such a part is refused unless an entry gives the key a kind.
//!
//! What a set refuses when a catalog is added is what is wrong whatever a
//! project names: a kind not spelled as a kind is (lowercase letters,
//! digits and hyphens), `netlist` (the board kind every project has), a
//! name one catalog provides twice, and a second catalog under a name the
//! set already holds.
//!
//! # What a set adds to a kind's description
//!
//! The set treats every catalog's kinds alike; no kind is special to it.
//! An option a kind declares as one of the set's core kinds
//! ([`embsim_board::OptionValues::CoreKind`], the `p2` package's `core`)
//! is described with every core the set holds, and a kind that needs the
//! set itself reads it through the assignment it is registered with
//! (`Assignment::catalog`, [`Catalog::as_any`]): that is how the `p2`
//! package finds the cores ([`CatalogSet::core_catalogs`]).

use std::collections::BTreeMap;

use embsim_board::{
    Assignment, BoardSpec, Catalog, CatalogBoard, Component, ComponentRequest, KindGuide, KindInfo,
    OptionValues, PartOptions, PartRegistry, ProjectError,
};

use crate::catalog::StandardCatalog;
use crate::p2::{core_kinds_listed, CoreCatalog, HeldInResetCores};

/// One catalog's kinds, by sort, as it described them when it joined.
#[derive(Debug, Default)]
struct Kinds {
    boards: Vec<KindInfo>,
    parts: Vec<String>,
    components: Vec<KindInfo>,
}

/// The names of `kinds`.
fn names(kinds: &[KindInfo]) -> impl Iterator<Item = &str> {
    kinds.iter().map(|kind| kind.name.as_ref())
}

/// Catalogs composed into one (module docs).
pub struct CatalogSet {
    /// The catalogs, in the order they were added; the standard catalog
    /// first.
    catalogs: Vec<Box<dyn Catalog>>,
    /// Each catalog's kinds, read once when it was added.
    kinds: Vec<Kinds>,
    /// The P2 core catalogs, in the order they were added; the standard
    /// catalog's `held-in-reset` first.
    cores: Vec<Box<dyn CoreCatalog>>,
    /// Every kind name, of every sort, and the catalogs that provide it.
    providers: BTreeMap<String, Vec<String>>,
    /// Every key a catalog's base registrations place parts by, and the
    /// catalogs that place by it.
    base_keys: BTreeMap<String, Vec<String>>,
    /// Every catalog's name, once, in the order it first joined.
    order: Vec<String>,
}

impl std::fmt::Debug for CatalogSet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CatalogSet")
            .field("catalogs", &self.catalogs())
            .finish()
    }
}

impl Default for CatalogSet {
    fn default() -> Self {
        Self::new()
    }
}

impl CatalogSet {
    /// The standard catalog, with its one core, `held-in-reset`.
    pub fn new() -> Self {
        let mut set = Self {
            catalogs: Vec::new(),
            kinds: Vec::new(),
            cores: Vec::new(),
            providers: BTreeMap::new(),
            base_keys: BTreeMap::new(),
            order: Vec::new(),
        };
        set.add(StandardCatalog)
            .expect("the standard catalog's kinds are spelled as kinds are");
        set.add_cores(HeldInResetCores)
            .expect("the standard core's name is spelled as a kind is");
        set
    }

    /// Add `catalog`'s board, part and component kinds and its base
    /// registrations. Refused, with the reason, for what is wrong whatever
    /// a project names (module docs, "Names").
    pub fn add(&mut self, catalog: impl Catalog + 'static) -> Result<(), ProjectError> {
        let name = catalog.name().to_string();
        let found = Kinds {
            boards: catalog.board_kinds(),
            parts: catalog
                .part_kinds()
                .iter()
                .map(|kind| kind.name().to_string())
                .collect(),
            components: catalog.component_kinds(),
        };
        let mut kinds: Vec<(&'static str, String)> = Vec::new();
        kinds.extend(names(&found.boards).map(|kind| ("board", kind.to_string())));
        kinds.extend(found.parts.iter().map(|kind| ("part", kind.clone())));
        kinds.extend(names(&found.components).map(|kind| ("component", kind.to_string())));
        self.admit(&name, &kinds)?;
        let mut scratch = PartRegistry::new();
        catalog.register_base(&mut scratch);
        for key in scratch.keys() {
            self.base_keys.entry(key).or_default().push(name.clone());
        }
        self.catalogs.push(Box::new(catalog));
        self.kinds.push(found);
        Ok(())
    }

    /// Add `cores`' core kinds, the values the `p2` kind's `core` option
    /// takes. Refused as [`Self::add`] refuses.
    pub fn add_cores(&mut self, cores: impl CoreCatalog + 'static) -> Result<(), ProjectError> {
        let name = cores.name().to_string();
        let kinds: Vec<(&'static str, String)> = cores
            .core_kinds()
            .into_iter()
            .map(|kind| ("core", kind.name.into_owned()))
            .collect();
        // A core catalog may share its name with the catalog it ships beside
        // (the standard catalog and its `held-in-reset`): one crate, one name.
        self.admit_named(&name, &kinds, true)?;
        self.cores.push(Box::new(cores));
        Ok(())
    }

    fn admit(&mut self, name: &str, kinds: &[(&'static str, String)]) -> Result<(), ProjectError> {
        self.admit_named(name, kinds, false)
    }

    fn admit_named(
        &mut self,
        name: &str,
        kinds: &[(&'static str, String)],
        cores: bool,
    ) -> Result<(), ProjectError> {
        let refuse = |why: String| ProjectError::message(format!("catalog {name}: {why}"));
        if name.trim().is_empty() {
            return Err(ProjectError::message(
                "a catalog's name is what an error naming it prints: give it its crate's name",
            ));
        }
        let taken = if cores {
            self.cores.iter().any(|catalog| catalog.name() == name)
        } else {
            self.catalogs.iter().any(|catalog| catalog.name() == name)
        };
        if taken {
            return Err(refuse(format!(
                "the set already holds a catalog named {name:?}; an error naming two catalogs \
                 has to tell them apart, so each has its own name"
            )));
        }
        let mut own: BTreeMap<&str, &str> = BTreeMap::new();
        for (sort, kind) in kinds {
            if kind == "netlist" {
                return Err(refuse(format!(
                    "{sort} kind \"netlist\" is the board kind every project has, read from a \
                     netlist file; no catalog provides it"
                )));
            }
            let spelled = !kind.is_empty()
                && kind
                    .chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-');
            if !spelled {
                return Err(refuse(format!(
                    "{sort} kind {kind:?}: a kind is lowercase letters, digits and hyphens"
                )));
            }
            if let Some(first) = own.insert(kind.as_str(), sort) {
                return Err(refuse(format!(
                    "{kind:?} is both a {first} kind and a {sort} kind of this catalog; board, \
                     part, component and core kinds share one namespace"
                )));
            }
            let already = self
                .providers
                .get(kind)
                .is_some_and(|providers| providers.iter().any(|provider| provider == name));
            if already {
                return Err(refuse(format!(
                    "{sort} kind {kind:?} is already a kind of {name}; board, part, component \
                     and core kinds share one namespace"
                )));
            }
        }
        for (_, kind) in kinds {
            let providers = self.providers.entry(kind.clone()).or_default();
            if !providers.iter().any(|provider| provider == name) {
                providers.push(name.to_string());
            }
        }
        if !self.order.iter().any(|seen| seen == name) {
            self.order.push(name.to_string());
        }
        Ok(())
    }

    /// The catalogs' names, in the order they joined, the standard catalog
    /// first; a core catalog that shares its name with a catalog (one
    /// crate, one name) is named once.
    pub fn catalogs(&self) -> Vec<String> {
        self.order.clone()
    }

    /// Every P2 core kind the set holds, in the order they were added.
    pub fn core_kinds(&self) -> Vec<KindInfo> {
        self.cores
            .iter()
            .flat_map(|catalog| catalog.core_kinds())
            .collect()
    }

    /// The P2 core catalogs the set holds, in the order they were added
    /// (the standard catalog's `held-in-reset` first): what a package kind
    /// seats its core from (`embsim_boards::p2::register_p2`).
    pub fn core_catalogs(&self) -> Vec<&dyn CoreCatalog> {
        self.cores.iter().map(|catalog| catalog.as_ref()).collect()
    }

    /// Every part kind, as someone choosing one reads it: the guides of
    /// every catalog, in the order the catalogs were added
    /// ([`Catalog::part_kinds`]).
    pub fn guide(&self) -> Vec<KindGuide> {
        self.part_kinds()
    }

    /// The first catalog whose kinds `has` says provide `kind`.
    fn provider(&self, has: impl Fn(&Kinds) -> bool) -> Option<&dyn Catalog> {
        self.kinds
            .iter()
            .position(has)
            .map(|index| self.catalogs[index].as_ref())
    }

    /// `option` as the set describes it: an option taking one of the set's
    /// core kinds names each of them after what it means.
    fn described(&self, mut option: embsim_board::RequiredOption) -> embsim_board::RequiredOption {
        if option.values == OptionValues::CoreKind {
            option.means = format!(
                "{}: {}",
                option.means,
                core_kinds_listed(&self.core_kinds())
            )
            .into();
        }
        option
    }
}

/// The kinds in `lists`, each name once, in the order they first appear.
fn union(lists: impl Iterator<Item = Vec<KindInfo>>) -> Vec<KindInfo> {
    let mut kinds: Vec<KindInfo> = Vec::new();
    for kind in lists.flatten() {
        if !kinds.iter().any(|known| known.name == kind.name) {
            kinds.push(kind);
        }
    }
    kinds
}

impl Catalog for CatalogSet {
    fn name(&self) -> &str {
        "the catalog set"
    }

    fn board_kinds(&self) -> Vec<KindInfo> {
        union(self.kinds.iter().map(|kinds| kinds.boards.clone()))
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        match self.provider(|kinds| names(&kinds.boards).any(|name| name == spec.kind)) {
            Some(catalog) => catalog.board(spec),
            None => Err(ProjectError::message(format!(
                "board {}: no catalog in the set has board kind {:?}",
                spec.name, spec.kind
            ))),
        }
    }

    fn register_base(&self, registry: &mut PartRegistry) {
        for catalog in &self.catalogs {
            catalog.register_base(registry);
        }
    }

    fn part_kinds(&self) -> Vec<KindGuide> {
        self.catalogs
            .iter()
            .flat_map(|catalog| catalog.part_kinds())
            .map(|mut guide| {
                guide.info.required = std::mem::take(&mut guide.info.required)
                    .into_iter()
                    .map(|option| self.described(option))
                    .collect();
                guide
            })
            .collect()
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        match self.provider(|kinds| kinds.parts.iter().any(|name| name == assignment.kind)) {
            Some(catalog) => catalog.register_part(registry, assignment, options),
            None => Err(assignment.error("no catalog in the set has this part kind")),
        }
    }

    fn component_kinds(&self) -> Vec<KindInfo> {
        union(self.kinds.iter().map(|kinds| kinds.components.clone()))
    }

    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        match self.provider(|kinds| names(&kinds.components).any(|name| name == request.spec.kind))
        {
            Some(catalog) => catalog.component(request),
            None => Err(request.error("no catalog in the set has this component kind")),
        }
    }

    fn kind_clash(&self, kind: &str) -> Vec<String> {
        match self.providers.get(kind) {
            Some(providers) if providers.len() >= 2 => providers.clone(),
            _ => Vec::new(),
        }
    }

    fn base_key_clash(&self, key: &str) -> Vec<String> {
        match self.base_keys.get(key) {
            Some(providers) if providers.len() >= 2 => providers.clone(),
            _ => Vec::new(),
        }
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }
}
