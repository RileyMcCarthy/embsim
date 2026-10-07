#!/usr/bin/env python3
"""The assets of an embsim release, made and checked.

CI's release jobs run this (`.github/workflows/ci.yml`: `release-binaries`
on each platform, then `release`, which publishes), and a maintainer runs
the same commands to try a release before tagging it (TESTING.md,
"Releases"):

    release.py check [--tag vX.Y.Z]
        The tag is v<the workspace version>, and CHANGELOG.md has a section
        for that version. Without --tag, a tag build's own tag
        (GITHUB_REF_TYPE/GITHUB_REF_NAME); a run with no tag checks the
        changelog alone.

    release.py binary --target TRIPLE [--out DIR] [--no-run]
        Package the `embsim` that `cargo build --release --locked -p
        embsim-cli --target TRIPLE` built: run it first (its version and
        revision are this checkout's HEAD with no changes, its QEMU plan is
        this checkout's target, it runs a shipped project), then write
        embsim-<version>-<triple>.tar.gz and its .sha256: the binary, the
        README, the changelog, every licence and notice for what the binary
        carries, and the licence texts of the third-party crates compiled
        into it.

    release.py qemu-target [--out DIR]
        The sources `embsim qemu install` builds qemu-system-p2 from, as
        embsim-<version>-qemu-target.tar.gz and its .sha256: the target
        directory as git tracks it, the licence texts it names, and a
        README with its identity, the QEMU it is staged into and the steps.

    release.py checksums DIR
        Check every <asset>.sha256 in DIR against its asset, and write
        SHA256SUMS over every asset.

    release.py notes --dir DIR --out FILE
        The release's notes: the changelog's section, then what each asset
        in DIR is. Refuses a release missing an asset.

Archives are deterministic: entries sorted, owned by 0:0, modes 0755 or
0644, every time the commit's (or SOURCE_DATE_EPOCH), gzip with no name or
time. Standard library only; Python 3.9 or newer.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import os
import re
import subprocess
import sys
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# The platforms a release ships `embsim` for, and what each is for. The
# `release-binaries` matrix in ci.yml builds exactly these; `notes` refuses
# a release missing one. `glibc` is the newest glibc symbol version the
# binary may need: what the Linux runner (ubuntu-22.04) has, checked.
TARGETS = {
    "aarch64-apple-darwin": {"what": "macOS on Apple silicon (arm64)"},
    "x86_64-apple-darwin": {"what": "macOS on Intel (x86_64)"},
    "x86_64-unknown-linux-gnu": {
        "what": "Linux on x86_64, glibc 2.35 or newer (Ubuntu 22.04, Debian 12)",
        "glibc": (2, 35),
    },
}

# A shipped project the smoke run checks and runs: the P2-EC32MB from its
# netlist on its carrier's fingers, so the netlist, the catalog, the
# engine and the P2 package all run.
SMOKE_PROJECT = "boards/projects/ec32-carrier.toml"
SMOKE_RUN = "3ms"
SMOKE_RAN = "ran 3.000000 ms of virtual time"

# Files of the repository every binary archive carries, at the same paths:
# the README's links resolve inside the archive.
BINARY_DOCS = [
    "README.md",
    "CHANGELOG.md",
    "LICENSE",
    "LICENSES/GPL-2.0-or-later.txt",
    "LICENSES/LGPL-2.1-or-later.txt",
    "boards/netlists/LICENSE",
    "p2-qemu/rom/LICENSE-PARALLAX",
    "p2-qemu/qemu-target/LICENSE-PNut-TS",
]

QEMU_TARGET = "p2-qemu/qemu-target"

# A package's own licence files, at its top level.
LICENCE_FILE = re.compile(
    r"^(LICEN[CS]E|COPYING|COPYRIGHT|NOTICE|UNLICENSE)([-._].*)?$", re.IGNORECASE
)


def fail(message: str) -> None:
    raise SystemExit(f"release.py: {message}")


def run(
    command: list[str], cwd: Path = ROOT, check: bool = True
) -> subprocess.CompletedProcess:
    """`command` in `cwd`, its output captured as text."""
    try:
        done = subprocess.run(command, cwd=cwd, capture_output=True, text=True)
    except OSError as error:
        fail(f"cannot run {command[0]}: {error}")
    if check and done.returncode != 0:
        fail(
            f"`{' '.join(command)}` failed ({done.returncode}):\n"
            f"{done.stdout}{done.stderr}"
        )
    return done


def cargo_metadata(*extra: str) -> dict:
    return json.loads(
        run(["cargo", "metadata", "--format-version", "1", *extra]).stdout
    )


def workspace_version() -> str:
    """embsim's version: the `embsim-cli` package's, which is the workspace's."""
    for package in cargo_metadata("--no-deps")["packages"]:
        if package["name"] == "embsim-cli":
            return package["version"]
    fail("the workspace has no embsim-cli package")
    return ""


def the_tag(given: str | None) -> str | None:
    """The tag being released: --tag, else a tag build's own."""
    if given:
        return given
    if os.environ.get("GITHUB_REF_TYPE") == "tag":
        return os.environ.get("GITHUB_REF_NAME") or None
    return None


def check_tag(version: str, tag: str | None) -> None:
    if tag is not None and tag != f"v{version}":
        fail(
            f"the tag is {tag} and embsim's version is {version} (Cargo.toml, "
            f"[workspace.package]); a release's tag is v<version>. Set the version, "
            f"commit, and tag that commit v<version>"
        )


def changelog_section(version: str) -> str:
    """CHANGELOG.md's section for `version`, without its heading."""
    path = ROOT / "CHANGELOG.md"
    if not path.is_file():
        fail("there is no CHANGELOG.md; write one with a `## [<version>]` section")
    lines = path.read_text(encoding="utf-8").splitlines()
    heading = re.compile(r"^## \[" + re.escape(version) + r"\]")
    start = next((i for i, line in enumerate(lines) if heading.match(line)), None)
    if start is None:
        fail(
            f"CHANGELOG.md has no section for {version}: add a `## [{version}]` "
            f"heading with what the release changes"
        )
    body = []
    for line in lines[start + 1 :]:
        if line.startswith("## [") or re.match(r"^\[[^\]]+\]: ", line):
            break
        body.append(line)
    text = "\n".join(body).strip()
    if not text:
        fail(
            f"CHANGELOG.md's section for {version} is empty: say what the "
            f"release changes"
        )
    return text + "\n"


def head() -> str:
    return run(["git", "rev-parse", "HEAD"]).stdout.strip()


def source_date_epoch() -> int:
    """Every archive entry's time: SOURCE_DATE_EPOCH, else HEAD's commit time."""
    given = os.environ.get("SOURCE_DATE_EPOCH")
    if given:
        return int(given)
    return int(run(["git", "log", "-1", "--format=%ct", "HEAD"]).stdout.strip())


def tracked(directory: str) -> list[tuple[str, bool]]:
    """Every file git tracks under `directory`: its path, and whether git
    records it executable."""
    text = run(["git", "ls-files", "-s", "-z", "--", directory]).stdout
    files = []
    for entry in text.split("\0"):
        if not entry:
            continue
        meta, path = entry.split("\t", 1)
        files.append((path, meta.split()[0] == "100755"))
    return sorted(files)


def qemu_target_identity() -> str:
    """The P2 target's identity over the files git tracks, as `stage.sh` and
    `embsim_p2_qemu::target::identity` compute it, checked against what
    `stage.sh --identity` says of the directory on disk."""
    listing = []
    for path, _ in tracked(QEMU_TARGET):
        relative = path[len(QEMU_TARGET) + 1 :]
        name = relative.rsplit("/", 1)[-1]
        if name == "README.md" or name.startswith("LICENSE-") or name.startswith("."):
            continue
        digest = hashlib.sha256((ROOT / path).read_bytes()).hexdigest()
        listing.append((relative, f"{digest}  {relative}\n"))
    listing.sort(key=lambda item: item[0].encode())
    ours = hashlib.sha256("".join(line for _, line in listing).encode()).hexdigest()[
        :16
    ]
    staged = run(["sh", f"{QEMU_TARGET}/stage.sh", "--identity"]).stdout.strip()
    if staged != ours:
        fail(
            f"stage.sh says the P2 target's identity is {staged} and the files git "
            f"tracks give {ours}: {QEMU_TARGET} holds files git does not track, "
            f"which stage.sh counts and the archive leaves out. Commit them or "
            f"remove them"
        )
    return ours


def qemu_pin() -> tuple[str, str]:
    """QEMU_PIN's tag and commit."""
    fields = {}
    for line in (
        (ROOT / QEMU_TARGET / "QEMU_PIN").read_text(encoding="utf-8").splitlines()
    ):
        parts = line.split()
        if len(parts) == 2 and not line.startswith("#"):
            fields[parts[0]] = parts[1]
    if "tag" not in fields or "commit" not in fields:
        fail(f"{QEMU_TARGET}/QEMU_PIN has no `tag` or no `commit` line")
    return fields["tag"], fields["commit"]


def qemu_configure() -> list[str]:
    """The configure line `embsim qemu install` uses (`install::CONFIGURE`)."""
    source = (ROOT / "p2-qemu/src/install.rs").read_text(encoding="utf-8")
    found = re.search(r"pub const CONFIGURE: \[&str; \d+\] = \[(.*?)\];", source, re.S)
    if not found:
        fail("p2-qemu/src/install.rs has no `pub const CONFIGURE: [&str; N] = [...]`")
    return re.findall(r'"([^"]*)"', found.group(1))


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def write_sha256(asset: Path) -> None:
    (asset.parent / f"{asset.name}.sha256").write_text(
        f"{sha256_of(asset)}  {asset.name}\n"
    )


def write_archive(
    path: Path, entries: list[tuple[str, bytes, bool]], mtime: int
) -> None:
    """A deterministic .tar.gz of `entries` (archive path, bytes, executable),
    with an entry for every directory they sit in."""
    directories = set()
    for name, _, _ in entries:
        parts = name.split("/")[:-1]
        for i in range(1, len(parts) + 1):
            directories.add("/".join(parts[:i]))
    members = [(d, None, True) for d in directories] + list(entries)
    members.sort(key=lambda member: member[0].encode())
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as raw:
        with gzip.GzipFile(
            filename="", mode="wb", fileobj=raw, mtime=0, compresslevel=9
        ) as gz:
            with tarfile.open(fileobj=gz, mode="w", format=tarfile.USTAR_FORMAT) as tar:
                for name, data, executable in members:
                    info = tarfile.TarInfo(name)
                    info.mtime = mtime
                    info.uid = info.gid = 0
                    info.uname = info.gname = ""
                    if data is None:
                        info.type = tarfile.DIRTYPE
                        info.mode = 0o755
                        tar.addfile(info)
                    else:
                        info.size = len(data)
                        info.mode = 0o755 if executable else 0o644
                        tar.addfile(info, io.BytesIO(data))


def third_party(target: str, version: str) -> str:
    """The crates compiled into `embsim` for `target` besides embsim's own,
    each with its declared licence, then every licence text they ship, once,
    with the crates that ship it."""
    metadata = cargo_metadata("--locked", "--filter-platform", target)
    packages = {package["id"]: package for package in metadata["packages"]}
    workspace = set(metadata["workspace_members"])
    nodes = {node["id"]: node for node in metadata["resolve"]["nodes"]}
    roots = [i for i in workspace if packages[i]["name"] == "embsim-cli"]
    if len(roots) != 1:
        fail("cargo metadata names no single embsim-cli")
    # The normal dependencies, transitively: what the binary links. Build
    # and dev dependencies run at build time only.
    seen = set()
    stack = list(roots)
    while stack:
        current = stack.pop()
        if current in seen:
            continue
        seen.add(current)
        for dep in nodes[current]["deps"]:
            if any(kind["kind"] is None for kind in dep["dep_kinds"]):
                stack.append(dep["pkg"])
    crates = sorted(
        (packages[i] for i in seen if i not in workspace),
        key=lambda package: (package["name"], package["version"]),
    )
    texts: dict[str, list[str]] = {}
    names: dict[str, list[str]] = {}
    rows = []
    for package in crates:
        label = f"{package['name']} {package['version']}"
        directory = Path(package["manifest_path"]).parent
        files = sorted(
            f for f in directory.iterdir() if f.is_file() and LICENCE_FILE.match(f.name)
        )
        if package.get("license_file"):
            extra = directory / package["license_file"]
            if extra.is_file() and extra not in files:
                files.append(extra)
        licence = package.get("license") or f"see {package.get('license_file')}"
        repository = package.get("repository") or ""
        rows.append((label, licence, repository, bool(files)))
        for file in files:
            text = (
                file.read_text(encoding="utf-8", errors="replace")
                .replace("\r\n", "\n")
                .strip()
            )
            texts.setdefault(text, []).append(label)
            names.setdefault(text, []).append(file.name)
    width = max(len(label) for label, _, _, _ in rows)
    out = [
        f"Third-party software in embsim {version} for {target}",
        "=" * len(f"Third-party software in embsim {version} for {target}"),
        "",
        "The `embsim` binary is embsim's own code (LICENSE, and NOTICE for what",
        "it carries) and the crates below, compiled in. Each is listed with the",
        "licence its manifest declares; then every licence text the crates ship,",
        "once, with the crates that ship it.",
        "",
        "Crates",
        "------",
        "",
    ]
    for label, licence, repository, has_text in rows:
        note = "" if has_text else "  (ships no licence file)"
        out.append(f"{label:<{width}}  {licence}  {repository}{note}".rstrip())
    out += ["", "Licence texts", "-------------", ""]
    ordered = sorted(
        texts, key=lambda text: (sorted(set(names[text]))[0], texts[text][0])
    )
    for number, text in enumerate(ordered, 1):
        files = ", ".join(sorted(set(names[text])))
        out.append(f"[{number}] {files}, shipped by: {', '.join(texts[text])}")
        out.append("")
        out.append(text)
        out.append("")
    return "\n".join(out) + "\n"


def binary_notice(version: str, target: str, identity: str) -> str:
    return f"""embsim {version} for {target}
{"=" * len(f"embsim {version} for {target}")}

`embsim` is embsim's command (https://github.com/RileyMcCarthy/embsim),
MIT licensed (LICENSE). Besides embsim's own code and the crates listed in
THIRD-PARTY-LICENSES.txt, the binary carries, as data:

- The P2-EC32MB netlist, a transcription of Parallax's P2-EC32MB Rev B
  schematic. It is CC BY-SA 4.0, not MIT: boards/netlists/LICENSE.
- Parallax's P2 boot ROM (rom_booter_v33k), MIT, Copyright (c) 2019
  Parallax Inc.: p2-qemu/rom/LICENSE-PARALLAX.
- The QEMU Propeller 2 target that `embsim qemu install` writes out and
  builds qemu-system-p2 from (target identity {identity}); the binary
  never runs or links it. Its files are LGPL-2.1-or-later (the target and
  board), GPL-2.0-or-later and MIT (its two QEMU patches, each by the QEMU
  files it changes) and MIT (target-p2/insn.decode, from PNut-TS:
  p2-qemu/qemu-target/LICENSE-PNut-TS); the texts are in LICENSES/. The
  same files are this release's embsim-{version}-qemu-target.tar.gz.

qemu-system-p2 is QEMU, a GPL-2.0 program. embsim does not include it:
`embsim qemu install` builds it on your machine from QEMU and the target
above and installs it with its licence and source. embsim runs it as a
separate program and links none of it. This says how the pieces are put
together; it is not legal advice.
"""


def smoke(binary: Path, target: str, version: str, identity: str) -> None:
    """Run what was built: the release, revision and target it says; its QEMU
    plan; a shipped project, checked and run."""
    said = run([str(binary), "--version"]).stdout.splitlines() + ["", ""]
    expected = f"embsim {version}, git rev {head()[:12]}, from "
    if not said[0].startswith(expected):
        fail(
            f"{binary} --version says `{said[0]}`, not `{expected}...`: it was not "
            f"built from this checkout's HEAD with no changes to embsim's crates. "
            f"Commit or stash the changes, then build it again"
        )
    if f", for {target}, profile release" not in said[1]:
        fail(
            f"{binary} --version says `{said[1]}`: it is not a release build "
            f"for {target}"
        )
    plan = run([str(binary), "qemu", "install", "--dry-run"]).stdout
    tag, commit = qemu_pin()
    wanted = {
        "target": identity,
        "qemu": f"{tag} {commit}",
        "configure": " ".join(qemu_configure()),
    }
    lines = {
        line.split(" ", 1)[0]: line.split(" ", 1)[1]
        for line in plan.splitlines()
        if " " in line
    }
    for key, value in wanted.items():
        if not lines.get(key, "").startswith(value):
            fail(
                f"{binary} would install qemu-system-p2 with {key} "
                f"{lines.get(key)!r}, and this checkout's {key} is {value!r}: the "
                f"binary does not carry this checkout's P2 target. Build it again "
                f"from this checkout"
            )
    run([str(binary), "check", SMOKE_PROJECT])
    ran = run([str(binary), "run", SMOKE_PROJECT, "--for", SMOKE_RUN]).stdout
    if SMOKE_RAN not in ran:
        fail(
            f"{binary} run {SMOKE_PROJECT} --for {SMOKE_RUN} did not say "
            f"`{SMOKE_RAN}`:\n{ran}"
        )
    floor = TARGETS[target].get("glibc")
    if floor:
        symbols = run(["objdump", "-T", str(binary)]).stdout
        needed = sorted(
            {
                tuple(int(n) for n in v)
                for v in re.findall(r"GLIBC_(\d+)\.(\d+)", symbols)
            }
        )
        if needed and needed[-1] > floor:
            fail(
                f"{binary} needs glibc {needed[-1][0]}.{needed[-1][1]}, and the "
                f"release says {floor[0]}.{floor[1]} or newer: build it on the older "
                f"system ci.yml names"
            )
        if needed:
            print(f"  needs glibc {needed[-1][0]}.{needed[-1][1]} at most")
    print(f"  ran: --version, qemu install --dry-run, check and run {SMOKE_PROJECT}")


def binary(args: argparse.Namespace) -> None:
    target = args.target
    if target not in TARGETS:
        fail(f"{target} is not a release target; the targets are {', '.join(TARGETS)}")
    version = workspace_version()
    identity = qemu_target_identity()
    target_dir = Path(cargo_metadata("--no-deps")["target_directory"])
    built = target_dir / target / "release" / "embsim"
    if not built.is_file():
        fail(
            f"{built} is not there: build it first with "
            f"`cargo build --release --locked -p embsim-cli --target {target}`"
        )
    print(f"embsim {version} for {target}: {built}")
    if not args.no_run:
        smoke(built, target, version, identity)
    top = f"embsim-{version}-{target}"
    entries = [(f"{top}/embsim", built.read_bytes(), True)]
    for doc in BINARY_DOCS:
        entries.append((f"{top}/{doc}", (ROOT / doc).read_bytes(), False))
    entries.append(
        (f"{top}/NOTICE", binary_notice(version, target, identity).encode(), False)
    )
    entries.append(
        (
            f"{top}/THIRD-PARTY-LICENSES.txt",
            third_party(target, version).encode(),
            False,
        )
    )
    archive = Path(args.out) / f"{top}.tar.gz"
    write_archive(archive, entries, source_date_epoch())
    write_sha256(archive)
    print(f"wrote {archive} ({archive.stat().st_size} bytes) and its .sha256")


def qemu_target_readme(version: str, identity: str) -> str:
    tag, commit = qemu_pin()
    configure = " ".join(qemu_configure())
    return f"""# The QEMU Propeller 2 target of embsim {version}

The sources `qemu-system-p2` is built from: the files embsim {version}
carries and `embsim qemu install` writes out, stages into QEMU and builds,
byte for byte. Target identity `{identity}`: the identity the built
program reports in its handshake, and the directory it is installed into,
`~/.embsim/qemu/{identity}/`.

- `qemu-target/`: the target (`target-p2/`), the board (`hw-p2/`), the
  two QEMU patches, `stage.sh`, and `QEMU_PIN`, the QEMU release it is
  staged into. `qemu-target/README.md` is the target's own guide.
- `LICENSES/`: the GPL-2.0 and LGPL-2.1 texts its files name.

## Building it

`embsim qemu install` does all of this. By hand, with git, a C compiler,
ninja, pkg-config, glib's development files and python3:

```bash
git clone --depth 1 --branch {tag} https://gitlab.com/qemu-project/qemu.git qemu
git -C qemu rev-parse HEAD   # must print {commit}
sh qemu-target/stage.sh qemu
mkdir qemu/build-p2 && cd qemu/build-p2
../configure {configure}
ninja qemu-system-p2
```

Point embsim at the result with `EMBSIM_QEMU_SYSTEM_P2=<path>`, or put it
on `PATH`; `embsim qemu path` says whether it is the one embsim needs.

## Licence

The target, the board and their build files are LGPL-2.1-or-later;
`register-p2.patch` is GPL-2.0-or-later; `host-thread.patch` is MIT for
its hunks to MIT-licensed QEMU files and GPL-2.0-or-later for the rest;
`target-p2/insn.decode` is MIT (`qemu-target/LICENSE-PNut-TS`). Each file
says which. Built into QEMU, which is licensed as a whole under the GPL,
version 2, the program is a GPL-2.0 work; its corresponding source is QEMU
at the commit above with these files staged into it, built as above. This
says how the pieces are put together; it is not legal advice.
"""


def qemu_target(args: argparse.Namespace) -> None:
    version = workspace_version()
    identity = qemu_target_identity()
    top = f"embsim-{version}-qemu-target"
    entries = [
        (f"{top}/README.md", qemu_target_readme(version, identity).encode(), False)
    ]
    for path, executable in tracked(QEMU_TARGET):
        relative = path[len(QEMU_TARGET) + 1 :]
        entries.append(
            (f"{top}/qemu-target/{relative}", (ROOT / path).read_bytes(), executable)
        )
    for licence in ("LICENSES/GPL-2.0-or-later.txt", "LICENSES/LGPL-2.1-or-later.txt"):
        entries.append((f"{top}/{licence}", (ROOT / licence).read_bytes(), False))
    archive = Path(args.out) / f"{top}.tar.gz"
    write_archive(archive, entries, source_date_epoch())
    write_sha256(archive)
    print(
        f"wrote {archive} ({archive.stat().st_size} bytes), target identity {identity}"
    )


def assets(directory: Path) -> list[Path]:
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file()
        and path.name != "SHA256SUMS"
        and not path.name.endswith(".sha256")
    )


def checksums(args: argparse.Namespace) -> None:
    directory = Path(args.dir)
    found = assets(directory)
    if not found:
        fail(f"{directory} holds no assets")
    lines = []
    for asset in found:
        digest = sha256_of(asset)
        recorded = directory / f"{asset.name}.sha256"
        if recorded.is_file():
            said = recorded.read_text().split()
            if said[:2] != [digest, asset.name]:
                fail(f"{recorded} says {' '.join(said)}, and {asset.name} is {digest}")
        else:
            write_sha256(asset)
        lines.append(f"{digest}  {asset.name}\n")
    (directory / "SHA256SUMS").write_text("".join(lines))
    print(f"SHA256SUMS: {len(lines)} assets")


def notes(args: argparse.Namespace) -> None:
    version = workspace_version()
    directory = Path(args.dir)
    present = {asset.name for asset in assets(directory)}
    rows = []
    for target, about in TARGETS.items():
        name = f"embsim-{version}-{target}.tar.gz"
        if name not in present:
            fail(f"the release has no {name}: its `release-binaries` job uploaded none")
        rows.append((name, f"`embsim` for {about['what']}"))
    source = f"embsim-{version}-qemu-target.tar.gz"
    if source not in present:
        fail(f"the release has no {source}: run `release.py qemu-target`")
    tag, commit = qemu_pin()
    rows.append(
        (
            source,
            f"the P2 target `embsim qemu install` builds `qemu-system-p2` from, "
            f"target identity `{qemu_target_identity()}`, staged into QEMU {tag} "
            f"(`{commit[:12]}`)",
        )
    )
    unknown = sorted(present - {name for name, _ in rows})
    if unknown:
        fail(
            f"{directory} holds assets the notes do not describe: {', '.join(unknown)}"
        )
    text = changelog_section(version)
    text += "\n## Assets\n\n| File | What it is |\n|---|---|\n"
    for name, what in rows:
        text += f"| `{name}` | {what} |\n"
    text += "| `SHA256SUMS`, `<archive>.sha256` | the SHA-256 of each archive |\n"
    text += (
        "\nInstalling, and getting `qemu-system-p2`: the README's "
        f"[Install](https://github.com/RileyMcCarthy/embsim/blob/v{version}/README.md#install).\n"
    )
    Path(args.out).write_text(text, encoding="utf-8")
    print(f"wrote {args.out}")


def check(args: argparse.Namespace) -> None:
    version = workspace_version()
    tag = the_tag(args.tag)
    check_tag(version, tag)
    changelog_section(version)
    print(
        f"embsim {version}: "
        + (f"tag {tag}, " if tag else "no tag, ")
        + "CHANGELOG.md has its section"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    commands = parser.add_subparsers(dest="command", required=True)
    one = commands.add_parser(
        "check", help="the tag is the version; the changelog has it"
    )
    one.add_argument(
        "--tag", help="the tag being released (default: a tag build's own)"
    )
    one.set_defaults(handler=check)
    one = commands.add_parser("binary", help="run and package one platform's embsim")
    one.add_argument(
        "--target", required=True, help="the target triple it was built for"
    )
    one.add_argument(
        "--out", default="dist", help="where the archive goes (default: dist)"
    )
    one.add_argument("--no-run", action="store_true", help="package without running it")
    one.set_defaults(handler=binary)
    one = commands.add_parser("qemu-target", help="the P2 target's source archive")
    one.add_argument(
        "--out", default="dist", help="where the archive goes (default: dist)"
    )
    one.set_defaults(handler=qemu_target)
    one = commands.add_parser("checksums", help="check each .sha256; write SHA256SUMS")
    one.add_argument("dir", help="the directory of assets")
    one.set_defaults(handler=checksums)
    one = commands.add_parser("notes", help="the release notes")
    one.add_argument(
        "--dir", default="dist", help="the directory of assets (default: dist)"
    )
    one.add_argument("--out", required=True, help="the file to write")
    one.set_defaults(handler=notes)
    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
