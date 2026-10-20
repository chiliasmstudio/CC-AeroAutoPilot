"""Fail a CI build when firmware source changes without a matching version bump."""

import subprocess
import sys
import xml.etree.ElementTree as ET


COMPUTER_FILES = {
    "bundle.py",
    "run.lua",
    "fcc.lua",
    "display.lua",
    "fcc-cli.lua",
    "boot.lua",
}
TURTLE_FILES = {"turtle_startup.lua"}


def git(*args):
    return subprocess.check_output(["git", *args], text=True).strip()


def parse_versions(xml_text):
    root = ET.fromstring(xml_text)
    return {
        "computer": parse_version(root.findtext("computer")),
        "turtle": parse_version(root.findtext("turtle")),
    }


def parse_version(value):
    if not value:
        raise ValueError("version.xml is missing a version value")
    parts = value.strip().removeprefix("v").split(".")
    if len(parts) != 3 or not all(part.isdigit() for part in parts):
        raise ValueError(f"invalid semantic version: {value!r}")
    return tuple(int(part) for part in parts)


def main(base_sha):
    changed = set(git("diff", "--name-only", f"{base_sha}...HEAD").splitlines())
    computer_changed = bool(changed & COMPUTER_FILES) or any(path.startswith("modules/") for path in changed)
    turtle_changed = bool(changed & TURTLE_FILES)

    if not computer_changed and not turtle_changed:
        print("No firmware source changes; version bump not required.")
        return 0

    base_xml = git("show", f"{base_sha}:version.xml")
    with open("version.xml", encoding="utf-8") as version_file:
        current_xml = version_file.read()
    base_versions = parse_versions(base_xml)
    current_versions = parse_versions(current_xml)

    missing = []
    for target, changed_source in (("computer", computer_changed), ("turtle", turtle_changed)):
        if changed_source and current_versions[target] <= base_versions[target]:
            missing.append(target)

    if missing:
        targets = ", ".join(missing)
        print(f"Firmware source changed without increasing version.xml: {targets}.")
        print("Use: python bundle.py --bump-computer patch, --bump-turtle patch, or --bump patch.")
        return 1

    print("Firmware source changes have matching version bumps.")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python check_version_bump.py <base-commit>")
        sys.exit(2)
    try:
        sys.exit(main(sys.argv[1]))
    except (subprocess.CalledProcessError, ET.ParseError, ValueError) as error:
        print(f"Version check failed: {error}")
        sys.exit(2)
