"""Validates every Cursor-on-Target event XML file in a directory against
`cot_event.xsd` (this project's own XSD, matching the publicly documented
CoT 2.0 event schema's structure; see the README "Honest framing" for why
this is not the official TAK product's exact XSD), plus two semantic
invariants an XSD's `xs:double` type cannot express on its own: latitude
in [-90, 90] and longitude in [-180, 180].

Usage: python3 validate_cot.py <directory-of-.cot-files>
"""
import sys
from pathlib import Path

from lxml import etree


def validate_directory(directory: Path, xsd_path: Path) -> dict:
    schema = etree.XMLSchema(etree.parse(str(xsd_path)))
    files = sorted(directory.glob("*.cot"))

    schema_valid = 0
    range_valid = 0
    failures = []

    for path in files:
        tree = etree.parse(str(path))
        if schema.validate(tree):
            schema_valid += 1
        else:
            failures.append((path.name, "schema", str(schema.error_log)))
            continue

        point = tree.getroot().find("point")
        lat = float(point.get("lat"))
        lon = float(point.get("lon"))
        if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
            range_valid += 1
        else:
            failures.append((path.name, "range", f"lat={lat} lon={lon}"))

    return {
        "total": len(files),
        "schema_valid": schema_valid,
        "range_valid": range_valid,
        "failures": failures,
    }


def main():
    directory = Path(sys.argv[1])
    xsd_path = Path(__file__).parent / "cot_event.xsd"
    result = validate_directory(directory, xsd_path)
    print(f"total_files={result['total']}")
    print(f"schema_valid={result['schema_valid']}/{result['total']}")
    print(f"lat_lon_range_valid={result['range_valid']}/{result['total']}")
    for name, kind, detail in result["failures"]:
        print(f"FAIL {name} ({kind}): {detail}")
    if result["schema_valid"] != result["total"] or result["range_valid"] != result["total"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
