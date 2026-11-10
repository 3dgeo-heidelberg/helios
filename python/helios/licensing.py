"""Process-wide licensing observations for objects loaded from DOI repositories."""

from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional

from doiggie.license import License


@dataclass
class _DOIObservation:
    licenses: set[License] = field(default_factory=set)
    loaded_objects: Counter[str] = field(default_factory=Counter)
    files: set[str] = field(default_factory=set)
    archives: set[str] = field(default_factory=set)


# Process-wide storage of licensing observations, keyed by normalized DOI.
_OBSERVATIONS: dict[str, _DOIObservation] = {}

_SCOPE_NOTE = (
    "This report includes only data loaded via from_doi(). Other data may also have "
    "licenses attached."
)


def record_licensing_observation(
    *,
    doi: str,
    model_class: type,
    licenses: Iterable[License],
    filename: Optional[Path],
    archive_filename: Optional[str],
) -> None:
    """Record a successful DOI load in the private process-wide ledger."""

    observation = _OBSERVATIONS.setdefault(doi, _DOIObservation())
    observation.licenses.update(licenses)
    observation.loaded_objects[model_class.__name__] += 1
    if filename is not None:
        observation.files.add(Path(filename).as_posix())
    if archive_filename is not None:
        observation.archives.add(archive_filename)


def licensing_report() -> str:
    """Return a licensing report for every object loaded from a DOI in this process."""

    lines = ["HELIOS++ licensing report", ""]
    if not _OBSERVATIONS:
        lines.append("No objects have been loaded from a DOI in this process.")
        lines.extend(["", _SCOPE_NOTE])
        return "\n".join(lines)

    for doi, observation in sorted(_OBSERVATIONS.items()):
        lines.append(f"DOI: {doi}")
        object_counts = (
            f"{name} × {count}"
            for name, count in sorted(observation.loaded_objects.items())
        )
        lines.append(f"  Loaded objects: {', '.join(object_counts)}")
        if observation.files:
            lines.append(f"  Root files: {', '.join(sorted(observation.files))}")
        if observation.archives:
            lines.append(f"  Archives: {', '.join(sorted(observation.archives))}")

        if not observation.licenses:
            lines.append("  License: not reported")
        else:
            for license_ in sorted(
                observation.licenses, key=lambda item: item.name.casefold()
            ):
                lines.append(f"  License: {license_.name}")
                if license_.description:
                    lines.append(f"    Description: {license_.description}")
                for identifier in license_.identifiers:
                    lines.append(
                        f"    Identifier ({identifier.scheme.value}): {identifier.value}"
                    )
                for reference in license_.references:
                    lines.append(
                        f"    Reference ({reference.role.value}): {reference.uri}"
                    )
                if license_.copyright:
                    lines.append(f"    Copyright: {license_.copyright}")

        lines.append("")

    lines.append(_SCOPE_NOTE)
    return "\n".join(lines).rstrip()


__all__ = ["licensing_report"]
