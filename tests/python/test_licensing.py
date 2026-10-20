from doiggie.license import (
    License,
    LicenseIdentifier,
    LicenseIdentifierScheme,
    LicenseReference,
    LicenseReferenceRole,
)

import helios
from helios.licensing import record_licensing_observation
from helios.scene import StaticScene
from helios.survey import Survey


def test_licensing_report_aggregates_doi_loads():
    license_ = License(
        name="Creative Commons Attribution-ShareAlike 4.0",
        description="Creative Commons license metadata",
        identifiers=(
            LicenseIdentifier(
                scheme=LicenseIdentifierScheme.SPDX,
                value="CC-BY-SA-4.0",
            ),
        ),
        references=(
            LicenseReference(
                role=LicenseReferenceRole.TEXT,
                uri="https://creativecommons.org/licenses/by-sa/4.0/legalcode",
            ),
        ),
        copyright="Example authors",
    )
    for model_class in (Survey, Survey, StaticScene):
        record_licensing_observation(
            doi="10.1234/example",
            model_class=model_class,
            licenses=(license_,),
            filename=None,
            archive_filename="bundle.zip",
        )

    report = helios.licensing_report()

    assert report.count("DOI: 10.1234/example") == 1
    assert "Loaded objects: StaticScene × 1, Survey × 2" in report
    assert "Archives: bundle.zip" in report
    assert "License: Creative Commons Attribution-ShareAlike 4.0" in report
    assert "Description: Creative Commons license metadata" in report
    assert "Identifier (spdx): CC-BY-SA-4.0" in report
    assert (
        "Reference (text): "
        "https://creativecommons.org/licenses/by-sa/4.0/legalcode" in report
    )
    assert "Copyright: Example authors" in report
    assert "Requirements:" not in report
    assert "Compatibility:" not in report
    assert report.endswith(
        "This report includes only data loaded via from_doi(). Other data may also "
        "have licenses attached."
    )


def test_licensing_report_marks_missing_metadata_as_unknown():
    record_licensing_observation(
        doi="10.1234/unknown",
        model_class=Survey,
        licenses=(),
        filename=None,
        archive_filename=None,
    )

    report = helios.licensing_report()

    assert "License: not reported" in report
