import pytest

import helios.scanner as scanner_module
from helios.scanner import *


def test_preinstantiated_scanners():
    for scanner_name in list_scanners():
        scanner_factory = getattr(scanner_module, scanner_name)
        assert isinstance(scanner_factory(), Scanner)


def test_list_scanners_matches_registry():
    assert list_scanners() == list(scanner_module.SCANNER_REGISTRY.keys())


def test_scanner_from_name():
    for scanner_name in list_scanners():
        assert isinstance(scanner_from_name(scanner_name), Scanner)


def test_scanner_from_name_invalid():
    with pytest.raises(ValueError, match="Unknown scanner"):
        scanner_from_name("not-a-valid-scanner")


def test_scanneer_flag_from_xml_set():
    from helios.utils import is_xml_loaded

    scanner = Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50")
    assert is_xml_loaded(scanner)
