from helios.scanner import *


def test_preinstantiated_scanners():
    assert isinstance(leica_als50(), Scanner)
    assert isinstance(leica_als50_ii(), Scanner)
    assert isinstance(optech_2033(), Scanner)
    assert isinstance(optech_3100(), Scanner)
    assert isinstance(optech_galaxy(), Scanner)
    assert isinstance(riegl_lms_q560(), Scanner)
    assert isinstance(riegl_lms_q780(), Scanner)
    assert isinstance(riegl_vq_780i(), Scanner)
    assert isinstance(riegl_vux_1uav(), Scanner)
    assert isinstance(riegl_vux_1uav22(), Scanner)
    assert isinstance(riegl_vux_1ha22(), Scanner)
    assert isinstance(riegl_vq_880g(), Scanner)
    assert isinstance(riegl_vq_1560i(), Scanner)
    assert isinstance(livox_mid70(), Scanner)
    assert isinstance(livox_mid100(), Scanner)
    assert isinstance(livox_mid100a(), Scanner)
    assert isinstance(livox_mid100b(), Scanner)
    assert isinstance(livox_mid100c(), Scanner)
    assert isinstance(riegl_vz_400(), Scanner)
    assert isinstance(riegl_vz_1000(), Scanner)
    assert isinstance(riegl_vq_450(), Scanner)
    assert isinstance(livox_mid70_tls(), Scanner)
    assert isinstance(vlp16(), Scanner)
    assert isinstance(velodyne_hdl_64e(), Scanner)
    assert isinstance(tractor_scanner(), Scanner)
    assert isinstance(pano_scanner(), Scanner)


def test_scanneer_flag_from_xml_set():
    from helios.utils import is_xml_loaded

    scanner = Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50")
    assert is_xml_loaded(scanner)
