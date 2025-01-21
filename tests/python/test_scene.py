from helios.scene import *


def test_construct_scene_from_xml():
    scene = Scene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")

    assert len(scene.scene_parts) == 5
