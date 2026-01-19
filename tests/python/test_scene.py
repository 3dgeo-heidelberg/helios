# from helios.scene import Scene
# from helios.scenepart import ScenePart, ObjectType

# def test_add_scene_part(sample_scene):
#     scene_part = ScenePart(object_type=ObjectType.STATIC_OBJECT)
#     sample_scene.add_scene_part(scene_part)
#     assert len(sample_scene.scene_parts) == 1
#     assert scene_part in sample_scene.scene_parts


# def test_specific_scene_part(sample_scene_with_parts):
#     scene_part = sample_scene_with_parts.specific_scene_part(1)
#     assert scene_part is not None
#     assert scene_part.object_type == ObjectType.DYN_MOVING_OBJECT

# def test_specific_scene_part_out_of_range(sample_scene_with_parts):
#     scene_part = sample_scene_with_parts.specific_scene_part(10)
#     assert scene_part is None

# def test_delete_scene_part(sample_scene_with_parts):
#     initial_len = len(sample_scene_with_parts.scene_parts)
#     assert sample_scene_with_parts.delete_scene_part(0) is True
#     assert len(sample_scene_with_parts.scene_parts) == initial_len - 1

# def test_delete_scene_part_out_of_range(sample_scene_with_parts):
#     initial_len = len(sample_scene_with_parts.scene_parts)
#     assert sample_scene_with_parts.delete_scene_part(10) is False
#     assert len(sample_scene_with_parts.scene_parts) == initial_len
