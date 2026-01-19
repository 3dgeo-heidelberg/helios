# from helios.scenepart import ScenePart
# from helios.scene import Scene
# import numpy as np


# ### different checks for transformation
# def test_no_transformation(scene_part):
#     initial_origin = scene_part.origin.copy()
#     scene_part.transform()
#     assert np.allclose(scene_part.origin, initial_origin)

# def test_translation_only(scene_part):
#     translation_vector = np.array([1.0, 2.0, 3.0])
#     scene_part.transform(translation=translation_vector)
#     assert np.allclose(scene_part.origin, np.array([1.0, 2.0, 3.0]))

# def test_scaling_only(scene_part):
#     initial_scale = scene_part.scale
#     scene_part.transform(scale=2.0)
#     assert scene_part.scale == 2.0 * initial_scale

# def test_translation_and_scaling(scene_part):
#     translation_vector = np.array([1.0, 2.0, 3.0])
#     initial_scale = scene_part.scale
#     scene_part.transform(translation=translation_vector, scale=2.0)
#     assert np.allclose(scene_part.origin, np.array([1.0, 2.0, 3.0]))  # Check translation
#     assert scene_part.scale == 2.0 * initial_scale  # Check scaling

# def test_not_on_ground(scene_part):
#     translation_vector = np.array([1.0, 2.0, 3.0])
#     scene_part.transform(translation=translation_vector, scale=None, is_on_ground=False)
#     assert scene_part.is_on_ground is False

# def test_apply_to_specific_axis(scene_part):
#     scene_part.transform(translation=np.array([1.0, 0.0, 0.0]), apply_to_axis=0)
#     assert np.allclose(scene_part.origin, np.array([1.0, 0.0, 0.0]))

# ### IMPORTANT In general, think about usage of quaternions for rotation in the project!!!!!!!!!!!!!

# def test_rotate_using_axis_and_angle(scene_part):
#     scene_part.rotate(axis=np.array([0, 0, 1]), angle=90)  # Rotate 90 degrees around z-axis
#     assert np.allclose(scene_part.rotation.quaternion, [0, 0, 0.70710678, 0.70710678])


# def test_rotate_with_origin(scene_part):
#     origin = np.array([1.0, 1.0, 1.0])
#     scene_part.rotate(axis=np.array([0, 0, 1]), angle=90, origin=origin)
#     assert np.allclose(scene_part.origin, [1.0, 1.0, 1.0])  # Origin remains unchanged

# def test_rotate_with_custom_origin(scene_part):
#     custom_origin = np.array([2.0, 2.0, 2.0])
#     scene_part.rotate(axis=np.array([0, 0, 1]), angle=90, origin=custom_origin)
#     assert np.allclose(scene_part.origin, [2.0, 2.0, 2.0])  # Origin is updated to custom_origin


# ### motion tests
# def test_make_motion_translation_only(scene_part):
#     translation = np.array([0.5, 0, 0])
#     scene_part.make_motion(translation=translation, loop=1)
#     expected_origin = np.array([0.5, 0.0, 0.0])
#     assert np.allclose(scene_part.origin, expected_origin)


# def test_make_motion_translation_and_rotation(scene_part):
#     translation = np.array([0.5, 0, 0])
#     rotation_axis = np.array([0, 0, 1])
#     rotation_angle = 90  # degrees
#     scene_part.make_motion(translation=translation, rotation_axis=rotation_axis, rotation_angle=rotation_angle, radians=False, loop=1)
#     expected_origin = np.array([0.5, 0.0, 0.0])
#     assert np.allclose(scene_part.origin, expected_origin)

# def test_make_motion_with_loop(scene_part):
#     translation = np.array([0.2, 0, 0])
#     loop = 50
#     scene_part.make_motion(translation=translation, rotation_axis=None, rotation_angle=None, loop=loop)
#     expected_origin = np.array([0.2 * loop, 0.0, 0.0])
#     assert np.allclose(scene_part.origin, expected_origin)

# def test_make_motion_with_rotation_center(scene_part):
#     translation = np.array([0.5, 0, 0])
#     rotation_axis = np.array([0, 0, 1])
#     rotation_angle = 90  # degrees
#     rotation_center = np.array([1.0, 1.0, 0.0])
#     scene_part.make_motion(translation=translation, rotation_axis=rotation_axis, rotation_angle=rotation_angle, radians=False, rotation_center=rotation_center, loop=1)
#     # Verify the origin and rotation
#     expected_origin = np.array([1.5, 1.0, 0.0])
#     assert np.allclose(scene_part.origin, expected_origin)


# ## test motion_sequence
# def test_make_motion_sequence_translations_only(scene_part):
#     translations = [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])]
#     scene_part.make_motion_sequence(translations=translations, rotation_axes=None, rotation_angles=None)
#     expected_origin = np.array([1.0, 1.0, 1.0])
#     assert np.allclose(scene_part.origin, expected_origin)


# def test_make_motion_sequence_translation_and_rotation(scene_part):
#     translations = [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])]
#     rotation_axes = [np.array([0, 0, 1]), np.array([0, 1, 0]), np.array([1, 0, 0])]
#     rotation_angles = [90, 90, 90]  # degrees
#     scene_part.make_motion_sequence(translations=translations, rotation_axes=rotation_axes, rotation_angles=rotation_angles, radians=False)
#     expected_origin = np.array([1.0, 1.0, 1.0])
#     assert np.allclose(scene_part.origin, expected_origin)


# def test_make_motion_sequence_with_rotation_centers(scene_part):
#     translations = [np.array([1.0, 0.0, 0.0])]
#     rotation_axes = [np.array([0, 0, 1])]
#     rotation_angles = [90]  # degrees
#     rotation_centers = [np.array([0.0, 0.0, 0.0])]
#     scene_part.make_motion_sequence(translations=translations, rotation_axes=rotation_axes, rotation_angles=rotation_angles, radians=False, rotation_centers=rotation_centers)
#     expected_origin = np.array([1.0, 0.0, 0.0])
#     assert np.allclose(scene_part.origin, expected_origin)
