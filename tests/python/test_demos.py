from helios.survey import Survey
from .compare import compare_clouds, speed_from_traj

import pytest


def test_arbaro_tls(regression_data, persisting_output_dir):
    survey = Survey.from_xml("data/surveys/demo/tls_arbaro_demo.xml")
    survey.gps_time = "2022-01-01 00:00:00"
    path = survey.run(format="las", output_dir=persisting_output_dir)
    assert len(list(path.glob("*.las"))) == 2
    assert len(list(path.glob("*.txt"))) == 2

    with open(path / "leg000_trajectory.txt", "r") as f:
        line = f.readline()
        assert line.startswith("1.0000 25.5000 -6.9727")

    if regression_data:
        compare_clouds(
            path / "leg000_points.las", regression_data / "arbaro_tls_leg000_points.las"
        )
        compare_clouds(
            path / "leg001_points.las", regression_data / "arbaro_tls_leg001_points.las"
        )


def test_tiffloader_als(regression_data, persisting_output_dir):
    survey = Survey.from_xml("data/test/als_hd_demo_tiff_min.xml")
    survey.gps_time = "2022-01-01 00:00:00"
    path = survey.run(format="las", output_dir=persisting_output_dir)

    assert len(list(path.glob("*.las"))) == 3
    assert len(list(path.glob("*.txt"))) == 3

    with open(path / "leg000_trajectory.txt", "r") as f:
        line = f.readline()
        assert line.startswith("-2172.6258 887.1097 1235.5")
    if regression_data:
        compare_clouds(
            path / "leg000_points.las",
            regression_data / "tiffloader_als_leg000_points.las",
        )
        compare_clouds(
            path / "leg001_points.las",
            regression_data / "tiffloader_als_leg001_points.las",
        )
        compare_clouds(
            path / "leg002_points.las",
            regression_data / "tiffloader_als_leg002_points.las",
        )


def test_xyz_voxels_tls(regression_data, persisting_output_dir):
    survey = Survey.from_xml("data/surveys/voxels/tls_sphere_xyzloader_normals.xml")
    survey.gps_time = "2022-01-01 00:00:00"
    path = survey.run(format="las", output_dir=persisting_output_dir)

    assert len(list(path.glob("*.las"))) == 1
    assert len(list(path.glob("*.txt"))) == 1

    if regression_data:
        compare_clouds(
            path / "leg000_points.las",
            regression_data / "xyzVoxels_tls_leg000_points.las",
        )


def test_interpolated_traj(regression_data, persisting_output_dir):
    survey = Survey.from_xml("data/surveys/demo/als_interpolated_trajectory.xml")
    survey.gps_time = "2022-01-01 00:00:00"
    path = survey.run(format="laz", output_dir=persisting_output_dir)

    assert len(list(path.glob("*.laz"))) == 4
    assert len(list(path.glob("*.txt"))) == 4

    with open(path / "leg000_trajectory.txt", "r") as f:
        for _ in range(3):
            next(f)
        line = f.readline()
        assert line.startswith("13.4766 1.7424 400.0000")

    if regression_data:
        compare_clouds(
            path / "leg000_points.laz",
            regression_data / "interpolated_traj_leg000_points.laz",
        )
        compare_clouds(
            path / "leg001_points.laz",
            regression_data / "interpolated_traj_leg001_points.laz",
        )
        compare_clouds(
            path / "leg002_points.laz",
            regression_data / "interpolated_traj_leg002_points.laz",
        )
        compare_clouds(
            path / "leg003_points.laz",
            regression_data / "interpolated_traj_leg003_points.laz",
        )


def test_quadcopter(regression_data, persisting_output_dir):
    survey = Survey.from_xml(
        "data/surveys/toyblocks/uls_toyblocks_survey_scene_combo.xml"
    )
    survey.gps_time = "2022-01-01 00:00:00"
    path = survey.run(format="laz", output_dir=persisting_output_dir)

    assert len(list(path.glob("*.laz"))) == 4
    assert len(list(path.glob("*.txt"))) == 4

    assert speed_from_traj(path / "leg000_trajectory.txt") == pytest.approx(10.0, 0.001)
    assert speed_from_traj(path / "leg002_trajectory.txt") == pytest.approx(7.0, 0.001)
    assert speed_from_traj(path / "leg004_trajectory.txt") == pytest.approx(4.0, 0.001)

    with open(path / "leg000_trajectory.txt", "r") as f:
        for _ in range(3):
            next(f)
        line = f.readline()
        assert line.startswith("-69.9983 -60.0000 59.7611")

    if regression_data:
        compare_clouds(
            path / "leg000_points.laz", regression_data / "quadcopter_leg000_points.laz"
        )
        compare_clouds(
            path / "leg001_points.laz", regression_data / "quadcopter_leg001_points.laz"
        )
        compare_clouds(
            path / "leg002_points.laz", regression_data / "quadcopter_leg002_points.laz"
        )
        compare_clouds(
            path / "leg003_points.laz", regression_data / "quadcopter_leg003_points.laz"
        )


def test_als_multichannel_split(regression_data, persisting_output_dir):
    survey = Survey.from_xml("data/surveys/demo/light_als_toyblocks_multiscanner.xml")
    survey.gps_time = "2022-01-01 00:00:00"
    path = survey.run(
        format="laz", split_by_channel=True, output_dir=persisting_output_dir
    )

    assert len(list(path.glob("*.laz"))) == 6

    if regression_data:
        compare_clouds(
            path / "leg000_points_dev0.laz",
            regression_data / "als_multichannel_split_leg000_points_dev0.laz",
        )
        compare_clouds(
            path / "leg000_points_dev1.laz",
            regression_data / "als_multichannel_split_leg000_points_dev1.laz",
        )
        compare_clouds(
            path / "leg000_points_dev2.laz",
            regression_data / "als_multichannel_split_leg000_points_dev2.laz",
        )
        compare_clouds(
            path / "leg002_points_dev0.laz",
            regression_data / "als_multichannel_split_leg002_points_dev0.laz",
        )
        compare_clouds(
            path / "leg002_points_dev1.laz",
            regression_data / "als_multichannel_split_leg002_points_dev1.laz",
        )
        compare_clouds(
            path / "leg002_points_dev2.laz",
            regression_data / "als_multichannel_split_leg002_points_dev2.laz",
        )
