# ruff: noqa
from helios.platforms import *
from numpy.lib.recfunctions import unstructured_to_structured


def test_preinstantiated_platforms():
    assert isinstance(sr22(), Platform)
    assert isinstance(quadcopter(), Platform)
    assert isinstance(copter_linearpath(), Platform)
    assert isinstance(tractor(), Platform)
    assert isinstance(tractor_leftside(), Platform)
    assert isinstance(vehicle_linearpath(), Platform)
    assert isinstance(vmx_450_car_left(), Platform)
    assert isinstance(vmx_450_car_right(), Platform)
    assert isinstance(vmq_1ha_car(), Platform)
    assert isinstance(simple_linearpath(), Platform)
    assert isinstance(tripod(), Platform)


def test_platform_defaults():
    Platform()
    sps = StaticPlatformSettings()
    dps = DynamicPlatformSettings()
    traj_settings = TrajectorySettings()
    dps_t = DynamicPlatformSettings(trajectory_settings=traj_settings)
    pd = Platform(platform_settings=dps)
    pd = Platform(platform_settings=dps_t)
    ps = Platform(platform_settings=sps)


def test_traj_from_np_loading():
    traj = np.arange((70)).reshape((10, 7))
    traj = unstructured_to_structured(traj, dtype=traj_dtype)

    traj_settings = TrajectorySettings()
    dps = DynamicPlatformSettings(trajectory_settings=traj_settings)
    p = Platform(platform_settings=dps, trajectory=traj)

    assert p.trajectory.shape == (10,)
    assert p.trajectory["x"].shape == (10,)
    assert len(p.trajectory[0]) == 7


def test_traj_from_csv_loading():
    tps = TrajectoryParserSettings()
    traj_settings = TrajectorySettings(trajectory_parser_settings=tps)
    dps = DynamicPlatformSettings(trajectory_settings=traj_settings)
    p = Platform(platform_settings=dps)

    csv = "data/trajectories/cycloid.trj"
    p.load_traj_csv(csv=csv)
    assert p.trajectory.shape == (51,)
    t = np.void(
        [(3.7, -60.0, 60.0, 330.7, 13.002584, 1.122905, 400.0)], dtype=traj_dtype
    )
    assert all([a == b for a, b in zip(p.trajectory[0], t[0])])


def test_traj_from_csv_reordering():
    tps = TrajectoryParserSettings(
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )
    traj_settings = TrajectorySettings(trajectory_parser_settings=tps)
    dps = DynamicPlatformSettings(trajectory_settings=traj_settings)
    p = Platform(platform_settings=dps)

    csv = "data/trajectories/cycloid.trj"
    p.load_traj_csv(csv=csv)
    assert p.trajectory.shape == (51,)
    t = np.void(
        [(3.7, 13.002584, 1.122905, 400.0, -60.0, 60.0, 330.7)], dtype=traj_dtype
    )
    assert all([a == b for a, b in zip(p.trajectory[0], t[0])])
