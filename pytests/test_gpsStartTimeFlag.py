from .test_demo_scenes import run_helios_executable, find_playback_dir
from pathlib import Path
import datetime
import numpy as np



def run_helios_pyhelios(survey_path: Path, output_dir: Path, options=None) -> Path:
    import pyhelios
    pyhelios.setDefaultRandomnessGeneratorSeed("43")
    from pyhelios import SimulationBuilder
    simB = SimulationBuilder(
        surveyPath=str(survey_path.absolute()),
        assetsDir=[str(Path("assets"))],
        outputDir=str(output_dir),
    )
    simB.setLasOutput(False)
    simB.setRebuildScene(True)
    simB.setNumThreads(1)
    simB.setKDTJobs(1)
    simB.setFixedGpsTimeStart(options['gpsStartTime'])
    sim = simB.build()

    sim.start()
    output = sim.join()
    sim = None
    return find_playback_dir(survey_path, output_dir), pyhelios.outputToNumpy(output)


def test_gpsStartTimeFlag_exe(output_dir):
    now = datetime.datetime(year=1994, month=1, day=18, hour=1, minute=45, second=22)
    unixtime = datetime.datetime.timestamp(now)
    stringtime = now.strftime('%Y-%m-%d %H:%M:%S')
    # first, run 'as is':
    r1 = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                               output_dir,
                               options=['--gpsStartTime', ''])
    # run with posix ts:
    r2 = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                               output_dir,
                               options=['--gpsStartTime', f'{unixtime:.3f}'])
    # run with string ts:
    r3 = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                               output_dir,
                               options=['--gpsStartTime', stringtime])

    assert (r1 / 'leg000_points.xyz').exists()


def test_gpsStartTimeFlag_pyh(output_dir):
    now = datetime.datetime(year=1994, month=1, day=18, hour=1, minute=45, second=22)
    unixtime = datetime.datetime.timestamp(now)
    stringtime = now.strftime('%Y-%m-%d %H:%M:%S')
    # first, run 'as is':
    r1, _ = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                             output_dir,
                             options={'gpsStartTime': ''})
    # run with posix ts:
    r2, _ = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                             output_dir,
                             options={'gpsStartTime': f'{unixtime:.0f}'})
    # run with string ts:
    r3, _ = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                             output_dir,
                             options={'gpsStartTime': stringtime})

    assert (r1 / 'leg000_points.xyz').exists()


def test_matching_gpstime_meas_traj(output_dir):
    now = datetime.datetime(year=1994, month=1, day=18, hour=1, minute=45, second=22)
    stringtime = now.strftime('%Y-%m-%d %H:%M:%S')
    # run with string ts:
    r, (meas, traj) = run_helios_pyhelios(Path('data') / 'test' / 'als_hd_demo_tiff_min.xml',
                              output_dir,
                              options={'gpsStartTime': stringtime})
    gpstime_meas = meas[:, 16]
    gpstime_traj = traj[:, 3]

    gpstime_traj_selection = gpstime_traj[5]
    np.testing.assert_almost_equal(gpstime_traj.max(), gpstime_meas.max(), decimal=1)
    # find a point in the gpstime meas array that is close to the gpstime traj with some margin
    idx = np.where(np.isclose(gpstime_meas, gpstime_traj_selection, atol=0.1))[0]
    assert len(idx) > 0, f"no matching gps time found for {gpstime_traj_selection} in meas"
