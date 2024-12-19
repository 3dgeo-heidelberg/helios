from .test_demo_scenes import run_helios_executable, find_playback_dir
from pathlib import Path
import datetime



def run_helios_pyhelios(survey_path: Path, output_dir: Path, options=None) -> Path:
    import pyhelios
    pyhelios.default_rand_generator_seed("43")
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
    return find_playback_dir(survey_path, output_dir)


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
    r1 = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                             output_dir,
                             options={'gpsStartTime': ''})
    # run with posix ts:
    r2 = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                             output_dir,
                             options={'gpsStartTime': f'{unixtime:.0f}'})
    # run with string ts:
    r3 = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                             output_dir,
                             options={'gpsStartTime': stringtime})

    assert (r1 / 'leg000_points.xyz').exists()
