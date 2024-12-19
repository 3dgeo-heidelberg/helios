import shutil
from math import pi, cos, sin
from scipy import stats as sstats
import numpy as np
import tqdm
import matplotlib.pyplot as plt
import pyhelios
import sys

try:
    import polyscope as ps
except ImportError:
    print('Polyscope is not installed. Please install Polyscope with "pip install polyscope".')
    sys.exit()


DELETE_AFTER = True

scene = """
<?xml version="1.0" encoding="UTF-8"?>
<document>
    <scene id="plane_scene" name="plane scene">
        <part id="1">
            <filter type="objloader">
                <param type="string" key="filepath" value="data/sceneparts/basic/groundplane/groundplane.obj" />
            </filter>
            <filter type="scale">
                <param type="double" key="scale" value="100" />
            </filter>
        </part>
        <part id="2">
            <filter type="objloader">
                <param type="string" key="filepath" value="data/sceneparts/basic/groundplane/groundplane.obj" />
            </filter>
            <filter type="scale">
                <param type="double" key="scale" value="100" />
            </filter>
        </part>
    </scene>
</document>"""

survey = """
<?xml version="1.0" encoding="UTF-8"?>
<document>
    <!-- Default scanner settings: -->
    <survey defaultScannerSettings="profile1" name="plane_demo" scene="data/scenes/plane_scene.xml#plane_scene" platform="data/platforms.xml#tripod" scanner="data/scanners_tls.xml#riegl_vz400">
        <FWFSettings binSize_ns="0.25" beamSampleQuality="1" />
        <leg>
            <platformSettings x="0" y="-10" z="0" onGround="false" />
            <scannerSettings active="true" pulseFreq_hz="100000" scanAngle_deg="50.0" scanFreq_hz="120" headRotatePerSec_deg="20.0" headRotateStart_deg="-10" headRotateStop_deg="10" trajectoryTimeInterval_s="0.1"/>
        </leg>
    </survey>
</document>"""

with open('data/surveys/plane_survey.xml', 'w') as f:
    f.write(survey)

tls_pos = np.array([[0, -10, 0]])

pyhelios.default_rand_generator_seed("42")
# Build multiple simulations

angles = np.arange(0, 90, 10)

sigmas = []
q975s = []
q025s = []
lods_m3c2 = []
lods_ep = []
nptss = []

ps.init()

for angle in angles:
    print("Angle: {ang:.3f} deg".format(ang=angle))
    with open("data/scenes/plane_scene.xml", "w") as f:
        f.write(scene)
    angle = angle * pi / 180
    dists = []

    simBuilder = pyhelios.SimulationBuilder(
        'data/surveys/plane_survey.xml',
        'assets/',
        'output/'
    )
    simBuilder.setNumThreads(0)
    simBuilder.setLasOutput(True)
    simBuilder.setZipOutput(True)
    simBuilder.setCallbackFrequency(0)  # Run without callback
    simBuilder.setExportToFile(True)    # Enable export point cloud to file
    simBuilder.setFinalOutput(True)     # Return output at join
    # General rotation
    simBuilder.addRotateFilter(cos(pi/4), sin(pi/4), 0, 0, "")
    # Surface 1 rotation
    simBuilder.addRotateFilter(
        cos(-angle/2), 0, 0, sin(-angle/2), "1"
    )
    # Surface 2 rotation
    simBuilder.addRotateFilter(
        cos(angle/2), 0, 0, sin(angle/2), "2"
    )
    simBuilder.setLegNoiseDisabled(True)
    simBuilder.setRebuildScene(True)
    simBuilder.setWriteWaveform(False)
    simBuilder.setCalcEchowidth(False)
    simBuilder.setFullwaveNoise(False)
    simBuilder.setPlatformNoiseDisabled(True)
    sim = simBuilder.build()
    
    detector = sim.getScanner().detector
    detector.accuracy = 0.005

    for i in tqdm.tqdm(range(5)):
        sim0 = sim.sim.copy()
        sim0.start()
        meas = sim0.join()[0]
        

        sim1 = sim.sim.copy()
        sim1.start()
        meas1 = sim1.join()[0]

        del sim1
        del sim0

        points = [[meas[m].position[0],
                  meas[m].position[1],
                  meas[m].position[2]] for m in range(len(meas))]
        points = np.array(points)
        points = points[np.linalg.norm(points, axis=1) < 1, :]  # 1 m searchrad, "corepoint" at
        points1 = [[meas1[m].position[0],
                  meas1[m].position[1],
                  meas1[m].position[2]] for m in range(len(meas1))]
        points1 = np.array(points1)
        points1 = points1[np.linalg.norm(points1, axis=1) < 1, :]

        dists.append(np.mean(points[:, 1]) - np.mean(points1[:, 1]))

    q975 = np.percentile(dists, 97.5)
    q025 = np.percentile(dists, 2.5)
    q025s.append(q025)
    q975s.append(q975)
    sigma_y_2 = np.var(points[:, 1])
    n_points = points.shape[0]

    lod_m3c2 = 1.96 * np.sqrt((sigma_y_2 / n_points) + (sigma_y_2 / n_points))
    sigma2_mean_ep = detector.accuracy**2  # / n_points

    sigmaD = 2 * (n_points - 1) * sigma2_mean_ep / (
                2 * n_points - 2)  # /n_points # adaption because our stddev comes from error prop and is not an estimate; weighted average of the matrices
    p = 1  # three-dimensional
    # lods_m3c2_ep.append(1.96 * np.sqrt(sigma2_mean_ep/n_points)) #--> good solution

    Tsqalt = 1 / (sigmaD) * (n_points ** 2 / (2 * n_points))

    # lods_m3c2_ep.append(np.sqrt(sstats.chi2.cdf(.95, p) / Tsqalt))
    t_rel2 = Tsqalt * (2 * n_points - p - 1) / (p * (2 * n_points - 2))
    lod_ep = np.sqrt(sstats.f.ppf(.95, p, 2 * n_points - p - 1) / t_rel2)
    lods_m3c2.append(lod_m3c2)
    lods_ep.append(lod_ep)
    print(n_points, lod_ep, q975, sigma_y_2)
    nptss.append(n_points)
    ps_cloud = ps.register_point_cloud("meas", points)
    ps_tls = ps.register_point_cloud("TLS", tls_pos)
    ps.set_up_dir("z_up")
    # ps.show()



# angles = 90 - angles

fig, ax1 = plt.subplots()

ax1.fill_between(angles, q975s, q025s, color='xkcd:lavender', alpha=0.4)
ax1.plot(angles, lods_m3c2, color='xkcd:brick red', label='LoD M3C2')
ax1.plot(angles, lods_ep, color='xkcd:bright red', label='LoD M3C2-EP')
ax1.set_ylabel(r"Level of Detection [m]", color='tab:red')
ax1.set_xlabel(r"Angle $\varphi$ (between plane normal and view direction) [deg]")
ax1.tick_params(axis='y', labelcolor='tab:red')
ax1.set_ylim([0, 0.005])

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Number of points', color=color)  # we already handled the x-label with ax1
ax2.plot(angles, nptss, color=color)
ax2.tick_params(axis='y', labelcolor=color)

fig.legend()
plt.show()

# delete everything in output folder
if DELETE_AFTER:
    shutil.rmtree("output/plane_demo")
