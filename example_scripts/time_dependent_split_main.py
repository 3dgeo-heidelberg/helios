import time_dependent_split_functions as tds
import os
import sys
from pathlib import Path
import numpy as np
import tds_argparser
import xml.etree.ElementTree as ET

# Parse arguments.
args = tds_argparser.args

### Paths to relevant files and directories.
# os.chdir("C:/Users/an274/heliospp_alt")
# HELIOS_DIR = Path("C:/Users/an274/heliospp_alt")  # make CLI argument --assets
# sys.path.append(str(HELIOS_DIR))
fixed_gps_time = "2024-07-07 00:00:00"
original_survey = args.survey_file
survey_name = ET.parse(original_survey).find('survey').attrib['name']

assets = [os.getcwd()]
if args.assets_path:
    assets.append(args.assets_path)

if args.output_path:
    outdir = args.output_path
else:
    outdir = '/output'

original_scene = ET.parse(original_survey).find('survey').attrib['scene'].split('#')[0]
original_scene_fname = Path(original_scene).stem
scene_dir = Path(original_scene).parent
scene_name = ET.parse(original_survey).find('survey').attrib['scene'].split('#')[1]
bboxes_obj_dir = Path(scene_dir) / f'{original_scene_fname}_bboxes'
bboxes_pc_dir = Path(outdir) / survey_name / 'bbox_surveys'

merged_bboxes_las = Path(scene_dir) / f'{original_scene_fname}_bboxes.laz'

# create folder where bboxes will be stored
Path(bboxes_obj_dir).mkdir(parents=True, exist_ok=True)

output_interval_clouds = Path(outdir) / survey_name / 'interval_surveys'
merged_intervals = Path(outdir) / survey_name / 'merged_intervals'  # add timestamp?
merged_filtered_intervals = Path(outdir) / survey_name / 'merged_filtered_intervals'  # add timestamp?
final_cloud = Path(outdir) / survey_name / f'{survey_name}_final.laz'  # add timestamp?

import pyhelios

obj_of_int = []
# Create sub scenes with one part each out of the original scene file.
split_scene_files = tds.split_xml(original_scene, scene_dir)


# Initialize arrays that store min and max coordinates for each scene part.
mins = np.zeros(shape=(len(split_scene_files), 3))
maxs = np.zeros(shape=(len(split_scene_files), 3))


# Run simulation for each scene part to get bounding box information.
for i, paths in enumerate(split_scene_files):
    # Overwrite original survey for each scene part scene.
    tds.write_survey(original_survey, paths)

    pyhelios.loggingSilent()
    # Build simulation parameters
    simBuilder = pyhelios.SimulationBuilder(
        str(original_survey),
        assets,
        outdir
    )
    simBuilder.setNumThreads(0)
    simBuilder.setRebuildScene(True)
    simBuilder.setLasOutput(True)
    simBuilder.setZipOutput(True)

    simB = simBuilder.build()
    scene = simB.sim.getScene()
    shift = scene.getShift()
    part = scene.getScenePart(0)
    part.computeBound()
    bbox = part.getBound()

    min_coords = [bbox.getMinVertex().getPosition().x + shift.x,
                  bbox.getMinVertex().getPosition().y + shift.y,
                  bbox.getMinVertex().getPosition().z + shift.z]
    max_coords = [bbox.getMaxVertex().getPosition().x + shift.x,
                  bbox.getMaxVertex().getPosition().y + shift.y,
                  bbox.getMaxVertex().getPosition().z + shift.z]

    mins[i] = min_coords
    maxs[i] = max_coords


# Create .objs using the min, max values of the scene parts. Obj paths are stored in list.
objs_outfiles = []
for i in range(len(mins)):
    obj_outfile = tds.create_obj_box(mins[i], maxs[i], f"bbox_{i+1}.obj", bboxes_obj_dir)
    objs_outfiles.append(obj_outfile)


# Writes scenes and surveys with one bbox .obj each.
bbox_scene_outfiles = tds.write_scene_string(bboxes_obj_dir, objs_outfiles)
survey_outfiles = tds.write_multiple_surveys(original_survey, bbox_scene_outfiles, bboxes_obj_dir, f"bbox_survey")


# Run simulation for each bbox survey.
for path in survey_outfiles:
    pyhelios.loggingSilent()
    # Build simulation parameters

    simBuilder = pyhelios.SimulationBuilder(
        str(path),
        assets,
        str(bboxes_pc_dir)
    )
    simBuilder.setNumThreads(0)
    simBuilder.setRebuildScene(True)
    simBuilder.setLasOutput(True)
    simBuilder.setZipOutput(True)
    simBuilder.setFixedGpsTimeStart(fixed_gps_time)
    simB = simBuilder.build()

    sim = simBuilder.build()
    sim.start()
    sim.join()



# Merges all legs of all bbox surveys into one las/laz
sub_dirs = []
paths = []
for fil in os.listdir(bboxes_pc_dir):
        sub_dirs.append(os.path.join(bboxes_pc_dir,fil))
for i, sub_dir in enumerate(sub_dirs):

        for fil in os.listdir(sub_dir):
            paths.append(os.path.join(sub_dir, fil))
tds.laz_merge(paths, merged_bboxes_las)


# Checks for scene parts in a user defined interval
interval = 30
obj_ids = tds.objs_in_interval(merged_bboxes_las, interval)


# Writes scenes and surveys for intervals.
interval_scene_outfiles = tds.gen_interval_scene(original_scene, scene_dir, obj_ids, obj_of_int)
interval_surveys = tds.write_multiple_surveys(original_survey, interval_scene_outfiles, scene_dir, f"interval_survey")


# Run simulation for each interval survey.
for path in interval_surveys:
    pyhelios.loggingSilent()
    pyhelios.setDefaultRandomnessGeneratorSeed("123")

    # Build simulation parameters

    simBuilder = pyhelios.SimulationBuilder(
        str(path),
        assets,
        str(output_interval_clouds)
    )
    simBuilder.setNumThreads(0)
    simBuilder.setRebuildScene(True)
    simBuilder.setLasOutput(True)
    simBuilder.setZipOutput(False)
    simBuilder.setFixedGpsTimeStart(fixed_gps_time)

    sim = simBuilder.build()
    sim.start()
    sim.join()


# Merge legs of interval and write them to separate interval pcs.
pc_paths = []
sub_dirs = []
for file in os.listdir(output_interval_clouds):
        sub_dirs.append(os.path.join(output_interval_clouds,file))

for i, sub_dir in enumerate(sub_dirs):
        paths = []
        for file in os.listdir(sub_dir):
            paths.append(os.path.join(sub_dir, file))
        tds.laz_merge(paths, f"{merged_intervals}merged_interval_{i+1}.laz")
        pc_paths.append(f"{merged_intervals}{i+1}.laz")


# Filter merged interval pcs to points inside the interval time.
tds.filter_and_write(pc_paths, merged_filtered_intervals, obj_ids, obj_of_int, interval)


# Merge filtered interval pcs to final pc.
filtered_clouds_path = []
for file in os.listdir(merged_filtered_intervals):
    filtered_clouds_path.append(os.path.join(merged_filtered_intervals, file))
tds.laz_merge(filtered_clouds_path, final_cloud )
