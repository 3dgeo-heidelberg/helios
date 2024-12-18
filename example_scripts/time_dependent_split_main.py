import time_dependent_split_functions as tds
import os
from pathlib import Path
import numpy as np
import tds_argparser
import xml.etree.ElementTree as ET
import pyhelios

# Parse arguments.
args = tds_argparser.args

### Paths to relevant files and directories.
fixed_gps_time = "2024-07-07 00:00:00"
original_survey = args.survey_file
survey_name = ET.parse(original_survey).find('survey').attrib['name']
obj_of_int = args.obj_of_int
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
split_scene_dir = scene_dir / f"{original_scene_fname}_split"
sp_surveys_dir = f"{Path(original_survey).stem}_per_sp"
bboxes_obj_dir = Path(scene_dir) / f'{original_scene_fname}_bboxes'
bboxes_pc_dir = Path(outdir) / survey_name / 'bbox_surveys'

merged_bboxes_pc = Path(scene_dir) / f'{original_scene_fname}_bboxes.laz'

interval_surveys_dir = Path(outdir) / survey_name / 'interval_surveys'
merged_intervals_dir = Path(outdir) / survey_name / 'merged_intervals'  # add timestamp?
merged_filtered_intervals_dir = Path(outdir) / survey_name / 'merged_filtered_intervals'  # add timestamp?
final_pc = Path(outdir) / survey_name / f'{survey_name}_final.laz'  # add timestamp?


# create folders
Path(bboxes_obj_dir).mkdir(parents=True, exist_ok=True)
Path(split_scene_dir).mkdir(parents=True, exist_ok=True)
Path(merged_intervals_dir).mkdir(parents=True, exist_ok=True)
Path(merged_filtered_intervals_dir).mkdir(parents=True, exist_ok=True)

# Create sub scenes with one part each out of the original scene file.
split_scene_files = tds.split_xml(original_scene, split_scene_dir)

# Initialize arrays that store min and max coordinates for each scene part.
mins = np.zeros(shape=(len(split_scene_files), 3))
maxs = np.zeros(shape=(len(split_scene_files), 3))

# Run simulation for each scene part to get bounding box information.
print("\nGetting bounding boxes\n")
for i, paths in enumerate(split_scene_files):
    # Write new survey for each scene part scene based on original survey
    survey = tds.write_survey(original_survey, paths, suffix=f"sub_{i}", 
                              subfolder=sp_surveys_dir)

    pyhelios.loggingSilent()
    # Build simulation parameters
    simBuilder = pyhelios.SimulationBuilder(
        str(survey),
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


# Create bbox .objs using the min, max values of the scene parts and store paths in list
objs_outfiles = []
for i, (min_val, max_val) in enumerate(zip(mins, maxs)):
    obj_outfile = tds.create_obj_box(
        min_val, max_val, f"bbox_{i+1}.obj", bboxes_obj_dir
    )
    objs_outfiles.append(obj_outfile)


# Writes scenes and surveys with one bbox .obj each.
bbox_scene_outfiles = tds.write_bbox_scenes(bboxes_obj_dir, objs_outfiles)
survey_outfiles = tds.write_multiple_surveys(original_survey, 
                                             bbox_scene_outfiles, 
                                             bboxes_obj_dir,
                                             f"bbox_survey")

# Run simulation for each bbox survey.
print("\nSimulating over bounding boxes\n")
bbox_output_files = []
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
    output = sim.join()
    for j in range(output.outpaths.length()):
        bbox_output_files.append(output.outpaths.get(j))


print("\nMerging bounding box point clouds\n")
# Merges all legs of all bbox surveys into one las/laz
tds.laz_merge(bbox_output_files, merged_bboxes_pc)

# Checks for scene parts in a user defined interval
interval = args.time_interval_s
obj_ids = tds.objs_in_interval(merged_bboxes_pc, interval)

# Writes scenes and surveys for intervals.
interval_scene_outfiles = tds.gen_interval_scene(original_scene,
                                                 scene_dir,
                                                 obj_ids,
                                                 obj_of_int)
interval_surveys = tds.write_multiple_surveys(original_survey, 
                                              interval_scene_outfiles,
                                              scene_dir,
                                              f"interval_survey")

interval_output_folders = []
# Run simulation for each interval survey.
print(f"\nSimulating {len(interval_surveys)} separate intervals\n")
for i, path in enumerate(interval_surveys):
    print(f"\nInterval {i+1}")
    pyhelios.loggingSilent()

    # Build simulation parameters
    simBuilder = pyhelios.SimulationBuilder(
        str(path),
        assets,
        str(interval_surveys_dir)
    )
    simBuilder.setNumThreads(0)
    simBuilder.setRebuildScene(True)
    simBuilder.setLasOutput(True)
    if args.zip_output_flag:
        simBuilder.setZipOutput(True)
        ext = "laz"
    else:
        ext = "las"
    simBuilder.setFixedGpsTimeStart(fixed_gps_time)

    sim = simBuilder.build()
    sim.start()
    output = sim.join()
    interval_output_folders.append(Path(output.outpath).parent)


# Merge legs of interval and write them to separate interval pcs.
merged_interval_paths = []
for i, interval_dir in enumerate(interval_output_folders):
    paths = list(Path(interval_dir).glob(f"*.{ext}"))
    outpath = Path(merged_intervals_dir) / f"merged_interval_{i+1}.laz"
    tds.laz_merge(paths, outpath)
    merged_interval_paths.append(outpath)

# Filter merged interval pcs to points inside the interval time.
print("\nMerging intervals\n")
tds.filter_and_write(merged_interval_paths, 
                     merged_filtered_intervals_dir,
                     obj_ids,
                     obj_of_int,
                     interval)


# Merge filtered interval pcs to final pc.
filtered_clouds_path = []
for file in os.listdir(merged_filtered_intervals_dir):
    filtered_clouds_path.append(
        os.path.join(merged_filtered_intervals_dir, file)
        )
tds.laz_merge(filtered_clouds_path, final_pc)

if args.delete_flag:
    paths_to_delete = [sp_surveys_dir, bboxes_obj_dir, bboxes_pc_dir, interval_surveys_dir, merged_intervals_dir, merged_bboxes_pc, split_scene_dir]
    paths_to_delete += objs_outfiles + interval_output_folders
    tds.delete_files(paths_to_delete)
