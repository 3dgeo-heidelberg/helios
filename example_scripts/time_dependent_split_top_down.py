import time_dependent_split_functions as tds
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
import os
import tds_argparser
import pyhelios

# Import arguments for parameters
args = tds_argparser.args

original_survey = args.survey_file
obj_of_int = args.obj_of_int
prim_threshold = args.prim_threshold
min_int_t = args.min_int_t
assets = [os.getcwd()]
if args.assets_path:
    assets.append(args.assets_path)

if args.output_path:
    outdir = args.output_path
else:
    outdir = '/output'
fixed_gps_time = "2024-07-07 00:00:00"

# Set Directories and output paths
survey_name = ET.parse(original_survey).find('survey').attrib['name']
original_scene = ET.parse(original_survey).find('survey').attrib['scene'].split('#')[0]
original_scene_fname = Path(original_scene).stem
scene_dir = Path(original_scene).parent
scene_name = ET.parse(original_survey).find('survey').attrib['scene'].split('#')[1]
split_scene_dir = scene_dir / f"{original_scene_fname}_split"
sp_surveys_dir2 = f"{Path(original_survey).stem}_per_sp"
sp_surveys_dir = Path(outdir) / survey_name / 'scene_part_sub_scenes'
bboxes_obj_dir = Path(scene_dir) / f'{original_scene_fname}_bboxes'
bboxes_pc_dir = Path(outdir) / survey_name / 'bbox_surveys'
merged_bboxes_pc = Path(scene_dir) / f'{original_scene_fname}_bboxes.laz'
interval_surveys_dir = Path(outdir) / survey_name / 'interval_surveys'
merged_intervals_dir = Path(outdir) / survey_name / 'merged_intervals'
merged_filtered_intervals_dir = Path(outdir) / survey_name / 'merged_filtered_intervals'
final_pc = Path(outdir) / survey_name / f'{survey_name}_final.laz'
Path(bboxes_obj_dir).mkdir(parents=True, exist_ok=True)
Path(sp_surveys_dir).mkdir(parents=True, exist_ok=True)
Path(split_scene_dir).mkdir(parents=True, exist_ok=True)
Path(merged_intervals_dir).mkdir(parents=True, exist_ok=True)
Path(merged_filtered_intervals_dir).mkdir(parents=True, exist_ok=True)

# Create sub scenes with one part each out of the original scene file.
split_scene_files = tds.split_xml(original_scene, split_scene_dir)

# Initialize arrays that store min and max coordinates for each scene part.
mins = np.zeros(shape=(len(split_scene_files), 3))
maxs = np.zeros(shape=(len(split_scene_files), 3))

# Initialize data frame that stores the primitive numbers for each scene part.
df = pd.DataFrame(columns=['Scene Number', 'Primitive Count'])

# Run simulation for each scene part to get bounding box information.
print("\nGetting bounding boxes\n")
for i, paths in enumerate(split_scene_files):
    # Overwrite original survey for each scene part scene.
    survey = tds.write_survey(original_survey, paths, suffix=f"sub_{i}",
                              subfolder=sp_surveys_dir)

    pyhelios.loggingSilent()
    # Build simulation parameters.
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
    num_prim = scene.getNumPrimitives()
    df = pd.concat([df, pd.DataFrame([{'Scene Number': i + 1, 'Primitive Count': num_prim}])])

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

# Calculate the sum of primitives and compute the maximum percentage of prims per interval.
prim_sum = df['Primitive Count'].sum()
max_prim = prim_sum * prim_threshold

# Create bounding box .objs out of the min and  max coordinates of each scene part.
objs_outfiles = []
for i in range(len(mins)):
    obj_outfile = tds.create_obj_box(mins[i], maxs[i], f"BBox_{i+1}.obj", bboxes_obj_dir)
    objs_outfiles.append(obj_outfile)


# Writes scenes and surveys with one bbox .obj each.
Bbox_scene_outfiles = tds.write_bbox_scenes(bboxes_obj_dir, objs_outfiles)
survey_outfiles = tds.write_multiple_surveys(original_survey, Bbox_scene_outfiles, bboxes_obj_dir, f"BBox_survey")

# Run simulation for each bbox survey.
bbox_output_files = []
print("\nSimulating bounding box objs\n")
for path in survey_outfiles:
    pyhelios.loggingSilent()
    # Build simulation parameters.

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

    sim = simBuilder.build()
    sim.start()
    output = sim.join()
    for j in range(output.outpaths.length()):
        bbox_output_files.append(output.outpaths.get(j))

# merge bounding boxes.
tds.laz_merge(bbox_output_files, merged_bboxes_pc)

#get initial_interval_duration from max gps_time.
bbpc_coords, bbpc_attr = tds.read_las(merged_bboxes_pc)
gps_time = bbpc_attr["gps_time"]
global_min_t = 590000 + 7590
bbpc_attr["gps_time"] = gps_time - global_min_t
initial_interval_duration = np.max(bbpc_attr["gps_time"])

# Assess scene part presence for each interval.
obj_ids = tds.objs_in_interval(merged_bboxes_pc, initial_interval_duration)

# Create data frame that stores info about each interval and run func to split each interval to desired primitive size.
interval_data = pd.DataFrame(columns=['Interval', 'Object IDs', 'Primitive Count', 'Interval Duration'])
i_nr = 1
for i, ids in enumerate(obj_ids):
    interval_data, i_nr = tds.process_interval(i + 1, initial_interval_duration, min_int_t, ids, df, max_prim, i_nr, interval_data, merged_bboxes_pc)

# Output the final intervals.
print("Final Interval Data:")
print(interval_data)

# Create scenes and surveys for each interval.
id_list = interval_data["Object IDs"]
interval_scene_outfiles = tds.gen_interval_scene(original_scene, scene_dir, id_list, obj_of_int)
interval_surveys = tds.write_multiple_surveys(original_survey, interval_scene_outfiles, scene_dir, f"Interval_survey")

# Run simulation for each interval survey.
interval_output_folders = []
print("\nSimulating interval surveys\n")
for path in interval_surveys:
    pyhelios.loggingSilent()
    pyhelios.setDefaultRandomnessGeneratorSeed("123")

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
    outpath = Path(merged_intervals_dir) / f"merged_interval_{i+1}.{ext}"
    tds.laz_merge(paths, outpath)
    merged_interval_paths.append(outpath)

# Filter merged interval pcs to points inside the interval time.
print("\nMerging intervals\n")
interval_duration = interval_data["Interval Duration"]
tds.filter_and_write_dynamic(merged_interval_paths, merged_filtered_intervals_dir, id_list, obj_of_int, interval_duration)

# Merge filtered interval pcs to final pc.
filtered_clouds_path = []
for file in os.listdir(merged_filtered_intervals_dir):
    filtered_clouds_path.append(os.path.join(merged_filtered_intervals_dir,file))
tds.laz_merge(filtered_clouds_path, final_pc)

coords, attr = tds.read_las(final_pc)
print(f"Total amount of points: {len(coords)} ")
print("done")

# delete auxiliary files if needed.
if args.delete_flag:
    paths_to_delete = [sp_surveys_dir, bboxes_obj_dir, bboxes_pc_dir, interval_surveys_dir, merged_intervals_dir, merged_bboxes_pc, split_scene_dir]
    paths_to_delete += objs_outfiles + interval_output_folders
    tds.delete_files(paths_to_delete)

