### This Script holds functions for the time dependent split in HELIOS++


import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import shutil
import copy
from pathlib import Path
import laspy
import numpy as np
import pandas as pd
import math

def split_xml(file, output_dir):
    """
    Function to split a HELIOS++ scene into sub scenes, one for each part.

    :param file: Path to the original scene file.
    :param output_dir: Directory where the sub scenes will be saved.

    :return outfiles: List of paths to the sub scene files.
    """

    xml_start = """<?xml version="1.0" encoding="UTF-8"?>
<document>
    <scene id="scene" name="scene">
"""

    xml_end = """
    </scene>
</document>
"""
    # Parse the XML file
    tree = ET.parse(file)
    root = tree.getroot()
    scene = root.find("scene")
    parts = scene.findall("part")

    outfiles = []
    # Split the XML file into multiple files
    for i, part in enumerate(parts):
        part_fp = part.find(".//param[@key='filepath']").get("value")
        part_name = Path(part_fp).stem
        outfile = Path(output_dir) / f"{part_name}_scene_part_{i}.xml"
        with open(outfile, "w") as f:
            f.write(xml_start)
            f.write("        " + ET.tostring(part).decode("utf-8"))
            f.write(xml_end)
        outfiles.append(outfile)
    return outfiles


def write_survey(template_path, sub_scene_path, suffix="", subfolder=None):
    """
    Function which overwrites the original survey to add the scene path.

    :param template_path: Path to the original survey file.
    :param sub_scene_path: Path to the sub scene.

    :return output_filename: Path to the new survey file.

    """
    template = ET.parse(template_path)
    template_root = template.getroot()

    scene_path = template_root.find('.//survey[@scene]')
    scene_path.attrib["scene"] = str(sub_scene_path) + "#scene"

    if subfolder:
        outfolder = Path(template_path).parent / subfolder
    else:
        outfolder = Path(template_path).parent
    Path(outfolder).mkdir(parents=True, exist_ok=True)
    output_filename = f'{outfolder}/{Path(template_path).stem}_{suffix}.xml'
    template.write(output_filename, xml_declaration=True)

    return output_filename


def create_obj_box(min_coords, max_coords, filename, output_dir):
    """
    This functions takes the min and max coordinates of a scene part and 
    creates an obj of the bbox.

    :param min_coords: Minimum coordinates of the scene part.
    :param max_coords: Maximum coordinates of the scene part.
    :param filename: File name.   <- To be removed?
    :param output_dir: Directory where the bbox .objs will be saved.

    :return obj_outfile: Path to the bbox.
    """

    # Define min and max coordinates
    min_x, min_y, min_z = min_coords
    max_x, max_y, max_z = max_coords



    # Define Vertices
    vertices = [
        (min_x, min_y, min_z),
        (max_x, min_y, min_z),
        (max_x, max_y, min_z),
        (min_x, max_y, min_z),
        (min_x, min_y, max_z),
        (max_x, min_y, max_z),
        (max_x, max_y, max_z),
        (min_x, max_y, max_z)
    ]

    # Define faces using the Vertices
    faces = [
        (1, 2, 3, 4),
        (5, 6, 7, 8),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 4, 8, 7),
        (4, 1, 5, 8)
    ]

    # Write to .obj file
    obj_outfile = Path(output_dir) / filename
    with open(obj_outfile, 'w') as file:

        for vertex in vertices:
            file.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")

        for face in faces:
            file.write(f"f {' '.join(map(str, face))}\n")

    return obj_outfile


def write_bbox_scenes(output_dir, objs_outfiles):
    """
    Function which writes scenes containing a single bbox.

    :param output_dir: Directory where the bbox sub scenes are saved.
    :param objs_outfiles: List of paths to the bboxes.

    :return bbox_outfiles: List of paths to the created scenes.
    """

    bbox_outfiles = []

    for i, path in enumerate(objs_outfiles):
        xml = f"""<?xml version="1.0" encoding="UTF-8"?>
        <document>
            <scene id="scene" name="scene">
                <part id="{i+1}">
                    <filter type="objloader">
                        <param type="string" key="filepath" value="{path}" />
                    </filter> 
                        <filter type="scale"> 
                    <param type="double" key="scale" value="1" />
                    </filter>
                </part>
            </scene>
        </document>
        """
        outfile = Path(output_dir) / f"BBox_{i}_scene.xml"
        with open(outfile, "w") as file:
            file.write(xml)
            bbox_outfiles.append(outfile)

    return bbox_outfiles


def write_multiple_surveys(template_path,
                           scene_outfiles,
                           output_dir,
                           filename):
    """
    Function that creates multiple surveys with the bbox scenes.

    :param template_path: Path to the original survey file.
    :param scene_outfiles: List of paths to the bbox scene files.
    :param output_dir: Directory where the bbox sub surveys will be saved.
    :param filename: File name.

    :return survey_outfiles: List of paths to the surveys.
    """

    survey_outfiles = []

    for i, paths in enumerate(scene_outfiles):
        template = ET.parse(template_path)
        template_root = template.getroot()

        scene_path = template_root.find('.//survey[@scene]')
        scene_path.attrib["scene"] = str(paths) + "#scene"

        survey_outfile = Path(output_dir) / f"{filename}_{i}.xml"

        template.write(survey_outfile, xml_declaration=True)
        survey_outfiles.append(survey_outfile)

    return survey_outfiles


def laz_merge(filepaths, outfile):
    """
    Function which merges multiple las/laz files into a single file.

    :param filepaths: Path list of point clouds to be merged.
    :param outfile: Path where the merged file will be saved.
    """
    for i, f in enumerate(filepaths):
        with laspy.open(f, "r") as lf_0:
            las = lf_0.read()
            if len(las.points) == 0:
                print("Empty file, skipping...")
                continue
            else:
                print(f"Leg {i} not empty. Merging...")
                las.write(outfile)
                break
    with laspy.open(outfile, "a") as lf:
        scales = lf.header.scales
        offsets = lf.header.offsets

        for file in filepaths[i+1:]:
            if Path(file).suffix == ".laz" or Path(file).suffix == ".las":

                with laspy.open(file, "r") as lf_a:
                    lf_aa = lf_a.read()
                    if len(lf_aa.points) > 0:
                        lf_aa.X = (lf_aa.x - offsets[0]) / scales[0]
                        lf_aa.Y = (lf_aa.y - offsets[1]) / scales[1]
                        lf_aa.Z = (lf_aa.z - offsets[2]) / scales[2]
                        lf.append_points(lf_aa.points)
    
    return outfile


def objs_in_interval(infile, interval=9.5):
    """
    Function which creates a list of all scene parts within a 
    user defined interval.

    :param infile: Path to the merged bbox las/laz file.
    :param interval: Interval in seconds.

    :return object_ids: List of arrays which store the hitObjectId
    for each interval.
    """

    coords, att = read_las(infile)
    gps_time = att["gps_time"]
    global_min_t = 590000 + 7590  # Investigate the reason for this offset
    att["gps_time"] = gps_time - global_min_t
    norm_gps_time = att["gps_time"]
    min_t = 0
    max_t = np.max(norm_gps_time)
    upper_interval = min_t + interval
    unique_gps_times = np.unique(norm_gps_time)
    global_unique_hits = np.unique(att['hitObjectId'])
    print(f"Objects with the following IDs where scanned:{global_unique_hits}")
    object_ids = []
    i = 1
    while min_t <= max_t:
        # Find all unique hitObjectIds in the current interval
        mask = (norm_gps_time >= min_t) & (norm_gps_time < upper_interval)
        unique_ids = np.unique(att['hitObjectId'][mask])
        print(f"Object IDs in Interval {i}:{unique_ids}")
        object_ids.append(unique_ids)

        min_t += interval
        upper_interval += interval
        i += 1

    return object_ids


def gen_interval_scene(original_scene_file, output_dir, id_list, obj_of_int):
    """
    Function which creates interval scenes with the scene parts that are 
    present in the interval. Checks if a Interval with the same scene parts was already created to avoid redundant sims.

    :param original_scene_file: Path to the original scene file.
    :param output_dir: Directory where the interval scene will be saved.
    :param id_list: List of arrays which store the hitObjectId for
                    each interval.
    :param obj_of_int: List of objectIDs to be scanned.

    :return outfiles: List of paths to the interval scene files.
    """
    processed_scenes = set()
    outfiles = []
    i = 0
    for part_ids in id_list:
        if len(part_ids) < 1 or (obj_of_int and not 
                                 any(obj in part_ids for obj in obj_of_int)
                                 ):
            i += 1
        else:
            part_ids_set = frozenset(part_ids)

            if part_ids_set in processed_scenes:
                print(f"Scene with parts {part_ids} already created. Skipping...")
                i += 1
                continue  
                
            processed_scenes.add(part_ids_set)

            xml_start = """<?xml version="1.0" encoding="UTF-8"?>
<document>
    <scene id="scene" name="scene">
"""

            xml_end = """
    </scene>
</document>
"""
            # Parse the XML file
            tree = ET.parse(original_scene_file)
            root = tree.getroot()
            scene = root.find("scene")
            parts = scene.findall("part")


            outfile = Path(output_dir) / f"Interval_{i}_scene.xml"

            combined_parts = ""

            for id in part_ids:
                combined_parts += ET.tostring(parts[id-1], encoding='unicode')

            # Write the new XML content to the output file
            with open(outfile, "w") as f:
                f.write(xml_start)
                f.write(combined_parts)
                f.write(xml_end)
            i += 1
            outfiles.append(outfile)

    return outfiles


def filter_and_write(interval_paths, filtered_interval_dir, id_list, obj_of_int, interval=5):
    """
    Function that filters the interval point clouds so that only points inside the intervals' GPS time remain.
    Writes these filtered point clouds to new files.

    :param interval_paths: List of paths to the merged interval point clouds.
    :param filtered_interval_dir: Directory where the filtered interval point clouds will be saved.
    :param id_list: List of arrays which store the hitObjectId for each interval.
    :param obj_of_int: List of Objects that the User is interested in. Intervals without these objects will be ignored.
    :param interval: Interval in seconds.
    """

    i_start = 0
    i_end = interval

    # Track which scene has already been processed, map part IDs to file index
    part_ids_to_file_index = {}
    file_index = 0

    for i, part_ids in enumerate(id_list):
        if len(part_ids) < 1 or (obj_of_int and not any(obj in part_ids for obj in obj_of_int)):
            print(f"Interval {i + 1} is empty!")
        else:
            # Convert part_ids to frozenset for consistent identification of the same scene parts
            part_ids_set = frozenset(part_ids)

            # Check if the part IDs were processed; reuse the file if so
            if part_ids_set not in part_ids_to_file_index:
                part_ids_to_file_index[part_ids_set] = file_index
                file_index += 1

            filename = interval_paths[part_ids_to_file_index[part_ids_set]]
            print(f"interval {i + 1} ranging from {i_start} to {i_end}")

            coords, attributes = read_las(filename)
            gps_time = attributes["gps_time"]
            global_min_t = 590000 + 7590  # Investigate the reason for this offset
            attributes["gps_time"] = gps_time - global_min_t

            # Filter point cloud by GPS time for the current interval
            pc_coords_filtered = coords[(attributes['gps_time'] >= i_start) & (attributes['gps_time'] < i_end)]
            pc_attributes_filtered = {}
            for k, v in attributes.items():
                pc_attributes_filtered[k] = v[(attributes['gps_time'] >= i_start) & (attributes['gps_time'] < i_end)]
            print(f"Filtered {len(pc_coords_filtered)} points from a total of {len(coords)} points")

            # Write filtered point cloud to file
            if len(pc_coords_filtered) > 0:
                write_las(pc_coords_filtered, f"{filtered_interval_dir}/merged_filtered_interval_{i + 1}.laz",
                          attribute_dict=pc_attributes_filtered)
                print(f"Wrote interval {i + 1}")

        i_start += interval
        i_end += interval

    return 0


def read_las(infile):
    """
    Function to read coordinates and attribute information of point cloud data from las/laz file.

    :param infile: Path to pc

    :return: coords: Array of point coordinates of shape (N,3)
    :return: attributes: Dictionary of point attributes
    """

    # read the file using the laspy read function
    indata = laspy.read(infile)

    # get the coordinates (XYZ) and stack them in a 3D array
    coords = np.vstack((indata.x, indata.y, indata.z)).transpose()

    # get all attribute names in the las file as list
    las_fields = list(indata.points.point_format.dimension_names)

    # create a dictionary to store attributes
    attributes = {}

    # loop over all available fields in the las point cloud data
    for las_field in las_fields[3:]:  # skip the first three fields, which contain coordinate information (X,Y,Z)
        attribute = np.array(indata.points[las_field])
        attributes[las_field] = attribute

    return coords, attributes


def write_las(outpoints, outfilepath, attribute_dict={}):
    """             
    Function which writes a las/laz file.

    :param outpoints: 3D array of points to be written to output file
    :param outfilepath: Path to the output file
    :param attribute_dict: dictionary of attributes
    """
    # create a header for new las file
    hdr = laspy.LasHeader(version="1.4", point_format=6)

    # create the las data
    las = laspy.LasData(hdr)

    # write coordinates into las data
    las.x = outpoints[:, 0]
    las.y = outpoints[:, 1]
    las.z = outpoints[:, 2]

    attribute_dict.pop("ExtraBytes", None)

    for key, vals in attribute_dict.items():
        try:
            las[key] = vals
        except Exception as e:
            print(e)
            las.add_extra_dim(laspy.ExtraBytesParams(
                name=key,
                type=type(vals[0])
            ))
            las[key] = vals

    # write las file
    las.write(outfilepath)

    return 0


def delete_files(paths):
    """
    Function which deletes files and directories provided as a list.

    :param paths: List of paths to be deleted.
    """
    for path in paths:
        if os.path.isfile(path):
            os.remove(path)
        elif os.path.isdir(path):
            shutil.rmtree(path)

    return 0




def process_interval(interval_num, duration, min_int_t, ids, df, max_prim, i_nr, interval_data, merged_bboxes_las):
    """
    This recursive functions splits each interval until a user defined primitive threshold is met or a min interval
    duration is reached.
    This function is needed when doing the top-down split without a set interval.

    :param interval_num: Interval number, should start at 1.
    :param duration: Duration of the interval, initial value is the sim runtime, gets updated after each recursion.
    :param min_int_t: Minimum duration for each interval before recursive split is stopped.
    :param ids: IDs of the scene parts for each interval.
    :param df: DataFrame containing the primitive count for each scene.
    :param max_prim: Maximum allowed primitive count threshold.
    :param i_nr: Interval counter.
    :param interval_data: DataFrame to store interval data.
    :param merged_bboxes_las: Merged bounding boxes file.

    :return interval_data: Updated interval DataFrame.
    :return i_nr: Updated interval counter.
    """

    # Calculate the sum of primitives for this interval
    primitives_sum = df[df['Scene Number'].isin(ids)]['Primitive Count'].sum()

    # Check if this interval meets the threshold
    if primitives_sum <= max_prim:
        new_data = pd.DataFrame([{
            'Interval': i_nr,
            'Object IDs': ids,
            'Primitive Count': primitives_sum,
            'Interval Duration': duration
        }])
        interval_data = pd.concat([interval_data, new_data], ignore_index=True)
        i_nr += 1
    else:
        # Refine the interval by halving its duration
        refined_duration = duration / 2
        if refined_duration >= min_int_t:  # Minimum interval time
            # Split the interval into two sub-intervals
            sub_ids = [interval_num * 2 - 1, interval_num * 2]

            # Object IDs for each interval using the new interval length
            obj_ids = objs_in_interval(merged_bboxes_las, refined_duration)

            for sub_id in sub_ids:
                # Recursively process each new interval
                sub_interval_ids = obj_ids[sub_id - 1] if sub_id - 1 < len(obj_ids) else []
                interval_data, i_nr = process_interval(sub_id, refined_duration, min_int_t, sub_interval_ids, df, max_prim, i_nr, interval_data, merged_bboxes_las)
        else:
            # If the new interval time is lower than min_int_t, the interval will be added with despite being too big.
            print("Interval duration is too short, interval with Primitive amount above threshold added!")
            new_data = pd.DataFrame([{
                'Interval': i_nr,
                'Object IDs': ids,
                'Primitive Count': primitives_sum,
                'Interval Duration': duration  # duration of prev sub interval
            }])
            interval_data = pd.concat([interval_data, new_data], ignore_index=True)
            i_nr += 1

    return interval_data, i_nr




def filter_and_write_dynamic(interval_paths, filtered_interval_dir, id_list, obj_of_int, interval):
    """
    Function that filters the interval point clouds so that only points inside the intervals' GPS time remain.
    Writes these filtered point clouds to new files.
    This function is needed when doing the top-down split without a set interval.

    :param interval_paths: List of paths to the merged interval point clouds.
    :param filtered_interval_dir: Directory where the filtered interval point clouds will be saved.
    :param id_list: List of arrays which store the hitObjectId for each interval.
    :param interval: List of interval durations for each interval.
    """

    i = 0
    i_start = 0
    i_end = interval[0]
    part_ids_to_file_index = {}
    file_index = 0

    for il in interval:
        if len(id_list[i]) < 1 or (obj_of_int and not any(obj in id_list[i] for obj in obj_of_int)):
            print(f"Interval {i + 1} is empty!")
        else:
            # Convert part_ids to frozenset for consistent identification of the same scene parts
            part_ids_set = frozenset(id_list[i])

            # Check if the part IDs were processed; reuse the file if so
            if part_ids_set not in part_ids_to_file_index:
                part_ids_to_file_index[part_ids_set] = file_index
                file_index += 1

            # Get the filename for the interval's point cloud
            filename = interval_paths[part_ids_to_file_index[part_ids_set]]
            print(f"interval {i + 1} ranging from {i_start} to {i_end}")

            # Read point cloud data
            coords, attributes = read_las(filename)
            gps_time = attributes["gps_time"]
            global_min_t = 590000 + 7590  # Investigate the reason for this offset
            attributes["gps_time"] = gps_time - global_min_t

            # Filter point cloud by GPS time for the current interval
            pc_coords_filtered = coords[(attributes['gps_time'] >= i_start) & (attributes['gps_time'] < i_end)]
            pc_attributes_filtered = {}
            for k, v in attributes.items():
                pc_attributes_filtered[k] = v[(attributes['gps_time'] >= i_start) & (attributes['gps_time'] < i_end)]

            print(f"Filtered {len(pc_coords_filtered)} points from a total of {len(coords)} points")

            # Write filtered point cloud to file
            if len(pc_coords_filtered) > 0:
                write_las(pc_coords_filtered, f"{filtered_interval_dir}/merged_filtered_interval_{i + 1}.laz",
                          attribute_dict=pc_attributes_filtered)

        i_start += il
        if i + 1 < len(interval):
            i_end += interval[i + 1]
        else:
            i_end += il
        i += 1

    return 0


def check_tile_size(tile_size, voxel_size):
    """
    Function which adjusts tile size to be a multiple of voxel size.

    :param tile_size: Current tile size.
    :param voxel_size: Size of the voxel.

    :return: Adjusted tile size.
    """
    if tile_size % voxel_size == 0:
        print("fine")
    else:
        print("not fine")
        tile_size = math.ceil(tile_size / voxel_size) * voxel_size
        print("new tile size: ", tile_size)

    return tile_size


def read_xyz(file_path):
    """
    Reads an XYZ file and returns an array of points (x, y, z).

    :param file_path: Path to the XYZ file.

    :return: Array of points (x, y, z).
    """

    return np.loadtxt(file_path, delimiter=" ", usecols=(0, 1, 2))


def write_xyz(file_path, points):
    """
    Writes an array of points (x, y, z) to an XYZ file.
    :param file_path: Path to the XYZ file.
    :param points: Array of points (x, y, z).
    """
    np.savetxt(file_path, points, fmt="%.6f %.6f %.6f")

    return 0


def xyz_create_3DTiles(xyzfile, outDir, tilesize, scene_part_number, tilename="", buffer=0):
    """
    Tiles an XYZ file by dividing it into smaller 3D tiles based on the provided tile size.

    :param xyzfile: Path to the input XYZ file.
    :param outDir: Directory to save the tiled files.
    :param tilesize: The size of each tile in meters.
    :param tilename: Optional prefix for each tile's file name.
    :param buffer: Optional buffer around each tile boundary.

    :return: List of file paths to the generated tiles.
    """

    out_tiles = []
    # Read points from the XYZ file
    points = read_xyz(xyzfile)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # Calculate boundaries
    x_min, x_max = np.floor(x.min()), np.ceil(x.max())
    y_min, y_max = np.floor(y.min()), np.ceil(y.max())
    z_min, z_max = np.floor(z.min()), np.ceil(z.max())

    # Calculate the number of tiles in each dimension
    xTiles = int(np.ceil((x_max - x_min) / tilesize))
    yTiles = int(np.ceil((y_max - y_min) / tilesize))
    zTiles = int(np.ceil((z_max - z_min) / tilesize))

    # Define tile boundaries
    xVerts = np.arange(x_min, x_min + xTiles * tilesize + 1, tilesize)
    yVerts = np.arange(y_min, y_min + yTiles * tilesize + 1, tilesize)
    zVerts = np.arange(z_min, z_min + zTiles * tilesize + 1, tilesize)

    # Generate tiles and save points for each tile
    print("Generating tiles...")
    i = 1
    for xTile in range(xTiles):
        tileMinX = round(xVerts[xTile], 3)
        tileMaxX = round(xVerts[xTile + 1], 3)
        for yTile in range(yTiles):
            tileMinY = round(yVerts[yTile], 3)
            tileMaxY = round(yVerts[yTile + 1], 3)
            for zTile in range(zTiles):
                tileMinZ = round(zVerts[zTile], 3)
                tileMaxZ = round(zVerts[zTile + 1], 3)

                # Define the output tile filename
                outTile = os.path.join(
                    outDir, f"{tilename}_sp_{scene_part_number}_tile_{i}.xyz"
                )
                i += 1

                # Create a mask to filter points within the tile boundaries + buffer
                mask = (
                        (x >= (tileMinX - buffer)) & (x < (tileMaxX + buffer)) &
                        (y >= (tileMinY - buffer)) & (y < (tileMaxY + buffer)) &
                        (z >= (tileMinZ - buffer)) & (z < (tileMaxZ + buffer))
                )

                # Filter points for this tile and save if there are points
                tile_points = points[mask]
                if tile_points.shape[0] > 0:
                    write_xyz(outTile, tile_points)
                    print("Created tile:", outTile)
                    out_tiles.append(outTile)
    return out_tiles


def process_scene_and_split_xyz(scene_file, output_dir, tilesize, buffer=0):
    """
    Processes the scene XML, splits xyz files into tiles, and updates the XML accordingly.

    :param scene_file: Path to the input scene XML file.
    :param output_dir: Directory to save tiled xyz files and the modified XML.
    :param tilesize: The size of each tile in meters.
    :param buffer: Optional buffer around tile boundaries.

    :return: output_file: Path to the new scene file
    :return: obj_df: Needed to update obj_of_int
    """
    # Parse the XML file
    tree = ET.parse(scene_file)
    root = tree.getroot()
    scene = root.find(".//scene")

    original_parts = scene.findall(".//part")
    new_parts = []
    scene_part_number = 1
    obj_df = pd.DataFrame(columns=['Part Number', 'New Parts'])
    for part in original_parts:
        filters = part.findall(".//filter")
        is_xyzloader = any(f.get("type") == "xyzloader" for f in filters)

        if is_xyzloader:
            # Process XYZ loader
            filepath = part.find(".//param[@key='filepath']").get("value")
            voxel_size = float(part.find(".//param[@key='voxelSize']").get("value"))

            additional_filters = [f for f in filters if f.get("type") != "xyzloader"]
            new_tilesize = check_tile_size(tilesize, voxel_size)

            # Split the XYZ file into smaller tiles
            tiles = xyz_create_3DTiles(
                xyzfile=filepath,
                outDir=output_dir,
                tilesize=new_tilesize,
                scene_part_number=scene_part_number,
                buffer=buffer
            )
            num_new_parts = len(tiles)

            obj_df = pd.concat([
                obj_df,
                pd.DataFrame([{
                    'Part Number': scene_part_number,
                    'New Parts': num_new_parts
                }])
            ])
            scene_part_number += 1

            # Create new parts for each tile
            for tile in tiles:
                new_part = ET.Element("part")
                filter_element = ET.SubElement(new_part, "filter", type="xyzloader")

                # Copy all param elements from the original xyzloader filter
                original_filter = next(f for f in filters if f.get("type") == "xyzloader")
                for param in original_filter.findall(".//param"):
                    key = param.get("key")
                    value = tile if key == "filepath" else param.get("value")
                    ET.SubElement(filter_element, "param", type=param.get("type"), key=key, value=value)

                for additional_filter in additional_filters:
                    new_filter = copy.deepcopy(additional_filter)
                    new_part.append(new_filter)

                new_parts.append(new_part)

        else:
            obj_df = pd.concat([
                obj_df,
                pd.DataFrame([{
                    'Part Number': scene_part_number,
                    'New Parts': 1
                }])
            ])

            scene_part_number += 1
            new_parts.append(part)
    # Clear the scene and add the new parts
    for part in original_parts:
        scene.remove(part)
    scene.extend(new_parts)

    # Reformat XML
    raw_xml = ET.tostring(root, encoding="utf-8")
    parsed_xml = minidom.parseString(raw_xml)
    pretty_xml = parsed_xml.toprettyxml(indent="    ")

    output_file = os.path.join(output_dir, "modified_scene.xml")
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(pretty_xml)

    print(f"Formatted and modified XML saved to {output_file}")

    return output_file, obj_df


def update_obj_of_int(obj_df, obj_of_int):
    """
    Updates the obj_of_int list to include the new sub tiles.

    :param obj_df: DataFrame containing the mapping of original parts to their new parts count.
    :param obj_of_int: List of part numbers representing the original objects of interest (zero-indexed).

    :return updated_obj_of_int: List of all new parts corresponding to the original obj_of_int.
    """
    updated_obj_of_int = []
    cumulative_index = 0

    for _, row in obj_df.iterrows():
        part_number = row['Part Number'] - 1
        new_parts_count = int(row['New Parts'])

        # Check if the current part is in the obj_of_int list
        if part_number in obj_of_int:
            # Add all new parts for this object of interest
            updated_obj_of_int.extend(range(cumulative_index, cumulative_index + new_parts_count))

        cumulative_index += new_parts_count

    return updated_obj_of_int

