### This Script holds functions for the time dependent split in HELIOS++


import xml.etree.ElementTree as ET
from pathlib import Path
import laspy
import numpy as np
import matplotlib.pyplot as plt


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
    :param filename: File name.   <- To be removed?

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

    with laspy.open(filepaths[0], "r") as lf_0:
        las = lf_0.read()
        las.write(outfile)
    with laspy.open(outfile, "a") as lf:
        # ToDo: Check if this is needed
        scales = lf.header.scales
        offsets = lf.header.offsets

        for file in filepaths[1:]:
            if file.endswith(".las") or file.endswith(".laz"):

                with laspy.open(file) as lf_a:
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
    global_min_t = np.min(gps_time)
    att["gps_time"] = gps_time - global_min_t
    norm_gps_time = att["gps_time"]
    min_t = 0
    max_t = np.max(norm_gps_time)
    upper_interval = min_t + interval
    unique_gps_times = np.unique(norm_gps_time)
    global_unique_hits = np.unique(att['hitObjectId'])
    print(global_unique_hits)
    object_ids = []

    while min_t <= max_t:
        # Find all unique hitObjectIds in the current interval
        mask = (norm_gps_time >= min_t) & (norm_gps_time < upper_interval)
        unique_ids = np.unique(att['hitObjectId'][mask])
        print(unique_ids)
        object_ids.append(unique_ids)

        min_t += interval
        upper_interval += interval

    # Plotting unique GPS times
    # plt.figure(figsize=(10, 6))
    # plt.scatter(unique_gps_times, [1] * len(unique_gps_times), alpha=0.5)
    # plt.title("Unique GPS Times Distribution")
    # plt.xlabel("Normalized GPS Time")
    # plt.yticks([])
    # plt.show()
    
    return object_ids


def gen_interval_scene(original_scene_file, output_dir, id_list, obj_of_int):
    """
    Function which creates interval scenes with the scene parts that are 
    present in the interval.

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
    :param obj_of_int: ... ToDo @Jonas
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
            print(pc_attributes_filtered)
            print(f"Filtered {len(pc_coords_filtered)} points from a total of {len(coords)} points")

            # Write filtered point cloud to file
            if len(pc_coords_filtered) > 0:
                write_las(pc_coords_filtered, f"{filtered_interval_dir}merged_filtered_interval_{i + 1}.laz",
                          attribute_dict=pc_attributes_filtered)

        # Update time range for next interval
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
        attribute = np.array(indata.points[las_field])  # transpose shape to (N,1) to fit coordinates array
        if np.sum(attribute)==0: # if field contains only 0, it is empty
            continue
        # add the attribute to the dictionary with the name (las_field) as key
        attributes[las_field] = attribute

    # return coordinates and attribute data
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
