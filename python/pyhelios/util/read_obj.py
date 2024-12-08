
def read_obj(path, logging=False):
    import open3d as o3d
    import os
    import tempfile

    # Read file into memory
    with open(path, 'r') as f:
        lines = f.readlines()

    # Count number of faces with more than 3 vertices and overall maximum number of vertices
    count = 0
    count_four_verts = 0
    max_vert_num = 0
    four_verts = False

    for line in lines:
        vals = line.split(' ')
        if vals[0] == 'f' and len(vals) > 4:
            count += 1
            max_vert_num = len(vals) - 1 if len(vals) - 1 > max_vert_num else max_vert_num
        if vals[0] == 'f' and len(vals) == 5:
            four_verts = True
            count_four_verts += 1

    if logging:
        print('Total number of faces in OBJ with more than 3 vertices: {}'.format(count))
        print('Total number of faces in OBJ with 4 vertices: {}'.format(count_four_verts))
        print('Maximum number of vertices per face: {}'.format(max_vert_num))

    if max_vert_num > 4:
        print('[HELIOS.PY WARNING] {} contains at least one face with more than 4 vertices. \
        These faces will not be visualised and are not considered by HELIOS++'.format(path))

    # If a face with four vertices exists, create copy of obj and write modified faces to new obj
    if four_verts:
        print('{} contains at least one face with 4 vertices. \
        Converting obj to triangles for use in o3d visualisation....'.format(path))
        with tempfile.NamedTemporaryFile(mode='w', suffix='.obj', delete=False) as f:
            tempname = f.name
            for line in lines:
                linesplit = line.split(' ')
                # If line is face and has four verts, write modified lines to new obj
                if linesplit[0] == 'f' and len(linesplit) == 5:
                    f.write('f {} {} {}\n'.format(linesplit[1], linesplit[2], linesplit[3]))
                    f.write('f {} {} {}'.format(linesplit[1], linesplit[3], linesplit[4]))
                    # for i in range(len(linesplit)-3):
                    # f.write('f {} {} {}\n'.format(linesplit[1], linesplit[2+i], linesplit[3+i]))
                else:
                    # If not face and four verts simply copy line
                    f.write(line)

        # Create open3d object.
        geometry = o3d.io.read_triangle_mesh(tempname)
        geometry.compute_vertex_normals()

        os.remove(tempname)

    else:
        # Create open3d object.
        geometry = o3d.io.read_triangle_mesh(path)
        geometry.compute_vertex_normals()

    return geometry
