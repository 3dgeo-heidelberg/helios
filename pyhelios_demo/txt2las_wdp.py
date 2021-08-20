#!/usr/bin/python
# txt2las_wdp
# copyright 2021 3DGeo Heidelberg
# Script to transform a HELIOS++ xyz point cloud and fullwave file into las1.4 and external WDP file
# Result can be viewed e.g. with CloudCompare (open file with "open"-> select "las 1.3 and las 1.4" as file type)

import struct
import laspy
import numpy as np
from collections import OrderedDict

wfs = OrderedDict()
with open(r"..\output\Survey Playback\als_hd_demo\points\leg000_fullwave.txt", 'r') as inf:
    for line in inf.readlines():
        data = [float(x) for x in line.split(" ")]
        wf_idx = int(data[0])
        tmin = data[7]
        tmax = data[8]
        fwf = np.abs(np.array(data[10:]))
        fwf = fwf/np.max(fwf) * 255.
        fwf = fwf.astype(int)
        dx, dy, dz = data[4], data[5], data[6]
        wfs[wf_idx] = [tmin, tmax, dx, dy, dz, fwf]

max_len = max([len(x[-1][-1]) for x in wfs.items()])
wfs_lut = {idx: val for idx, val in enumerate(np.unique(wfs.keys())[0])}
wfs_lut_rev = {val: idx for idx, val in wfs_lut.items()}
pts = np.loadtxt(r"..\output\Survey Playback\als_hd_demo\points\leg000_points.xyz", delimiter=" ")

las = laspy.create(file_version="1.4", point_format=4)
xyz = pts[:, :3]
las.header.offsets = np.min(xyz, axis=0)
las.header.scales = [0.001, 0.001, 0.001]
las.x = xyz[:, 0]
las.y = xyz[:, 1]
las.z = xyz[:, 2]
las.intensity = pts[:, 3]
las.wavepacket_index=np.ones((xyz.shape[0]))
las.wavepacket_offset = 60 + np.array([wfs_lut_rev[idx] for idx in pts[:, 7]]) * (max_len)
las.wavepacket_size = [max_len * 8] * xyz.shape[0]
las.return_point_wave_location = [0] * xyz.shape[0]
las.x_t = [wfs[idx][2] for idx in pts[:, 7]]
las.y_t = [wfs[idx][3] for idx in pts[:, 7]]
las.z_t = [wfs[idx][4] for idx in pts[:, 7]]

VLR_data = struct.pack("<BBLLdd",
                        8,  # bits per sample
                        0,  # compression type
                        max_len,  # number of samples
                        10,  # temporal sample spacing (ps)
                        1,  # digitizer gain
                        0,  # digitizer offset
                       )
new_vlr = laspy.VLR(user_id="LASF_Spec", record_id=100,
              record_data=VLR_data)
# Append our new vlr to the current list.
las.vlrs.append(new_vlr)

las.write("test.las")

wf_data = bytes()
wf_data = b''.join([wf_data, struct.pack("<H", 0)])  #reserved
wf_data = b''.join([wf_data, struct.pack("<s", b"LASF_Spec       ")])  #user id
wf_data = b''.join([wf_data, struct.pack("<H", 65535)])  #record id
wf_data = b''.join([wf_data, struct.pack("<Q", max_len*len(wfs))])  #record length after header
wf_data = b''.join([wf_data, struct.pack("<s", b" "*32)])  #descripton

with open("test.wdp", 'wb') as f:
    f.write(wf_data)

    for wf in wfs.items():
        byt = np.zeros((max_len,), dtype=int)
        byt[:len(wf[1][-1])] = [int(d) for d in wf[1][-1]]
        f.write(struct.pack("<%dB" % max_len, *byt))


with open("test.las", 'r+b') as f:
    f.seek(6)
    f.write(struct.pack("<H", int('0000000000000100', 2)))