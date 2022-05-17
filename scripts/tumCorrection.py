#! /usr/bin/env python2
# move initial position to zero
from __future__ import print_function

import csv

import numpy as np
import tf.transformations


def line2TransformMatrix(line):
    pos = line[1:4]
    orient = line[4:]
    trans = tf.transformations.quaternion_matrix(orient)
    trans[0:3, 3] = np.array(pos)
    return trans


def transformMatrix2Line(trans_mat):
    posit_out = trans_mat[0:3, 3]
    quat_out = tf.transformations.quaternion_from_matrix(trans_mat)
    return [
        posit_out[0],
        posit_out[1],
        posit_out[2],
        quat_out[0],
        quat_out[1],
        quat_out[2],
        quat_out[3],
    ]


def correctTum(filename, filename_out, tf_matrix):
    data_out = []
    with open(filename, "r") as file:
        csvread = csv.reader(file)
        for idx, line in enumerate(csvread):
            line = [x for x in line[0].split(" ")]
            if idx == 0:
                data_out.append(line)
                continue
            TM = line2TransformMatrix(line)
            TM = tf_matrix.dot(TM)

            line_out = [line[0]] + transformMatrix2Line(TM)
            data_out.append(line_out)

    with open(filename_out, "w") as file:
        writer = csv.writer(file, delimiter=" ")
        writer.writerows(data_out)


matrices = {
    1: np.identity(4),
    2: np.array(
        [
            [-0.9649804, 0.2405966, 0.1045285, -9.28],
            [-0.2291086, -0.9670675, 0.1108583, 9.96],
            [0.1277582, 0.0830277, 0.9883239, 1.58],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
    3: np.array(
        [
            [-0.2838423, -0.9582356, 0.0348995, -7.305],
            [0.9564262, -0.2855284, -0.0610114, 10.902],
            [0.0684280, 0.0160612, 0.9975268, 14.11],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
    4: np.array(
        [
            [-0.3899601, -0.9186885, 0.0627905, 6.6],
            [0.9206989, -0.3878381, 0.0435333, 10.45],
            [-0.0156410, 0.0747874, 0.9970768, 14.82],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
    5: np.array(
        [
            [0.86431113, 0.50054155, 0.04923874, 8.6708615],
            [-0.50236076, 0.86390593, 0.03605417, 10.23562694],
            [-0.02449104, -0.05589767, 0.99813585, 4.35996567],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
}


def inv(mat):
    return np.linalg.inv(mat)


transforms = {
    1: matrices[1],
    2: matrices[2],
    3: matrices[3],
    4: matrices[4],
    5: matrices[5],
}


names = {
    1: "09_0_60.txt",
    2: "09_60_160.txt",
    3: "09_160_285.txt",
    4: "09_285_380.txt",
    5: "09_380_447.txt",
}


path = "/home/norlab/hilti_ws/src/hiltislamchallenge/results/online/exp09_cupola/odom/maxime/trajectories/bundled/"

for k in names.keys():
    filename = path + names[k]
    filename_out = path + "out/0{0}out.txt".format(str(k))
    correction = transforms[k]
    correctTum(filename, filename_out, correction)
    print("Processed", filename, "in", filename_out)
