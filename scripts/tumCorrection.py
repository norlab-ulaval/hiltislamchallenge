# move initial position to zero
import csv
import tf.transformations
import numpy as np


def line2TransformMatrix(line):
    pos = line[1:4]
    orient = line[4:]
    trans = tf.transformations.quaternion_matrix(orient)
    trans[0:3, 3] = np.array(pos)
    return trans


def transformMatrix2Line(trans_mat):
    posit_out = trans_mat[0:3, 3]
    quat_out = tf.transformations.quaternion_from_matrix(trans_mat)
    return [posit_out[0], posit_out[1], posit_out[2], quat_out[0], quat_out[1], quat_out[2], quat_out[3]]


def correctTum(filename, filename_out, tf_matrix):
    data_out = []
    with open(filename, 'r') as file:
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

    with open(filename_out, 'w') as file:
        writer = csv.writer(file, delimiter=" ")
        writer.writerows(data_out)


path = "/home/mbo/norlab_ws/data/results/trajectories/"
filename = path + "traj_icp_odom_base_link.bag.txt"
filename_out = path + "ICP_out.txt"
correction = np.identity(4)
correctTum(filename, filename_out, correction)
