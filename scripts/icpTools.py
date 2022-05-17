#! /usr/bin/env python3.6
import numpy as np
from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms
from pathlib import Path

DP = pm.PointMatcher.DataPoints
PM = pm.PointMatcher

home_dir = Path.home()
hilti_dir = home_dir / "hilti_ws/src/hiltislamchallenge"
DATA_PATH = str(hilti_dir / "results/online/exp09_cupola/vtks") + "/"
YAML_PATH = str(hilti_dir / "params/realtime_icp_config.yaml")
REFERENCE_NAME = "03acupola_map.vtk"
READING_NAME = "03bcupola_map.vtk"
READING_TRAJ_NAME = READING_NAME.rsplit("_", 1)[0] + "_traj.vtk"
print("Folder %s" % DATA_PATH)
print("Reference: %s" % REFERENCE_NAME)
print("Reading: %s" % READING_NAME)

# icp
icp = PM.ICP()
icp.setDefault()
icp.loadFromYaml(YAML_PATH)

T_init = np.array(
    [
        [0.5537509, -0.8209695, 0.1391731, -15.5],
        [0.7806463, 0.5700014, 0.2563002, -5.3],
        [-0.2897435, -0.0332815, 0.9565255, 5.5],
        [0, 0, 0, 1],
    ]
)


reference_cloud = DP(DP.load(DATA_PATH + REFERENCE_NAME))
reading_cloud = DP(DP.load(DATA_PATH + READING_NAME))
print("Transforming reading with the initial transformation")
print(T_init)
icp.transformations.apply(reading_cloud, (T_init))

reading_cloud.save(DATA_PATH + READING_NAME.split(".")[0] + "_transformed_init.vtk")

T = icp(reading_cloud, reference_cloud)
print("Transforming reading with the ICP transformation")
print(T)
icp.transformations.apply(reading_cloud, T)
reading_cloud.save(DATA_PATH + READING_NAME.split(".")[0] + "_transformed_icp.vtk")
T_combined = T.dot(T_init)
print("Combined transformation:")
print(T_combined)

# save trajectory
reading_traj = DP(DP.load(DATA_PATH + READING_TRAJ_NAME))
print("Applying the final transform on the trajectory file %s" % READING_TRAJ_NAME)
icp.transformations.apply(reading_traj, T_combined)
reading_traj.save(DATA_PATH + READING_TRAJ_NAME.split(".")[0] + "_transformed_icp.vtk")

# concatenation
reference_cloud.concatenate(reading_cloud)
reference_cloud.save(
    DATA_PATH
    + "concatenated_"
    + REFERENCE_NAME.split(".")[0]
    + "_"
    + READING_NAME.split(".")[0]
    + ".vtk"
)


# apply single transformation
# T = [[0.9958519, -0.0764168, 0.0493902, -8.376],
#      [0.0830509, 0.9851406, -0.1503352, -2.206],
#      [-0.0371681, 0.1538135, 0.9874006, 2.041],
#      [0, 0, 0, 1]]
#
# reference_cloud = DP(DP.load(DATA_PATH + "part1_end115s_traj.vtk"))
# icp = PM.ICP()
# icp.setDefault()
# icp.transformations.apply(reference_cloud, np.linalg.inv(T))
# reference_cloud.save(DATA_PATH + "part1_end115s_traj_transformed.vtk")
