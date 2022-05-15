import numpy as np
from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms

DP = pm.PointMatcher.DataPoints
PM = pm.PointMatcher

DATA_PATH = "/home/mbo/norlab_ws/data/results/hilti/slow_knn15/"
YAML_PATH = "/home/mbo/norlab_ws/src/hiltislamchallenge/params/realtime_icp_config.yaml"
REFERENCE_NAME = "part1_map.vtk"
READING_NAME = "part3_map.vtk"
READING_TRAJ_NAME = READING_NAME.split("_")[0] + "_traj.vtk"
print("Folder %s" % DATA_PATH)
print("Reference: %s" % REFERENCE_NAME)
print("Reading: %s" % READING_NAME)

# icp
icp = PM.ICP()
icp.setDefault()
icp.loadFromYaml(YAML_PATH)

T_init = np.array([[-0.9863347, -0.0516916, 0.1564345, 9.346933987561371],
                         [0.0709843, -0.9901881, 0.1203689, 0.6993824044186248],
                         [0.1486775, 0.1298285, 0.9803262, 1.9887952445529913],
                         [0, 0, 0, 1]])


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
reference_cloud.save(DATA_PATH + "concatenated_"+REFERENCE_NAME.split(".")[0]+"_"+READING_NAME.split(".")[0]+".vtk")


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
