import numpy as np
import open3d as o3d
import pickle

def load_data(data_path):
    '''
    Load data dictionary from data_path.
    '''
    with open(data_path, 'rb') as fp:
        data = pickle.load(fp)
    return data

def load_from_bin(bin_path):
    # load point cloud from a binary file
    obj = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    # ignore reflectivity info
    return obj[:, :3]

# Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
# xyz = load_from_bin("/home/hanlonm/HoleDet/pcl_edge_detection/data/0000000037.bin")
# data = load_data("/home/hanlonm/python_projects/DLAD/exercise1/data/data.p")
data = load_data("/home/hanlonm/python_projects/DLAD/exercise1/data/data.p")
xyz = data['velodyne'][:,:3]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.io.write_point_cloud("./demo.pcd", pcd)