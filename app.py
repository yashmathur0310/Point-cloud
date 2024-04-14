import streamlit as st
import numpy as np
import open3d as o3d
from PIL import Image

def generate_point_cloud(image):
    image_array = np.array(image)
    r = image_array[:,:,0]
    g = image_array[:,:,1]
    b = image_array[:,:,2]
    rows, cols, _ = image_array.shape
    points = []
    for r_idx in range(rows):
        for c_idx in range(cols):
            points.append([r_idx, c_idx, r[r_idx, c_idx], g[r_idx, c_idx], b[r_idx, c_idx]])
    points = np.array(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:,:3]) 
    pcd.colors = o3d.utility.Vector3dVector(points[:,2:] / 255.0) 
    return pcd

def visualize_point_cloud(point_cloud):
    rotation_matrix = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    point_cloud.rotate(rotation_matrix, center=(0, 0, 0))
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    geometries = [point_cloud, mesh]
    o3d.visualization.draw_geometries(geometries)

def main():
    st.title("Colored Image to Point Cloud")
    uploaded_file = st.file_uploader("Upload a colored image", type=["jpg", "jpeg", "png"])
    if uploaded_file is not None:
        image = Image.open(uploaded_file)
        st.image(image, caption='Uploaded Image', use_column_width=True)
        point_cloud = generate_point_cloud(image)
        st.write("Visualizing the point cloud...")
        visualize_point_cloud(point_cloud)

if __name__ == "__main__":
    main()
