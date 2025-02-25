import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree

def create_mesh_ball_pivoting(point_cloud, radii):
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        point_cloud, o3d.utility.DoubleVector(radii)
    )
    return mesh

def create_mesh_alpha_shape(point_cloud, alpha):
    # Create a watertight mesh using the alpha shape method
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        point_cloud, alpha
    )
    return mesh

def post_process_mesh(mesh):
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    return mesh

def create_save_mesh(points, name="mesh.stl", use_alpha_shape=False, alpha=100.0):
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=2)
    dists = np.linalg.norm(distances, axis=1)
    min_d = np.min(dists)
    max_d = np.max(dists)

    ### Radii: This is a key feature of the meshing process! 
    #### Enough radii need to be specified in a diverse range to capture the possible distances between points.
    radii = [
        0.125 * min_d, 0.25 * min_d, 0.5 * min_d, min_d, 
        (min_d + max_d) / 3, (min_d + max_d) / 2, 
        2 * (min_d + max_d) / 3, max_d, 2 * max_d, 
        4 * max_d, 8 * max_d
    ]

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0, max_nn=30)
    )
    
    point_cloud = point_cloud.remove_duplicated_points()

    # Use the chosen mesh creation method
    if use_alpha_shape:
        mesh = create_mesh_alpha_shape(point_cloud, alpha)
    else:
        mesh = create_mesh_ball_pivoting(point_cloud, radii)

    # Post-process the mesh
    mesh = post_process_mesh(mesh)
    mesh = mesh.filter_smooth_simple(number_of_iterations=10)
    
    # Fill holes in the mesh (legacy approach)
    for _ in range(10):
        mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh).fill_holes().to_legacy()

    mesh.compute_vertex_normals()

    # Save the mesh
    o3d.io.write_triangle_mesh(name, mesh)
    print("STL file saved as", name)
    return mesh

if __name__ == "__main__":
    revolved_outer = np.load('point_cloud_outer.npy')
    revolved_inner = np.load('point_cloud_inner.npy')
    mesh_outer = create_save_mesh(revolved_outer, "mesh_outer.stl", use_alpha_shape=False)
    mesh_inner = create_save_mesh(revolved_inner, "mesh_inner.stl", use_alpha_shape=False)