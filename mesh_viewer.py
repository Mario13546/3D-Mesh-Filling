import open3d as o3d

mesh_outer = o3d.io.read_triangle_mesh("pdg_mesh_outer.stl")
mesh_inner = o3d.io.read_triangle_mesh("pdg_mesh_inner.stl")

o3d.visualization.draw_geometries([mesh_outer, mesh_inner])