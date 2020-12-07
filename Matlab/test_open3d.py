import open3d as o3d
import os
import numpy as np
from matplotlib import pyplot as plt

def _relative_path(path):
    script_path = os.path.realpath(__file__)
    script_dir = os.path.dirname(script_path)
    return os.path.join(script_dir, path)



if __name__ == "__main__":
    # path = _relative_path("Surfel.ply")
    path = _relative_path("PtCloud.ply")

    pcd = o3d.io.read_point_cloud(path)
    o3d.visualization.draw_geometries([pcd],
                                    zoom=0.4,
                                    front=[-1.0, 0.0, -0.01],
                                    lookat=[1.0, -7.2596, 0.9284],
                                    up=[1.0, -0.0, 0.0])

    print('run Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
    print(mesh)
    o3d.visualization.draw_geometries([mesh],
                                    zoom=0.4,
                                    front=[-1.0, 0.0, -0.01],
                                    lookat=[1.0, -7.2596, 0.9284],
                                    up=[1.0, -0.0, 0.0])

    o3d.io.write_triangle_mesh("test_mesh.ply", mesh)

    print('visualize densities')
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    o3d.visualization.draw_geometries([density_mesh],
                                    zoom=0.4,
                                    front=[-1.0, 0.0, -0.01],
                                    lookat=[1.0, -7.2596, 0.9284],
                                    up=[1.0, -0.0, 0.0])