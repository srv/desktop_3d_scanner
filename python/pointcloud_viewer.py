import numpy as np
import open3d
import time

class ReconstructionViewer(object):
    def __init__(self):
        self.model = open3d.PointCloud()
        self.vis = open3d.Visualizer()
        self.vis.create_window(window_name= 'Laser 3D Pointcloud', width= 800L, height= 600L, left= 50L, right= 50L)
        self.ctr = self.vis.get_view_control()
        self.ctr.rotate(10,0)
        self.ctr.translate(-1,-1,1)

    def append(self, xyz_cloud, pose):
        pcd = open3d.PointCloud()
        pcd.points = open3d.Vector3dVector(xyz_cloud)
        pcd.transform(pose)
        self.model = self.model + pcd

    def drawnow(self):
        # Actualitzar el visor
        self.vis.add_geometry(self.model)
        self.vis.update_geometry()
        self.vis.reset_view_point(False)
        self.vis.poll_events()
        self.vis.update_renderer()
        time.sleep(0.1)

    def run(self):
        self.vis.run()
        
    def save(self):
        open3d.write_point_cloud("reconstruction.ply", self.model)



#table_pose = np.identity(4)
#table_incr = np.array([[0.984,-0.17,0,0],
#                       [0.17,0.984,0,0],
#                       [0,0,1,0],
#                       [0,0,0,1]])
#laser_points = np.array([[0.1,0.2,1],
#                         [0.11,0.21,1.1],
#                         [0.12,0.22,1.2],
#                         [0.13,0.23,1.3],
#                         [0.14,0.24,1.4],
#                         [0.15,0.25,1.5]])
#
#viewer  = ReconstructionViewer()
#
#for i in range(36):
#    viewer.append(laser_points, table_pose)
#    table_pose = np.matmul(table_pose, table_incr)
#    viewer.drawnow()
#    time.sleep(0.1)
#viewer.run()


# pcd = open3d.PointCloud()
# pcd.points = open3d.Vector3dVector(laser_points)

# # Augmentem el model resultant
# model = model + pcd

# for i in range(36):
#     # Transformar el PointCloud actual segons una matriu de transformacio
#     pcd.transform(table_pose)
#     # Augmentem el model resultant amb "la nova lectura transformada"
#     model = model + pcd

#     # Mostra el resultat a la pantalla
#     vis.add_geometry(model)

#     # Actualitzar el visor
#     vis.update_geometry()
#     vis.reset_view_point(True)
#     vis.poll_events()
#     vis.update_renderer()

#     # Esperar per visualizar millor
#     time.sleep(0.5)

# # Guardar el resultat al disc dur
# open3d.write_point_cloud("reconstruction.ply", model)
# vis.run()

