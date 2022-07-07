from apriltags_eth import make_default_detector
import numpy as np
import open3d as o3d
import math
import yaml
import open3d.visualization.gui as gui

def non_zero_mean(np_arr):
    exist = (np_arr > 0)
    numer = np_arr.sum(axis=0)
    den = exist.sum(axis=0)
    return numer/den

def np2string(nplist):
    line = str(nplist[0])
    for i in nplist[1:]:
        line += ", {}".format(i)
    return line

def rgb2gray(rgb):
    """trans rgb to grey

    Args:
        rgb (np.array[H, W, 3]): rgb image

    Returns:
        np.uint8 array [H,W]: grey image
    """
    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    return gray.astype(np.uint8)

def eulerAnglesToRotationMatrix(theta) :
    """euler angles to R

    Args:
        theta (np.array or list[3*1]): euler angles

    Returns:
        (np.array[3*3]): RotationMaxtrix
    """
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

class PlyAprilTagDetector:
    def __init__(self, config, tag_size, ply = None):
        self.tag_size = tag_size
        self.detector = make_default_detector()
        self.tags = {}
        self.tags_geo = []
        self.win_width = 1280
        self.win_height = 720
        if ply is not None:
            self.ply = ply

    def corners_get_depth(self, corner, depth):
        # TODO: 可以做双线性插值或者更稳定的深度信息提取
        corner = corner.astype(np.int32)
        corner_floor = np.floor(corner).astype(np.int32)
        bais = np.array([[0,0], [0,1], [1,0], [1,1]])
        depths = []
        for b in bais:
            c = corner_floor + b
            d = depth[c[:,1], c[:,0]]
            depths.append(d)
        depths = np.vstack(depths)
        depths = non_zero_mean(depths)
        if np.sum(depths<1e-5) > 0:
            print("Warning: zero depth appears, point cloud may be too sparse!")
        return depths
    
    def get_corner_by_depth(self, im, depth, cam_params):
        """Obtain the 3D coordinates of the corners through the depth of point cloud rendering

        Args:
            im (_type_): image of point cloud rendering
            depth (_type_): depth of point cloud rendering
            cam_params (_type_): cam parameters of point cloud rendering

        Returns:
            list(int): newly detected tag_id
        """
        cam_intrinsic = cam_params.intrinsic.intrinsic_matrix
        cam_extrinsic = cam_params.extrinsic
        new_tag_id = []
        if len(im.shape) > 2 and im.shape[2] == 3:
            im = rgb2gray(im*255)
        results = self.detector.extract_tags(im)
        for r in results:
            corners = np.array(r.corners)
            id = r.id
            corners_z = self.corners_get_depth(corners, depth)
            corners_cam = np.linalg.inv(cam_intrinsic) @ np.hstack([corners, np.ones((4,1))]).transpose() * corners_z
            corners_cam = corners_cam.transpose()
            diff = np.roll(corners_cam, 1 , axis=0 ) - corners_cam
            e = np.mean(np.abs(np.sqrt(np.sum(diff ** 2, 1)) - self.tag_size))
            if str(id) in self.tags.keys():
                if e > self.tags[str(id)]["side_len_error"]:
                    continue
                else:
                    corners_world = (np.linalg.inv(cam_extrinsic) @ np.hstack([corners_cam, np.ones((4,1))]).transpose()).transpose()[:,:3]

                    self.tags[str(id)]["vertex"] = corners_world
                    self.tags[str(id)]["side_len_error"] = e
            else:
                corners_world = (np.linalg.inv(cam_extrinsic) @ np.hstack([corners_cam, np.ones((4,1))]).transpose()).transpose()[:,:3]
                result = {}
                result["vertex"] = corners_world
                result["side_len_error"] = e
                self.tags[str(id)] = result
                new_tag_id.append(id)
        return new_tag_id
    
    def manual_traverse_ply(self, look_around_hegiht = 1.0):
        """Manual look-around traversal of point clouds

        Args:
            look_around_hegiht (float, optional): The height of the viewing angle when looking around (horizontal height). Defaults to 1.0.
        """
        def identity_T(vis):
            camera_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
            world2cam = np.array([[1.0,0,0,0],[0,0,-1.0, look_around_hegiht],[0,1.0,0,0],[0,0,0,1.0]])
            camera_params.extrinsic = world2cam
            vis.get_view_control().convert_from_pinhole_camera_parameters(camera_params)
            vis.update_renderer()

        def capture_image(vis):
            image = vis.capture_screen_float_buffer()
            depth = vis.capture_depth_float_buffer()
            vis.poll_events()
            cam_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
            new_tag_id = self.get_corner_by_depth( np.asarray(image), np.asarray(depth), cam_params)
            if len(new_tag_id) > 0:
                print(f"ADD {len(new_tag_id)} new tags : {new_tag_id}")
                for tag_id in new_tag_id:
                    tag_geo = self.generate_tag_geo(int(tag_id))
                    vis.add_geometry(tag_geo)
                    vis.get_view_control().convert_from_pinhole_camera_parameters(cam_params)
            vis.update_renderer()
            return False

        print("   Press 'D' to detect apriltag")
        print("   Press ' ' to arrive the origin of world coordinate system")

        key_to_callback = {}
        key_to_callback[ord("D")] = capture_image
        key_to_callback[ord(" ")] = identity_T
        o3d.visualization.draw_geometries_with_key_callbacks([self.ply], key_to_callback, width=self.win_width, height=self.win_height)

    def traverse_ply(self, look_around_hegiht = 1.0, detect_angle_inv = 10.0):
        """Automatic look-around traversal of point clouds

        Args:
            look_around_hegiht (float, optional): The height of the viewing angle when looking around (horizontal height). Defaults to 1.0.
            detect_angle_inv (float, optional): Angular interval for automatic detection of calibration plates when looking around. Defaults to 10.0.
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=self.win_width, height=self.win_height)
        vis.get_render_option().point_size = 2
        vis.add_geometry(self.ply)
        camera_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
        world2cam = np.array([[1.0,0,0,0],[0,0,-1.0, look_around_hegiht],[0,1.0,0,0],[0,0,0,1.0]])
        camera_params.extrinsic = world2cam
        euler_ang = np.array([0.,0.,0.])
        
        detect_iter = int(360/detect_angle_inv)
        for i in range(360):
            vis.get_view_control().convert_from_pinhole_camera_parameters(camera_params)
            cam_trans = np.eye(4)
            euler_ang[1] += math.pi *2 / 360
            cam_trans[2,3] = 1.0
            cam_trans[:3,:3] = eulerAnglesToRotationMatrix(euler_ang)
            camera_params.extrinsic = cam_trans @ world2cam
            vis.poll_events()
            vis.update_renderer()
            if i % detect_iter == 0:
                image = vis.capture_screen_float_buffer()
                depth = vis.capture_depth_float_buffer()
                new_tag_id = self.get_corner_by_depth( np.asarray(image), np.asarray(depth), vis.get_view_control().convert_to_pinhole_camera_parameters())
                if len(new_tag_id) > 0:
                    print(f"ADD {len(new_tag_id)} new tags : {new_tag_id}, tag_num = {len(self.tags)}")
                    for tag_id in new_tag_id:
                        tag_geo = self.generate_tag_geo(int(tag_id))
                        vis.add_geometry(tag_geo)
        self.cam_intrinsic = camera_params.intrinsic.intrinsic_matrix
        vis.destroy_window()

    def vis_all(self, look_around_hegiht=1.0, vis_scene = True):
        """Visualize all tags and scene point clouds

        Args:
            look_around_hegiht (float, optional): The height of the viewing angle when looking around (horizontal height). Defaults to 1.0.
            vis_scene (bool, optional): need to visualize the scene point cloud. Defaults to True.
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=self.win_width, height=self.win_height)
        vis.get_render_option().point_size = 1
        if vis_scene:
            vis.add_geometry(self.ply)
        camera_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
        world2cam = np.array([[1.0,0,0,0],[0,0,-1.0, look_around_hegiht],[0,1.0,0,0],[0,0,0,1.0]])
        camera_params.extrinsic = world2cam
        for tag_id in self.tags.keys():
            tag_geo = self.generate_tag_geo(int(tag_id))
            vis.add_geometry(tag_geo)
        vis.get_view_control().convert_from_pinhole_camera_parameters(camera_params)
        vis.run()
        vis.destroy_window()
    
    def filter_pointcloud(self, dist = 2.0):
        """filter pointcloud, delete points far from the center of the tags

        Args:
            dist (float, optional): distance threshold. Defaults to 2.0.
        """
        points = np.asarray(self.ply.points)
        colors = np.asarray(self.ply.colors)

        near_tag_all = np.zeros(points.shape[0]) > 1
        for tag_id in self.tags.keys():
            tag_center = np.mean(self.tags[tag_id]["vertex"],0)
            near_tag = np.linalg.norm(points - tag_center, axis = 1) < dist
            near_tag_all = np.logical_or(near_tag_all, near_tag)
        
        filtered_points = points[near_tag_all]
        filtered_colors = colors[near_tag_all]
        self.filtered_ply = o3d.geometry.PointCloud()
        self.filtered_ply.points = o3d.utility.Vector3dVector(filtered_points)
        self.filtered_ply.colors = o3d.utility.Vector3dVector(filtered_colors)

    def vis_gui(self, look_around_hegiht=1.0, filter_ply = True):
        """Visualize(GUI) all tags and scene point clouds

        Args:
            look_around_hegiht (float, optional): The height of the viewing angle when looking around (horizontal height). Defaults to 1.0.
            filter_ply (bool, optional): need to filter point cloud. Defaults to True.
        """
        app = gui.Application.instance
        app.initialize()
        vis = o3d.visualization.O3DVisualizer("Open3D - 3D Text", 1024, 768)
        vis.show_settings = True
        if filter_ply:
            self.filter_pointcloud()
            vis.add_geometry("scene_ply", self.filtered_ply)
        else:
            vis.add_geometry("scene_ply", self.ply)
        for tag_id in self.tags.keys():
            tag_geo = self.generate_tag_geo(int(tag_id))
            vis.add_geometry(f"tags_{tag_id}", tag_geo)
            vis.add_3d_label(self.tags[tag_id]["vertex"][0], "tag_{}".format(tag_id))
        world2cam = np.array([[1.0,0,0,0],[0,0,-1.0, look_around_hegiht],[0,1.0,0,0],[0,0,0,1.0]])
        extrinsic = world2cam
        vis.setup_camera(self.cam_intrinsic, extrinsic, self.win_width, self.win_height)
        app.add_window(vis)
        app.run()

    def eval_side_len(self):
        """Evaluate the error between the calculated vertex side length and tag_size.
           error will be saved in self.tags[tag_id]['side_len_error']
        """
        side_len_error = []
        for tag_id in self.tags.keys():
            diff = np.roll(self.tags[tag_id]['vertex'], 1 , axis=0 ) - self.tags[tag_id]['vertex']
            e = np.mean(np.abs(np.sqrt(np.sum(diff ** 2, 1)) - self.tag_size))
            side_len_error.append(e)
            self.tags[tag_id]['side_len_error'] = e
        side_len_error = np.array(side_len_error)
        print("side_len_error_mean = ", np.mean(side_len_error))

    def refine_tags(self, dist = 2):
        # TODO: 增加tag pose refine
        pass
    
    def save_yaml(self, path):
        with open(path, "w") as f:
            f.write("MapPoints:\n")
            for i, tag_id in enumerate(self.tags.keys()):
                f.write("  {}:\n".format(i))
                f.write("    tag_id: {}\n".format(tag_id))
                for ti in range(4):
                    f.write("    corner{}: [{}]\n".format(ti, np2string(self.tags[tag_id]["vertex"][ti])))
        print("tag_yaml saved in :", path)

    def generate_tag_geo(self, id, c = [1,0,0]):
        """Quadrilateral geometry for tag generation from four vertex points

        Args:
            id (_type_): tag id
            c (list, optional): tag geo color. Defaults to [1,0,0].

        Returns:
            o3d.geometry.LineSet: tag border geo
        """
        points = self.tags[str(id)]["vertex"]
        lines = [[0,1], [1,2], [2,3], [3,0]]
        lines_pcd = o3d.geometry.LineSet()
        lines_pcd.lines = o3d.utility.Vector2iVector(lines)
        lines_pcd.paint_uniform_color(c)
        lines_pcd.points = o3d.utility.Vector3dVector(points)
        return lines_pcd

if __name__ == "__main__":

    with open('../config/detector.yaml') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    
    pcd = o3d.io.read_point_cloud(config["file"]["ply_path"])
    detecter = PlyAprilTagDetector(config, tag_size=config["detector"]["tag_size"], ply=pcd)
    if config["detector"]["mode"] == "auto":
        detecter.traverse_ply(detect_angle_inv=config["detector"]["detect_angle"])
    elif config["detector"]["mode"] == "manual":
        detecter.manual_traverse_ply()
    else:
        print("Error mode (auto or manual)!")

    detecter.filter_pointcloud()
    detecter.eval_side_len()
    if config["detector"]["display_result"]:
        detecter.vis_gui(filter_ply=config["detector"]["filter_ply"])

    detecter.save_yaml(config["file"]["tag_result_path"])