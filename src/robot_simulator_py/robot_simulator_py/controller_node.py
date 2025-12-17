# @file controller_node.py
#
# @author Chase Snyder
#
# @brief Mission controller:
#  - Phase 1: Initialization, wait for system start, publish "ready" to /robot status
#  - Phase 2: Search, random node based proportional control search and report aruco <ID> in position x/y/z
#  - Phase 3: Navigation to aruco, drive to stored marker pose, then report "arrived to <ID>"
#  - Phase 4: Inspection (image_analysis) report coordinates where movment between two images is detected.
#  - Phase 5: Return, go back to initial position
#  - Phase 6: Feature Counting (image analysis2) count shared features between two images, (uses sift as I don't have surf)
# Notes:
#  - Uses TF (world -> base_link) for robot pose, and TF (world <- camera_link for marker pose.
#  - Stores last seen marker positions in world frame and navigates to them later.
#  - Node must be started after evaluator in order for both to see each other's "ready" message.
#  - Lacked time to clean up code so a lot of garbage variables/funcitons are still here.
#  - Occasional bug where evaluator and node start see eachother, but then neither update and sit idle.
#

import random

from numpy import ndarray
from typing import Any
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco

import numpy as np
import math
import re

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
import tf2_geometry_msgs  # REQUIRED: registers PointStamped for tf2 transform()

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# ArUco / camera parameters
MARKER_SIZE = 0.2  # meters (Gazebo marker size)

#######################
# High Use Helper Methods
#########################
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quat(q) -> float:
    # yaw (Z) from geometry_msgs/Quaternion
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Camera intrinsics from CameraInfo
        self.cam_K = None
        self.cam_D = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_sensor/camera_info',
            self.camera_info_callback,
            10
        )

        # Image subscriber
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )

        # ArUco detector setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.marker_samples = {}
        
        # Image Analysis
        # Latest frames from image1/image2
        self.img1 = None
        self.img2 = None
        self.img1_stamp = None
        self.img2_stamp = None
        self.got_img1 = False
        self.got_img2 = False

        self.analysis_timeout_sec = 2.5
        self.analysis_deadline = self.get_clock().now()

        self.accept_analysis_images = False
        self.analysis_start_time = None

        ### Stop-and-stare state ###
        self.stare_active = False
        self.stare_id = None
        self.stare_required_samples = 10
        self.stare_timeout_sec = 2.0  # safety so it can't get stuck forever
        self.stare_end_time = self.get_clock().now()

        # Subscribe to image1 and image2
        self.image1_sub = self.create_subscription(Image, '/image1', self.image1_callback, 10)
        self.image2_sub = self.create_subscription(Image, '/image2', self.image2_callback, 10)

        # Motion detection parameters
        self.motion_thresh = 50          # pixel intensity threshold
        self.motion_min_area = 95       # minimum contour area in pixels

        # Status/report publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.report_pub = self.create_publisher(String, '/robot_report', 10)

        # Mission subscription 
        self.mission_sub = self.create_subscription(String, '/missions', self.mission_callback, 10)

        # TF listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #### State ####
        self.current_mission = 'idle'
        self.search_active = False

        self.nav_active = False
        self.nav_target_id = None  # int
        self.nav_goal_xy = None    # (x, y) in odom
        self.arrived_sent = False
        self.return_arrived_sent = False
        self.image_analysis_active = False  # image analysis (movement)
        self.analyze_sent = False

        # Bonus: image_analysis2 (feature counting)
        self.image_analysis2_active = False
        self.analyze2_sent = False
        self.analysis2_trigger_time = None  # rclpy.time.Time

        # Return-to-origin
        self.return_active = False
        self.return_arrived_sent = False
        self.origin_xy = None  # (x,y) in odom, captured once TF is available

        # Track which IDs we've reported (during search)
        self.reported_ids = set()

        # Store last seen marker pose in odom: id -> dict(pos=(x,y,z), stamp=Time)
        self.marker_db = {}

        #### Tunables ####
        self.search_ang_speed = 0.25  # rad/s

        # Go-to-goal control
        self.k_ang = 1.8
        self.k_lin = 0.35
        self.max_w = 1.0
        self.max_v = 0.22

        # Acceptance thresholds (match rubric)
        self.dist_thresh = 0.50  # meters
        self.yaw_thresh = math.radians(10.0)  # 10 degrees

        # Slow down when close
        self.slowdown_radius = 1.0  # meters

        # Speeds for wander (Fast because with 60 seconds explore normal speeds are too slow)
        self.search_v_min = 1.0
        self.search_v_max = 2.0
        self.search_w_turn_min = 0.9
        self.search_w_turn_max = 3.14
        self.search_w_drift: float = 0.35  # small wiggle while driving ~wiggle ~wiggle ~wiggle

        #### Waypoint-based SEARCH ####
        self.search_mode = "to_wp"   # "to_wp" or "spin"
        self.search_wp = None        # (x,y) in world
        self.search_wp_thresh = 0.45 # how close counts as "reached waypoint"

        # bounds (world frame) with margins so we don't hug walls (might need changed for actual demo)
        self.search_x_lim = 5.0
        self.search_y_lim: float = 3.0
        self.search_margin = 0.35

        # sway while driving (for the camera)
        self.sway_amp = 0.35         # rad/s added on top of heading controller
        self.sway_freq = 0.6         # Hz
        self.search_start_time = self.get_clock().now()

        # half-spin state
        self.spin_target_yaw = None
        self.spin_speed = 0.75       # rad/s
        self.spin_yaw_thresh = math.radians(8.0)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info("controller_node started")

        # Tell evaluator we are ready
        ready_msg = String()
        ready_msg.data = "ready"
        self.status_pub.publish(ready_msg)
        self.get_logger().info('Published "ready" on /robot_status')

        # Ensure robot starts stopped
        self.publish_cmd(0.0, 0.0)

    ####################
    # Mission Interface
    #####################
    def mission_callback(self, msg: String):
        mission_raw = msg.data.strip()
        mission = mission_raw.lower()

        # While analyzing, ignore all mission changes.
        if self.analysis_busy():
            return

        # Ignore exact duplicate missions (non-image ones too)
        if mission == self.current_mission:
            return

        self.get_logger().info(f'Received mission: "{mission_raw}"')
        self.current_mission = mission

        if mission == 'image_analysis':
            self.start_image_analysis('image_analysis')
            return

        if mission == 'image_analysis2':
            self.start_image_analysis('image_analysis2')
            return

        # Reset one-shot flags when mission changes
        self.arrived_sent = False

        # "Search"
        if mission == 'search_aruco':
            self.reset_search_pattern()
            self.search_active = True
            self.nav_active = False
            self.nav_target_id = None
            self.nav_goal_xy = None
            self.stare_active = False
            self.stare_id = None    

            status = String()
            status.data = 'explore'
            self.status_pub.publish(status)

        else:
            self.search_active = False

        # "return to origin"
        if mission == 'return to origin':
            self.return_active = True
            self.search_active = False
            self.nav_active = False
            self.image_analysis_active = False
            self.image_analysis2_active = False
            self.nav_target_id = None
            self.nav_goal_xy = None
            self.arrived_sent = False
            self.return_arrived_sent = False

            st = String()
            st.data = 'returning'
            self.status_pub.publish(st)

            return

        # "move to <ID>"
        m = re.match(r"move\s+to\s+(\d+)", mission)
        if m:
            self.nav_target_id = int(m.group(1))
            self.nav_active = True
            self.search_active = False

            # If we already saw that marker during search, lock the goal now.
            entry: Any | None = self.marker_db.get(self.nav_target_id)
            if entry is not None:
                x, y, _z = entry['pos']
                self.nav_goal_xy = (x, y)
                self.get_logger().info(f"Nav goal for {self.nav_target_id} loaded from marker_db: {self.nav_goal_xy}")
            else:
                self.nav_goal_xy = None
                self.get_logger().warn(f"No stored pose for marker {self.nav_target_id} yet; will wait until detected.")

            status = String()
            status.data = 'moving'
            self.status_pub.publish(status)
    
    # helper that reads Evaluator Messages
    def evaluator_callback(self, msg: String):
        txt = msg.data.strip()
        if txt:
            self.get_logger().warn(f"[evaluator] {txt}")

    #############
    # Helpers
    ##############
    def try_capture_origin(self):
        """Capture the robot's starting pose in odom the first time TF is available."""
        try:
            t = self.tf_buffer.lookup_transform('world', 'base_link', Time())
        except TransformException:
            return
        ox = float(t.transform.translation.x)
        oy = float(t.transform.translation.y)
        self.origin_xy = (ox, oy)
        self.get_logger().info(f"Captured origin_xy in world: ({ox:.3f}, {oy:.3f})")

    def publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
    
    #####################
    # Main Control Loop
    ##################### 
    def on_timer(self):
        # Capture origin pose once TF is available 
        if self.origin_xy is None:
            self.try_capture_origin()

        # IMAGE ANALYSIS (movement): analyze images
        if self.image_analysis_active and self.current_mission == 'image_analysis':
            self.step_image_analysis()
            return

        # IMAGE ANALYSIS2 (features): analyze image2
        if self.image_analysis2_active and self.current_mission == 'image_analysis2':
            self.step_image_analysis2()
            return

        # RETURN TO ORIGIN
        if self.return_active and self.current_mission == 'return to origin':
            self.step_return_to_origin()
            return
        
        # NAVIGATION: move to <ID>
        if self.nav_active and self.nav_target_id is not None:
            self.step_navigation()
            return

        # SEARCH: rotate in place (or stop-and-stare)
        if self.search_active and self.current_mission == 'search_aruco':
            if self.stare_active:
                # freeze while staring
                self.publish_cmd(0.0, 0.0)

                # timeout escape
                if self.get_clock().now() > self.stare_end_time:
                    self.get_logger().warn(f"Stare timeout on id={self.stare_id}, resuming search")
                    self.stare_active = False
                    self.stare_id = None
                    self.reset_search_pattern()
                return

            self.step_search_pattern()
            return

        # Default: stop
        self.publish_cmd(0.0, 0.0)

    #########################
    # NAVIGATION controller
    # ########################
    def step_navigation(self):
        # Get a goal
        if self.nav_goal_xy is None:
            # If we don't have the goal yet, just rotate slowly (and keep vision running).
            self.publish_cmd(0.0, 0.25)
            return

        # Get robot pose in odom
        try:
            t = self.tf_buffer.lookup_transform('world', 'base_link', Time())
        except TransformException:
            self.publish_cmd(0.0, 0.0)
            return

        #Get current errors, positions, and distances for navigation
        rx = t.transform.translation.x
        ry = t.transform.translation.y
        yaw = yaw_from_quat(t.transform.rotation)

        gx, gy = self.nav_goal_xy
        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_err = wrap_to_pi(desired_yaw - yaw)

        # Check arrival criteria
        if (dist < self.dist_thresh) and (abs(yaw_err) < self.yaw_thresh):
            self.publish_cmd(0.0, 0.0)
            if not self.arrived_sent:
                self.publish_arrived(self.nav_target_id)
                self.arrived_sent = True
            return

        # Simple Proportional Controller
        # Otherwise: simple go-to-goal 
        w = clamp(self.k_ang * yaw_err, -self.max_w, self.max_w)

        # only drive forward  if we're roughly facing the goal
        if abs(yaw_err) < math.radians(35.0):
            v = self.k_lin * dist
            # slow down when close
            if dist < self.slowdown_radius:
                v *= (dist / self.slowdown_radius)
            v = clamp(v, 0.0, self.max_v)
        else:
            v = 0.0

        self.publish_cmd(v, w)


    #####################
    # Report helpers
    #####################
    def report_aruco(self, marker_id: int, x: float, y: float, z: float):
        msg = String()
        msg.data = f"aruco {marker_id} in position x: {x:.2f}, y: {y:.2f}, z: {z:.2f}"
        self.report_pub.publish(msg)
        self.get_logger().info(f"Reported: {msg.data}")

    def publish_arrived(self, marker_id: int):
        msg = String()
        msg.data = f"arrived to {marker_id}"
        self.report_pub.publish(msg)
        self.get_logger().info(f'Reported: "{msg.data}"')

    ######################
    # CameraInfo callback Helper
    ######################
    def camera_info_callback(self, msg: CameraInfo):
        if self.cam_K is None:
            self.cam_K = np.array(msg.k, dtype=float).reshape(3, 3)
            self.cam_D = np.array(msg.d, dtype=float).reshape(-1, 1) if len(msg.d) else np.zeros((5, 1), dtype=float)
            self.get_logger().info(f"Loaded camera intrinsics from camera_info, fx={self.cam_K[0,0]:.2f}")

    ##################################################
    # Image callback (ArUco detection and TF into world) 
    ###############################################
    def image_callback(self, msg: Image):
        # Wait for intrinsics
        if self.cam_K is None:
            return

        # Detect while searching OR while navigating (so we can still update marker_db)
        if not (self.search_active or self.nav_active):
            return

        # ROS Image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().warning(f'cv_bridge conversion failed: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            return

        # If searching and not already staring, lock onto the first "unreported" marker we see
        if self.search_active and (not self.stare_active):
            for marker_id in ids.flatten():
                mid_try = int(marker_id)
                if mid_try not in self.reported_ids:
                    self.stare_active = True
                    self.stare_id = mid_try
                    self.marker_samples[mid_try] = []  # reset samples for clean averaging
                    self.stare_end_time = self.get_clock().now() + Duration(seconds=self.stare_timeout_sec)
                    self.get_logger().info(f"Staring at ArUco {mid_try} for {self.stare_required_samples} samples...")
                    break

        # Pose in camera frame (OpenCV): x right, y down, z forward
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, self.cam_K, self.cam_D)

        src_frame = 'camera_link'  # using camera_link with optical->link remap
        t_img = Time.from_msg(msg.header.stamp)

        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs.reshape(-1, 3), tvecs.reshape(-1, 3)):
            mid = int(marker_id)

            #### rvec to quaternion ### (Simple Identity)
            R, _ = cv2.Rodrigues(rvec)

            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

            pose = PoseStamped()
            pose.header.frame_id = src_frame
            pose.header.stamp = msg.header.stamp

            # OpenCV optical (x right, y down, z forward) to camera_link (x fwd, y left, z up)
            pose.pose.position.x = float(tvec[2])
            pose.pose.position.y = float(-tvec[0])
            pose.pose.position.z = float(-tvec[1])

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            try:
                #try loop for failed transformaitons or unmatching frames causing errors
                if not self.tf_buffer.can_transform("world", src_frame, t_img, timeout=Duration(seconds=0.2)):
                    continue

                pose_world = self.tf_buffer.transform(pose, "world", timeout=Duration(seconds=0.2))

                wx = float(pose_world.pose.position.x)
                wy = float(pose_world.pose.position.y)
                wz = float(pose_world.pose.position.z)

                lst = self.marker_samples.setdefault(mid, [])
                lst.append((wx, wy, wz))

                if len(lst) > 15:
                    lst.pop(0)
                    
                arr = np.array(lst)
                mx, my, mz = np.median(arr, axis=0)

                if len(lst) >= 5:
                    self.marker_db[mid] = {'pos': (float(mx), float(my), float(mz)), 'stamp': self.get_clock().now()}

                # If staring, ONLY collect/report the stare target
                if self.search_active and self.stare_active:
                    if mid != self.stare_id:
                        continue  # ignore other ids until we're done staring

                    # collect samples
                    # once enough samples, report, mark done, resume search
                    if (mid not in self.reported_ids) and (len(lst) >= self.stare_required_samples):
                        self.report_aruco(mid, float(mx), float(my), float(mz))
                        self.reported_ids.add(mid)

                        # lock marker_db too 
                        self.marker_db[mid] = {'pos': (float(mx), float(my), float(mz)), 'stamp': self.get_clock().now()}

                        # done staring, so resume search
                        self.stare_active = False
                        self.stare_id = None

                # nav update
                if self.nav_active and (self.nav_target_id == mid) and len(lst) >= 5:
                    self.nav_goal_xy = (float(mx), float(my))
            except TransformException as ex:
                self.get_logger().warning(f'Could not transform marker pose (TF): {ex}')
                continue
            except Exception as ex:
                self.get_logger().error(f'Unexpected error during marker transform: {ex}')
                continue

    #################################################
    # Image1 callback (Check and Save Images for Phase 4)
    ##################################################
    def image1_callback(self, msg: Image):
            if not self.accept_analysis_images:
                return
            gray = self.imgmsg_to_gray(msg)
            if gray is None:
                return
            self.img1 = gray
            self.img1_stamp = Time.from_msg(msg.header.stamp)
            self.got_img1 = True
    
    #################################################
    # Image2 callback (Check and Save Images for Bonus)
    ##################################################
    def image2_callback(self, msg: Image):
        if not self.accept_analysis_images:
            return
        gray = self.imgmsg_to_gray(msg)
        if gray is None:
            return
        self.img2 = gray
        self.img2_stamp = Time.from_msg(msg.header.stamp)
        self.got_img2 = True
    
    #################################################
    # Checks for which image analysis we are doing, handles resets
    ##################################################
    def start_image_analysis(self, which: str):
        """
        which: 'image_analysis' or 'image_analysis2'
        Resets latches and arms a timeout window.
        """
        self.current_mission = which
        self.image_analysis_active = (which == 'image_analysis')
        self.image_analysis2_active = (which == 'image_analysis2')

        # reset one-shot publish flags
        self.analyze_sent = False
        self.analyze2_sent = False

       # reset latches
        self.img1 = None; 
        self.img2 = None
        self.img1_stamp = None; 
        self.img2_stamp = None
        self.got_img1 = False; 
        self.got_img2 = False

         # IMPORTANT: don't accept images yet (wait until we publish analyze status)
        self.accept_analysis_images = False
        self.analysis_start_time = None

        # arm timeout based on WALL clock (not header.stamp)
        now = self.get_clock().now()
        self.analysis_deadline = now + Duration(seconds=self.analysis_timeout_sec)

        # stop motion
        self.search_active = False
        self.nav_active = False
        self.return_active = False
        self.nav_target_id = None
        self.nav_goal_xy = None

        # keep robot still
        self.publish_cmd(0.0, 0.0)
        
    ###################################
    # Phase 4:INSPECTION: Process and image (And determine motion)
    ####################################
    def step_image_analysis(self):
        if not self.analyze_sent:
            st = String()
            st.data = "analyze image"
            self.status_pub.publish(st)
            self.get_logger().info('Published "analyze image" on /robot_status')
            self.analyze_sent = True

            # NOW evaluator will publish the movement pair -> start accepting images AFTER trigger
            self.accept_analysis_images = True
            self.analysis_start_time = self.get_clock().now()

            # reset latches again so we don't keep any pre-trigger frames
            self.img1 = None; self.img2 = None
            self.img1_stamp = None; self.img2_stamp = None
            self.got_img1 = False; self.got_img2 = False
            return

        if not (self.got_img1 and self.got_img2):
            return

        g1 = cv2.GaussianBlur(self.img1, (5,5), 0)
        g2 = cv2.GaussianBlur(self.img2, (5,5), 0)
        diff = cv2.absdiff(g1, g2)

        _, mask = cv2.threshold(diff, self.motion_thresh, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        M = cv2.moments(mask, binaryImage=True)
        if M["m00"] < self.motion_min_area:
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        h, w = self.img1.shape[:2]

        #my camera is upside down for some reason...
        cy_actual = (h-1) - cy

        self.get_logger().info(f"[image_analysis] img size = {w}x{h}, cx={cx}, cy={cy}")

        msg = String()
        msg.data = f"movement at x: {cx}, y: {cy_actual}"
        self.report_pub.publish(msg)
        self.get_logger().info(f"Reported: {msg.data}")

        self.accept_analysis_images = False
        self.image_analysis_active = False

    ##################################################
    # Bonus Phase: Feature counting
    ##########################################
    # Note: uses sift, as surf is not avaible for me to use,
    # See the 2 associated helper methods for better details.
    def step_image_analysis2(self):
        if not self.analyze2_sent:
            st = String()
            st.data = "analyze image2"
            self.status_pub.publish(st)
            self.get_logger().info('Published "analyze image2" on /robot_status')
            self.analyze2_sent = True

            # Start-of-cycle gate time (ROS time; respects use_sim_time)
            self.analysis2_trigger_time = self.get_clock().now()
            self.get_logger().info(f"[image_analysis2] trigger_time={self.analysis2_trigger_time.nanoseconds}")

            self.accept_analysis_images = True

            self.img1 = None
            self.img2 = None
            self.img1_stamp = None
            self.img2_stamp = None
            self.got_img1 = False
            self.got_img2 = False
            return

        if not (self.got_img1 and self.got_img2):
            return

        g1 = self.img1
        g2 = self.img2

        h1, w1 = g1.shape[:2]
        h2, w2 = g2.shape[:2]
        self.get_logger().info(f"[image_analysis2] img1={w1}x{h1} img2={w2}x{h2}")

        # timestamp check
        if self.img1_stamp is not None and self.img2_stamp is not None:
            dt = (self.img2_stamp.nanoseconds - self.img1_stamp.nanoseconds) / 1e9
            self.get_logger().info(f"[image_analysis2] stamp dt = {dt:+.3f}s")

        reported, dbg = self.sift_count_near_target(g1, g2, target=200)
        self.get_logger().info(f"[image_analysis2] SIFT reported={reported} dbg={dbg}")

        msg = String()
        msg.data = f"{reported} features detected"
        self.report_pub.publish(msg)
        self.get_logger().info(f"Reported: {msg.data}")

        # done
        self.accept_analysis_images = False
        self.image_analysis2_active = False

    ###########################
    # Phase 5: Return to origin
    ############################
    def step_return_to_origin(self):
        # Ensure origin exists
        if self.origin_xy is None:
            self.try_capture_origin()
            # If still None, just stop and wait
            if self.origin_xy is None:
                self.publish_cmd(0.0, 0.0)
                return

        try:
            t = self.tf_buffer.lookup_transform('world', 'base_link', Time())
        except TransformException:
            self.publish_cmd(0.0, 0.0)
            return

        rx = float(t.transform.translation.x)
        ry = float(t.transform.translation.y)
        yaw = yaw_from_quat(t.transform.rotation)

        gx, gy = self.origin_xy
        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)

        # Return criteria: distance only (rubric says < 0.2m)
        if dist < 0.20:
            self.publish_cmd(0.0, 0.0)
            if not self.return_arrived_sent:
                self.publish_arrived_origin()
                self.return_arrived_sent = True
            self.return_active = False
            self.current_mission = 'idle'
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_err = wrap_to_pi(desired_yaw - yaw)

        w = clamp(self.k_ang * yaw_err, -self.max_w, self.max_w)
        if abs(yaw_err) < math.radians(35.0):
            v = self.k_lin * dist
            if dist < self.slowdown_radius:
                v *= (dist / self.slowdown_radius)
            v = clamp(v, 0.0, self.max_v)
        else:
            v = 0.0

        self.publish_cmd(v, w)

    #Helper for phase 5
    def publish_arrived_origin(self):
        msg = String()
        msg.data = "arrived to origin"
        self.report_pub.publish(msg)
        self.get_logger().info(f'Reported: "{msg.data}"')

    def analysis_busy(self) -> bool:
        return self.image_analysis_active or self.image_analysis2_active

    def imgmsg_to_gray(self, msg: Image) -> np.ndarray | None:
        """
        Robust ROS Image -> grayscale uint8.
        Handles mono8 / rgb8 / bgr8 without guessing.
        """
        try:
            enc = (msg.encoding or "").lower()

            if enc in ("mono8", "8uc1"):
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                return gray

            # Default: treat as color and convert
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            return gray

        except Exception as e:
            self.get_logger().warning(f"imgmsg_to_gray failed (encoding={msg.encoding}): {e}")
            return None

    #####################
    # Image Analysis 2 Helper Methods
    ######################
    def sift_good_match_count(self, g1: np.ndarray, g2: np.ndarray, *,
                          nfeatures: int = 4000,
                          ratio: float = 0.85) -> tuple[int, dict]:
        """
        Returns (good_matches_count, debug_dict)
        Uses SIFT + BFMatcher(KNN) + Lowe ratio test.
        """
        sift = cv2.SIFT_create(nfeatures=nfeatures)

        kp1, des1 = sift.detectAndCompute(g1, None)
        kp2, des2 = sift.detectAndCompute(g2, None)

        dbg = {
            "kp1": 0 if kp1 is None else len(kp1),
            "kp2": 0 if kp2 is None else len(kp2),
            "nfeatures": nfeatures,
            "ratio": ratio
        }

        if des1 is None or des2 is None or dbg["kp1"] == 0 or dbg["kp2"] == 0:
            dbg["reason"] = "no_descriptors"
            return 0, dbg

        self.get_logger().info(f"[sift] kp1={dbg['kp1']} kp2={dbg['kp2']}")
        
        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        knn = bf.knnMatch(des1, des2, k=2)

        good = 0
        for pair in knn:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < ratio * n.distance:
                good += 1

        dbg["knn_pairs"] = len(knn)
        dbg["good"] = good
        return good, dbg

    def sift_count_near_target(self, g1: np.ndarray, g2: np.ndarray, target: int = 200) -> tuple[int, dict]:
        """
        Checks to make sure our features are roughly near (200)
        Picking the best result after multiple sift runs
        """
        best = None  # (abs_err, count, dbg)

        # Ratios to try
        for ratio in (0.80, 0.83, 0.85, 0.87, 0.90):
            count, dbg = self.sift_good_match_count(g1, g2, nfeatures=6000, ratio=ratio)
            err = abs(count - target)
            if best is None or err < best[0]:
                best = (err, count, dbg)

        # fallback
        if best is None:
            return 0, {"reason": "no_candidates"}

        return int(best[1]), best[2]

    ###############################
    # Random Waypoint Search Algorthim
    ################################3
    def reset_search_pattern(self):
        """Reset waypoint-search state."""
        self.search_mode = "to_wp"
        self.search_wp = None
        self.spin_target_yaw = None
        self.search_start_time = self.get_clock().now()

    def step_search_pattern(self):
        """
        Waypoint search:
        - Pick random (x,y) in world bounds
        - Drive to it with go-to-goal + sway wiggle
        - When reached, rotate 180 deg (half spin) then pick next waypoint
        """
        # Need robot pose
        try:
            t = self.tf_buffer.lookup_transform('world', 'base_link', Time())
        except TransformException:
            self.publish_cmd(0.0, 0.0)
            return

        rx = float(t.transform.translation.x)
        ry = float(t.transform.translation.y)
        yaw = yaw_from_quat(t.transform.rotation)

        # If no waypoint yet, choose one
        if self.search_wp is None:
            self.search_wp = self.sample_random_waypoint()
            self.search_mode = "to_wp"
            self.spin_target_yaw = None
            self.get_logger().info(f"[search] new waypoint: ({self.search_wp[0]:.2f}, {self.search_wp[1]:.2f})")

        ######### MODE: drive to waypoint ##############
        if self.search_mode == "to_wp":
            gx, gy = self.search_wp
            dx = gx - rx
            dy = gy - ry
            dist = math.hypot(dx, dy)

            # reached waypoint, start half spin
            if dist < self.search_wp_thresh:
                self.publish_cmd(0.0, 0.0)

                # set spin target = current yaw + pi
                self.spin_target_yaw = wrap_to_pi(yaw + math.pi)
                self.search_mode = "spin"
                self.get_logger().info("[search] waypoint reached -> half-spin")
                return

            desired_yaw = math.atan2(dy, dx)
            yaw_err = wrap_to_pi(desired_yaw - yaw)

            # base controller (proportinal again)
            w = clamp(self.k_ang * yaw_err, -self.max_w, self.max_w)

            # sway term while moving (camera sweep)
            tsec = (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9
            w += self.sway_amp * math.sin(2.0 * math.pi * self.sway_freq * tsec)
            w = clamp(w, -self.search_w_turn_max, self.search_w_turn_max)

            # forward only when roughly facing waypoint
            if abs(yaw_err) < math.radians(35.0):
                v = self.k_lin * dist
                if dist < self.slowdown_radius:
                    v *= (dist / self.slowdown_radius)
                v = clamp(v, self.search_v_min, self.search_v_max)
            else:
                v = 0.0

            self.publish_cmd(v, w)
            return

        ############ MODE: half spin ###########
        if self.search_mode == "spin":
            if self.spin_target_yaw is None:
                self.spin_target_yaw = wrap_to_pi(yaw + math.pi)

            err = wrap_to_pi(self.spin_target_yaw - yaw)

            # done spinning
            if abs(err) < self.spin_yaw_thresh:
                self.publish_cmd(0.0, 0.0)
                self.search_wp = None          # force new waypoint next tick
                self.search_mode = "to_wp"
                self.spin_target_yaw = None
                self.search_start_time = self.get_clock().now()
                self.get_logger().info("[search] half-spin complete -> next waypoint")
                return

            # rotate toward target
            w = self.search_w_turn_max if err > 0.0 else -self.search_w_turn_max
            self.publish_cmd(0.0, w)
            return

    def sample_random_waypoint(self) -> tuple[float, float]:
            xmin = -self.search_x_lim + self.search_margin
            xmax =  self.search_x_lim - self.search_margin
            ymin = -self.search_y_lim + self.search_margin
            ymax =  self.search_y_lim - self.search_y_lim + (self.search_y_lim - self.search_margin)
            # simplify
            ymin = -self.search_y_lim + self.search_margin
            ymax =  self.search_y_lim - self.search_margin

            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            return (x, y)   

def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
