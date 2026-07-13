import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from ros_gz_interfaces.msg import Altimeter
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import time
import threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from guppy_msgs.action import LaneFollow
from nav_msgs.msg import Odometry

#MADE THIS FROM THE CPP VERSION, I NEED THE HARDWARE TOPICS TO SEE IF THIS WILL WORK IRL

def clampd(v,lo,hi):
    return max(lo,min(hi,v))

def clampi(v,lo,hi):
    return int(max(lo,min(hi,v)))

def quat_to_euler_xyz(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Pole:
    __slots__ = ("cx","cy","area","bbox")

    def __init__(self,cx,cy,area,bbox):
        self.cx = cx
        self.cy = cy
        self.area = area
        self.bbox = bbox

class LaneNavigator(Node):
    def __init__(self):
        super().__init__("lane_navigator_py")
        self.declare_parameter("enabled",False)
        self.declare_parameter("enable_debug",True)
        self.declare_parameter("debug_show_every_n_frames",2)

        self.declare_parameter("target_depth",1.0)
        self.declare_parameter("depth_increases_down",True)
        self.declare_parameter("kp_depth",4.0)
        self.declare_parameter("ki_depth",0.1)
        self.declare_parameter("kd_depth",2.0)
        self.declare_parameter("max_heave_cmd",2.0)

        self.declare_parameter("kp_roll",3.0)
        self.declare_parameter("kd_roll",0.8)
        self.declare_parameter("kp_pitch",3.0)
        self.declare_parameter("kd_pitch",0.8)
        self.declare_parameter("max_att_cmd",2.0)

        self.declare_parameter("kp_yaw_px",0.006)
        self.declare_parameter("kd_yaw",0.05)
        self.declare_parameter("k_lane_theta",0.6)
        self.declare_parameter("max_yaw",1.2)
        self.declare_parameter("search_yaw",0.35)
        self.declare_parameter("lane_lookahead_frac",0.65)

        self.declare_parameter("forward_speed",0.35)
        self.declare_parameter("forward_speed_search",0.05)

        self.declare_parameter("hsv_red1_low",[0,100,100])
        self.declare_parameter("hsv_red1_high",[10,255,255])
        self.declare_parameter("hsv_red2_low",[170,100,100])
        self.declare_parameter("hsv_red2_high",[180,255,255])
        self.declare_parameter("hsv_white_low",[0,0,55])
        self.declare_parameter("hsv_white_high",[179,45,255])

        self.declare_parameter("min_contour_area",300)
        self.declare_parameter("min_aspect",2.0)
        self.declare_parameter("blur_ksize",5)
        self.declare_parameter("morph_ksize",5)
        self.declare_parameter("max_poles_each_color",6)
        self.declare_parameter("publish_debugimages", True)

        p = self.get_parameter
        self.debug_ = p("enable_debug").value
        self.publish_debug = p("publish_debugimages").value
        self.debug_every_n_ = max(1,int(p("debug_show_every_n_frames").value))
        
        self.target_depth_ = p("target_depth").value
        self.depth_incrases_down_= p("depth_increases_down").value
        # self.kp_depth_ = p("kp_depth").value
        # self.kd_depth = p("max_heave_cmd").value
        self.kp_depth_ = p("kp_depth").value
        self.ki_depth_ = p("ki_depth").value
        self.kd_depth_ = p("kd_depth").value
        self.max_heave_cmd_ = p("max_heave_cmd").value

        self.kp_roll_ = p("kp_roll").value
        self.kd_roll_ = p("kd_roll").value
        self.kp_pitch = p("kp_pitch").value
        self.kd_pitch = p("kd_pitch").value
        self.max_att_cmd_ = p("max_att_cmd").value

        self.kp_yaw_px_ = p("kp_yaw_px").value
        self.kd_yaw_ = p("kd_yaw").value
        self.k_lane_theta_ = p("k_lane_theta").value
        self.max_yaw_ = p("max_yaw").value
        self.search_yaw_ = p("search_yaw").value
        self.lane_lookahead_frac_ = p("lane_lookahead_frac").value
        self.forward_speed_ = p("forward_speed").value
        self.forward_speed_search_ = p("forward_speed_search").value
        self.min_area_ = p("min_contour_area").value
        self.min_aspect_ = p("min_aspect").value
        self.blur_ksize_ = p("blur_ksize").value
        self.morph_ksize_ = p("morph_ksize").value
        self.max_poles_each_color_ = p("max_poles_each_color").value

        self.red1_low_ = np.array(p("hsv_red1_low").value, dtype=np.uint8)
        self.red1_high_ = np.array(p("hsv_red1_high").value, dtype=np.uint8)
        self.red2_low_ = np.array(p("hsv_red2_low").value, dtype=np.uint8)
        self.red2_high_ = np.array(p("hsv_red2_high").value, dtype=np.uint8)
        self.white_low_ = np.array(p("hsv_white_low").value, dtype=np.uint8)
        self.white_high_ = np.array(p("hsv_white_high").value, dtype=np.uint8)

        # self.bridge_ = CvBridge()
        # self.was_enabled_ = False

        self.bridge_ = CvBridge()

        self._goal_active = False
        self._was_active = False
        self._goal_lock = threading.Lock()
        self._current_lost_timeout = 2.0
        self._last_lane_ok = False
        self._last_x_center = 0.0
        
        self.image_ = None
        self.have_image_ = False
        self.depth_ = 0.0
        self.have_depth_ = False
        self.roll_ = self.pitch_ = self.yaw_ = 0.0
        self.roll_rate_ = self.pitch_rate_ = self.yaw_rate_ = 0.0
        self.have_imu_ = False

        self.integral_depth_ = 0.0
        self.prev_depth_error_ = 0.0

        self.kf_inited = False
        self.kf_x_ = 0.0
        self.kf_v_ = 0.0
        self.P00_ = self.P11_ = 1.0
        self.P01_ = self.P10_ = 0.0
        self.q_pos_ = 50.0
        self.q_vel_ = 200.0
        self.r_meas_ = 400.0

        # self.frame_count_ = 0
        # self.prev_time_ = self.get_clock().now()

        # self.cmd_pub_ = self.create_publisher(Twist,"/cmd_vel",10)

        self.frame_count_ = 0
        self.prev_time_ = self.get_clock().now()
        # self._cb_group = ReentrantCallbackGroup()
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            LaneFollow,
            "lane_follow",
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group = self._cb_group,
        )
        self.cmd_pub_ = self.create_publisher(Twist, "/cmd_vel/task", 10)
        self.status_pub_ = self.create_publisher(Bool,"~/lane_found",10)

        self.create_subscription(
            Image, "/cube/image_raw", self.image_cb, qos_profile_sensor_data
        )
        # self.create_subscription(
        #     Altimeter, "/altimeter", self.alt_cb, qos_profile_sensor_data
        # )
        self.create_subscription(
            Odometry, "/odometry/filtered", self.alt_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/odometry/filtered", self.imu_cb, qos_profile_sensor_data
        )

        if self.publish_debug:
            self.dbg_rgb_pub_ = self.create_publisher(Image, "/lane_debug/rgb", 1)
            self.dbg_red_pub_ = self.create_publisher(Image, "/lane_debug/red_mask", 1)
            self.dbg_white_pub_ = self.create_publisher(Image, "/lane_debug/white_mask", 1)
            self.dbg_overlay_pub_ = self.create_publisher(Image, "/lane_debug/overlay", 1)

        # self.timer_ = self.create_timer(0.01,self.control_loop)
        self.timer_ = self.create_timer(0.01, self.control_loop, callback_group=self._cb_group)

        self.get_logger().info(
            f"lane_navigator_py running (100 Hz). "
            f"publish_debugimages={self.publish_debug} debug_every_n={self.debug_every_n_}"
        )

    def image_cb(self,msg: Image):
        try:
            self.image_ = self.bridge_.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            self.have_image_ = True
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}", throttle_duration_sec=2.0)

    def goal_callback(self,goal_request):
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warn("LaneFollow goal rejected: already active")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT
            
            
    def cancel_callback(self,goal_handle):
        self.get_logger().info("LaneFollow cancel requested")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self,goal_handle):
        self._current_lost_timeout = goal_handle.request.lost_timeout
        with self._goal_lock:
            self._goal_active = True
        
        feedback = LaneFollow.Feedback()
        result = LaneFollow.Result()
        last_seen = time.monotonic() 
        rate_hz = 10.0

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    return result
            
                feedback.lane_found = self._last_lane_ok
                feedback.x_center = self._last_x_center
                feedback.depth = self.depth_
                goal_handle.publish_feedback(feedback)
                if self._last_lane_ok:
                    last_seen = time.monotonic()
                elif time.monotonic() - last_seen > self._current_lost_timeout:
                    goal_handle.abort()
                    result.success = False
                    result.message = "lane lost past timeout"
                    return result
                
                time.sleep(1.0 /rate_hz)
            goal_handle.abort()
            result.success = False
            result.message = "rclpy shutdown"
            return result
        finally:
            with self._goal_lock:
                self._goal_active = False


    def alt_cb(self,msg: Altimeter):
        # z = float(msg.vertical_position)
        z = float(msg.pose.pose.position.z)
        self.depth_ = z if self.depth_incrases_down_ else -z
        self.have_depth_ = True

    #XYZ NEEDS TO BE SWAPPED TO WORK WITH GUPPY XYZ
    def imu_cb(self,msg:Imu):
        q = msg.orientation
        self.roll_, self.pitch_,self.yaw_= quat_to_euler_xyz(q.x, q.y, q.z, q.w)
        self.roll_rate_ = msg.angular_velocity.x
        self.pitch_rate_ = msg.angular_velocity.y
        self.yaw_rate_ = msg.angular_velocity.z
        self.have_imu_ = True

    def pole_centroids(self,mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poles = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area_:
                continue
            x,y,w,h = cv2.boundingRect(c)
            if w <= 0:
                continue
            aspect = h / float(w)
            if aspect < self.min_aspect_:
                continue
            M = cv2.moments(c)
            if abs(M["m00"]) < 1e-6:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            poles.append(Pole(cx, cy, area, (x, y, w, h)))
        poles.sort(key=lambda p: (-p.cy, -p.area))
        return poles[: self.max_poles_each_color_]
    
    def pick_best_pair(self,reds,whites):
        if not reds or not whites:
            return None, None
        NR = min(3,len(reds))
        NW = min(3,len(whites))
        best_score = float("inf")
        best_r = best_w = None
        for i in range(NR):
            for j in range(NW):
                r,w = reds[i],whites[j]
                if r.cx >= w.cx:
                    continue
                dy = abs(r.cy - w.cy)
                area_bonus = 0.0005 * (r.area + w.area)
                score = dy - area_bonus
                if score < best_score:
                    best_score = score
                    best_r , best_w = r,w
        return best_r , best_w
    
    @staticmethod
    def fit_line(poles):
        Syy = Sy = S1 = Sxy = Sx = 0.0
        for p in poles:
            y,x = float(p.cy),float(p.cx)
            Syy += y*y
            Sy += y
            S1 += 1.0
            Sxy += x * y
            Sx += x
        det = Syy * S1 - Sy * Sy
        if abs(det) < 1e-9:
            return 0.0, Sx / max(1.0,S1)
        a = (Sxy * S1 - Sx * Sy) / det
        b = (Syy * Sx - Sy * Sxy) / det
        return a,b
    
    def detect_lane(self,bgr):
        img = bgr
        if self.blur_ksize_ > 1:
            k = self.blur_ksize_ if self.blur_ksize_ % 2 == 1 else self.blur_ksize_ + 1
            img = cv2.GaussianBlur(img,(k,k),0.0)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, self.red1_low_,self.red1_high_)
        red2 = cv2.inRange(hsv,self.red2_low_,self.red2_high_)
        red_mask = cv2.bitwise_or(red1, red2)
        white_mask = cv2.inRange(hsv, self.white_low_, self.white_high_)

        mk = self.morph_ksize_ if self.morph_ksize_ % 2 == 1 else self.morph_ksize_ + 1
        mk = max(1, mk)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mk, mk))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        overlay = bgr.copy()
        overlay[red_mask > 0] = (0,0,255)
        overlay[white_mask > 0] = (255,255,255)
        
        red_poles = self.pole_centroids(red_mask)
        white_poles = self.pole_centroids(white_mask)

        for p in red_poles:
            x,y,w,h = p.bbox
            cv2.rectangle(overlay, (x,y), (x+w,y+h),(0,0,255),2)
            cv2.circle(overlay, (p.cx,p.cy),3, (200,200,200),-1)

        best_r , best_w = self.pick_best_pair(red_poles,white_poles)
        if best_r is None:
            return False, None, None, red_mask, white_mask, overlay
        
        mid = ((best_r.cx + best_w.cx) // 2, (best_r.cy + best_w.cy) // 2)
        cv2.circle(overlay,mid,8,(0,255,255),-1)
        cv2.line(overlay, (best_r.cx, best_r.cy), (best_w.cx , best_w.cy), (0, 255, 255), 2)
        x_center = float(mid[0])
        lane_theta =0.0

        if len(red_poles) < 2 or len(white_poles) < 2:
            return True, x_center, lane_theta, red_mask, white_mask, overlay
        
        H,W = bgr.shape[:2]
        y_look = clampi(self.lane_lookahead_frac_ * H,0,H-1)
        ar,br = self.fit_line(red_poles)
        aw,bw = self.fit_line(white_poles)
        x_left = ar*y_look + br
        x_right = aw * y_look + bw

        if x_left >= x_right:
            return True, x_center, lane_theta, red_mask, white_mask, overlay
        
        x_center = clampd(0.5 * (x_left + x_right), 0.0, float(W-1))
        a_center = 0.5 * (ar + aw)
        lane_theta = math.atan(a_center)

        def draw_xy(a,b,color):
            y0,y1 = 0, H-1
            x0 = clampi(a*y0 + b,-2000,2000)
            x1 = clampi(a*y1 + b, -2000,2000)
            cv2.line(overlay,(x0,y0),(x1,y1),color,2)

        draw_xy(ar,br,(0,0,255))
        draw_xy(aw,bw,(220,220,220))
        cv2.circle(overlay,(int(x_center),y_look),6,(0,255,0),-1)
        cv2.line(overlay,(W // 2, 0), (W // 2 , H-1), (0,255,255),1)
        return True, x_center, lane_theta, red_mask, white_mask, overlay
    
    def kf_reset(self,x0):
        self.kf_inited = True
        self.kf_x_ = x0
        self.kf_v_ = 0.0
        self.P00_, self.P01_ = 500.0, 0.0
        self.P10_, self.P11_ = 0.0, 500.0

    def kf_predict(self,dt):
        self.kf_x_ = self.kf_x_ + dt * self.kf_v_
        A00, A01 = 1.0,dt
        A10, A11 = 0.0,1.0
        P00, P01, P10, P11 = self.P00_, self.P01_, self.P10_, self.P11_
        AP00 = A00 * P00 + A01 * P10
        AP01 = A00 * P01 + A01 * P11
        AP10 = A10 * P00 + A11 * P10
        AP11 = A10 * P01 + A11 * P11

        # newP00 = AP00 * AP00 + AP01 * A01
        newP00 = AP00 * A00 + AP01 * A01
        newP01 = AP00 * A10 + AP01 * A11
        newP10 = AP10 * A00 + AP11 * A01
        newP11 = AP10 * A10 + AP11 * A11

        newP00 += self.q_pos_ * dt * dt
        newP11 += self.q_vel_*dt
        self.P00_, self.P01_, self.P10_, self.P11_ = newP00, newP01, newP10, newP11

    def kf_update(self,z_meas):
        y = z_meas - self.kf_x_
        S = self.P00_ + self.r_meas_
        if S < 1e-9:
            return
        K0 = self.P00_ / S
        K1 = self.P10_ / S

        self.kf_x_ = self.kf_x_ + K0 * y
        self.kf_v_ = self.kf_v_ + K1 * y
        P00, P01, P10, P11 = self.P00_, self.P01_, self.P10_, self.P11_
        self.P00_ = (1.0 - K0) * P00
        self.P01_ = (1.0 - K0) * P01
        self.P10_ = P10 - K1 * P00
        self.P11_ = P11 - K1 * P01

    def publish_dbg_images(self,rgb, red_mask, white_mask, overlay):
        if not self.publish_debug:
            return
        stamp = self.get_clock().now().to_msg()
        def to_bgr(img):
            m = self.bridge_.cv2_to_imgmsg(img,encoding ="bgr8")
            m.header.stamp = stamp
            m.header.frame_id = "camera_link" #THIS NEEDS TO BE CHANGED BECAUSE IDK WHAT THE FRAME FOR THE CAMERA IS
            return m
        
        def to_mono(img):
            m = self.bridge_.cv2_to_imgmsg(img,encoding="mono8")
            m.header.stamp = stamp
            m.header.frame_id = "camera_link"#THIS ALSO NEEDS TO BE CHANGED BECAUSE IDK WHAT THE FRAME FOR THE CAMERA IS
            return m
        
        self.dbg_rgb_pub_.publish(to_bgr(rgb))
        self.dbg_red_pub_.publish(to_mono(red_mask))
        self.dbg_white_pub_.publish(to_mono(white_mask))
        self.dbg_overlay_pub_.publish(to_bgr(overlay))

    def control_loop(self):
        # enabled = self.get_parameter("enabled").value
        # if not enabled:
        #     if self.was_enabled_:
        #         self.cmd_pub_.publish(Twist())
        #         self.was_enabled_ = False
        #     return
        # self.was_enabled_ = True
        if not self._goal_active:
            if self._was_active:
                self.cmd_pub_.publish(Twist())
                self._was_active = False
            return
        self._was_active = True

        #UNCOMMENT THIS WHEN YOU HAVE IMU/DEPTH SENSOR/ALTIMETER (STILL USES GAZEBO TOPICS)
        # if not (self.have_image_ and self.have_depth_ and self.have_imu_):
        #     return

        #SET IT TO THIS WHEN TESTING WITH ONLY CAMERA/WEBCAM
        if not self.have_image_:
            return
    
        now = self.get_clock().now()
        dt = (now - self.prev_time_).nanoseconds * 1e-9
        if dt <= 1e-6:
            return
        self.prev_time_ = now

        if self.kf_inited:
            self.kf_predict(dt)

        depth_error = self.target_depth_ - self.depth_
        self.integral_depth_ += depth_error * dt
        derivative_depth = (depth_error - self.prev_depth_error_) / dt
        self.prev_depth_error_ = depth_error

        # heave = (self.kp_depth_ * depth_error + self.ki_depth * self.integral_depth_ + self.kd_depth * derivative_depth)
        # heave = clampd(heave, -self.max_heave_cmd, self.max_heave_cmd)
        heave = (self.kp_depth_ * depth_error + self.ki_depth_ * self.integral_depth_ + self.kd_depth_ * derivative_depth)
        heave = clampd(heave, -self.max_heave_cmd_, self.max_heave_cmd_)

        roll_cmd = self.kp_roll_ * (0.0 - self.roll_)+ self.kd_roll_ * (0.0 - self.roll_rate_)
        pitch_cmd = self.kp_pitch * (0.0 - self.pitch_) + self.kd_pitch * (0.0 - self.pitch_rate_)
        roll_cmd = clampd(roll_cmd, -self.max_att_cmd_, self.max_att_cmd_)
        pitch_cmd = clampd(pitch_cmd, -self.max_att_cmd_, self.max_att_cmd_)

        # lane_ok, x_center, lane_theta, red_mask, white_mask, overlay = self.detect_lane(self.image_)
        lane_ok, x_center, lane_theta, red_mask, white_mask, overlay = self.detect_lane(self.image_)
        self._last_lane_ok = lane_ok
        self._last_x_center = x_center if lane_ok else self._last_x_center


        if lane_ok:
            if not self.kf_inited:
                self.kf_reset(x_center)
            else:
                self.kf_update(x_center)
            x_center = self.kf_x_
        else:
            y_draw = int(0.65 * self.image_.shape[0])
            cv2.circle(overlay,(int(self.kf_x_), y_draw), 6, (255, 255, 0), -1)
        
        self.frame_count_ += 1

        if self.publish_debug and (self.frame_count_ % self.debug_every_n_ == 0):
            self.publish_dbg_images(self.image_,red_mask,white_mask,overlay)

        self.status_pub_.publish(Bool(data=lane_ok))
        cmd = Twist()
        cmd.linear.z = heave
        cmd.angular.x = roll_cmd
        cmd.angular.y = pitch_cmd
        w = self.image_.shape[1]
        img_cx = 0.5*w

        if not lane_ok:
            cmd.linear.x = self.forward_speed_search_
            cmd.angular.z = clampd(self.search_yaw_ + (-self.kd_yaw_ * self.yaw_rate_), -self.max_yaw_, self.max_yaw_)
            self.cmd_pub_.publish(cmd)
            if self.debug_ and (self.frame_count_ % self.debug_every_n_ == 0):
                self.get_logger().info(f"[SEARCH] x={cmd.linear.x:.2f} z={cmd.linear.z:+.2f} " f"r={cmd.angular.x:+.2f} p={cmd.angular.y:+.2f} yaw={cmd.angular.z:+.2f} " f"depth={self.depth_:.2f}")
            return

        error_px = img_cx - x_center
        yaw_track = self.kp_yaw_px_ * error_px
        yaw_ff = -self.k_lane_theta_ * lane_theta
        yaw_damp = -self.kd_yaw_ * self.yaw_rate_
        yaw_cmd = clampd(yaw_track + yaw_ff + yaw_damp, -self.max_yaw_, self.max_yaw_)
        cmd.linear.x = self.forward_speed_
        cmd.angular.z = yaw_cmd
        self.cmd_pub_.publish(cmd)
        if self.debug_ and (self.frame_count_ % self.debug_every_n_ == 0):
            self.get_logger().info(
                f"[LANE] x={cmd.linear.x:.2f} z={cmd.linear.z:+.2f} "
                f"r={cmd.angular.x:+.2f} p={cmd.angular.y:+.2f} yaw={cmd.angular.z:+.2f} "
                f"err_px={error_px:+.1f} theta={lane_theta:+.3f} depth={self.depth_:.2f}"
            )
def main(args =None):
    # rclpy.init(args=args)
    # node = LaneNavigator()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
    rclpy.init(args = args)
    node = LaneNavigator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()










