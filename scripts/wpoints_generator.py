#!/usr/bin/env python

import rospy
import rospkg
import sys
import argparse
import actionlib
import time 
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from utils import Utils
import visual_tests

class trajectoryGenerator:

    def __init__(self, args, debug_mode):

        filename = args[1] + '/' + args[1] + '_' + args[2] + '.json'

        self.store_name = args[1]
        self.num_trajectory = args[2]
        self.map_source = args[3]
        debug_mode = bool(args[4])

        rospack = rospkg.RosPack()
        self.base_path = rospack.get_path('store_gfk') + '/'

        self.utils = Utils(filename, int(self.map_source))
        self.bridge = CvBridge()

        #creates folders needed to store the final acquisitions
        self.utils.dir_exists( self.base_path + 'acquisitions/' + self.store_name + '/' + self.num_trajectory)

        #load current trajectory and tranform it. full_trajectory is 
        # in the correct reference for the robot 
        self.full_trajectory = self.utils.load_trajectory() #TODO make rospack only used here

        self.map_shift = rospy.get_param("map_shift")
        # self.robot_radius = rospy.get_param("/move_base/global_costmap/robot_radius")
        self.robot_radius = rospy.get_param("robot_radius")
        self.patch_sz = 30 #based on the robot radius (must be divisible by 2)
        self.patch_len = 10 #(must be divisible by 2)
        
        px_robot_radius = self.utils.meters2pixels(self.robot_radius,0)[0]

        self.goal_cnt = 0
        
    
        if debug_mode:
            #performing all the steps to send goal but just for visualization except the map shift
            self.utils.set_map_image() #to check if the given goal is good 
            map_image = self.utils.get_map_image()
            img = self.utils.gray2bgr(map_image)

            #The drawings are the following:
            # at each waypoint of the trajectory we have a blue circle representing the robot size
            # a yellow square showing the area on which we ask if the waypoint is good or needs to be refined
            # arrow (for now only pointing at 0 degrees) to show the robot heading
            #if the point is bad, additional squares and arrows are shown
            goal_cnt = 0
            for position in self.full_trajectory:
                i,j = self.utils.map2image(position[0],position[1])
                   
                new_img_pt = self.utils.find_feasible_point((i,j),self.patch_sz,self.patch_len,goal_cnt)
                robot_pose, object_pose = self.utils.get_object_direction()
                
                img = visual_tests.draw_patch(img,new_img_pt,self.patch_sz,(0,255,0))
                # img = visual_tests.draw_robot(img,new_img_pt,px_robot_radius,(255,0,0))
                # img = visual_tests.draw_arrow(img,new_img_pt,(new_img_pt[0]+25,new_img_pt[1]),(0,255,0))
                #this happens if shelfs are too far
                if robot_pose != 0 and object_pose != 0 :
                    img = visual_tests.draw_arrow(img,robot_pose,object_pose,(255,0,0))
                if( i!=new_img_pt[0] or j != new_img_pt[1] ):
                    img = visual_tests.draw_patch(img,(i,j),self.patch_sz,(0,0,255))
                    # img = visual_tests.draw_robot(img,(i,j),px_robot_radius,(255,0,0))
                    # img = visual_tests.draw_arrow(img,(i,j),(i+25,j),(0,0,255))
                    if robot_pose != 0 and object_pose != 0 :
                        img = visual_tests.draw_arrow(img,robot_pose,object_pose,(255,0,0))
                         
                goal_cnt = goal_cnt + 1
                self.utils.show_img_and_wait_key("recursive", img) 
            
            self.utils.show_img_and_wait_key("Patches",img)
            self.utils.save_image(self.base_path + '/patches.jpg', img)
            # visual_tests.show_good_bad_points(self.full_trajectory, int(self.map_source))
            # visual_tests.check_traj_correspondences(self.full_trajectory, filename, int(self.map_source) ) #TO REVIEW (occhio a quando moltiplico i ratio, da togliere)
            # visual_tests.check_feasibility() #stessa cosa della funzione sopra

            sys.exit(0)
    
        #ROS interface
        #subscribers
        #get cameras info
        #TODO parametrize them into a .yaml file
        self.tl_cam_rgb_info = rospy.wait_for_message("/tl_rgbd_camera/rgb/camera_info", CameraInfo)
        # self.tl_cam_d_info = rospy.wait_for_message("/tl_rgbd_camera/depth/camera_info", CameraInfo)
        self.tr_cam_rgb_info = rospy.wait_for_message("/tr_rgbd_camera/rgb/camera_info", CameraInfo)
        # self.tr_cam_d_info = rospy.wait_for_message("/tr_rgbd_camera/depth/camera_info", CameraInfo)
        self.bl_cam_rgb_info = rospy.wait_for_message("/bl_rgbd_camera/rgb/camera_info", CameraInfo)
        # self.bl_cam_d_info = rospy.wait_for_message("/bl_rgbd_camera/depth/camera_info", CameraInfo)
        self.br_cam_rgb_info = rospy.wait_for_message("/br_rgbd_camera/rgb/camera_info", CameraInfo)
        # self.br_cam_d_info = rospy.wait_for_message("/br_rgbd_camera/depth/camera_info", CameraInfo)
    
        self.tl_rgb_img_sub = rospy.Subscriber("/tl_rgbd_camera/rgb/image_raw",Image, self.tl_rgb_img_cb)
        # self.tl_d_img_sub = rospy.Subscriber("/tl_rgbd_camera/depth/image_raw",Image, self.tl_d_img_cb)
        self.tr_rgb_img_sub = rospy.Subscriber("/tr_rgbd_camera/rgb/image_raw",Image, self.tr_rgb_img_cb)
        # self.tr_d_img_sub = rospy.Subscriber("/tr_rgbd_camera/depth/image_raw",Image, self.tr_d_img_cb)
        self.bl_rgb_img_sub = rospy.Subscriber("/bl_rgbd_camera/rgb/image_raw",Image, self.bl_rgb_img_cb)
        # self.bl_d_img_sub = rospy.Subscriber("/bl_rgbd_camera/depth/image_raw",Image, self.bl_d_img_cb)
        self.br_rgb_img_sub = rospy.Subscriber("/br_rgbd_camera/rgb/image_raw",Image, self.br_rgb_img_cb)
        # self.br_d_img_sub = rospy.Subscriber("/br_rgbd_camera/depth/image_raw",Image, self.br_d_img_cb)

        self.tl_rgb_img_msg = Image()
        # self.tl_d_img_msg = Image()
        self.tr_rgb_img_msg = Image()
        # self.tr_d_img_msg = Image()
        self.bl_rgb_img_msg = Image()
        # self.bl_d_img_msg = Image()
        self.br_rgb_img_msg = Image()
        # self.br_d_img_msg = Image()

        self.pose_sub = rospy.Subscriber("/robot_pose",PoseWithCovarianceStamped, self.pose_cb) #try gazebo ground truth eventually
        self.pose = PoseWithCovarianceStamped()
        self.laser_scan_sub = rospy.Subscriber("/scan",LaserScan, self.laser_cb)
        self.laser_msg = LaserScan()

        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel",Twist, queue_size=10)

        #action for move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        wait = self.client.wait_for_server()
        if not wait:
            rospy.signal_shutdown("Could not find move_base Action Server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0 #not sending orientation, we will rotate to align at the end


    def run(self):

        self.utils.set_map_image() #to check if the given goal is good 
        map_image = self.utils.get_map_image()
        img = self.utils.gray2bgr(map_image)

        # for position in self.full_trajectory:
        #     #this double transformation is required
        #     i,j = self.utils.map2image(position[0],position[1])
        #     new_img_pt = self.utils.find_feasible_point(img,(i,j),self.patch_sz)
        #     new_goal_x, new_goal_y = self.utils.image2map(new_img_pt[0],new_img_pt[1])
        #     x, y = self.utils.shift_goal((new_goal_x,new_goal_y),self.map_shift)

        #     self.send_waypoint(x, y)
        #     self.goal_cnt = self.goal_cnt + 1
                
        #     time.sleep(1.0)
        #     self.capture()

        i,j = self.utils.map2image(1.9 ,8.2)
        new_img_pt = self.utils.find_feasible_point(img,(i,j),self.patch_sz, self.patch_len,self.goal_cnt)
        robot_pose, object_pose = self.utils.get_object_direction()
        x, y = self.utils.image2map(new_img_pt[0],new_img_pt[1])
        # x, y = self.utils.shift_goal((new_goal_x,new_goal_y),self.map_shift)

        self.send_waypoint(x, y)
        self.goal_cnt = self.goal_cnt + 1
            
        time.sleep(1.0)
        self.align()
        time.sleep(1.0)
        self.capture() 
  
        rospy.signal_shutdown("Reached last goal, shutting down wpoint generator node")
        rospy.logdebug("Reached last goal, shutting down wpoint generator node")


    def send_waypoint(self, x, y):

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        #send the current goal
        self.client.send_goal(self.goal,done_cb=self.done_cb,feedback_cb=self.feedback_cb) #it is possible to add the others callbacks directly from here
        
        result = self.client.wait_for_result() #blocking execution until some result comes

        if not result:
            rospy.logerr("Something went wrong with the waypoint number: !", self.goal_cnt)
            rospy.signal_shutdown("Something went wrong with the waypoint number: !", self.goal_cnt)
        else:
            return self.client.get_result()

    def align(self):
        rospy.logdebug("Aligning robot.")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        #get pose and turning direction sign
        curr_yaw = self.pose.pose.pose.orientation.z
        yaw_goal = 0
        dir_sign = 0
        if abs(curr_yaw) < abs(curr_yaw - self.utils.pi()):
           yaw_goal = 0
           dir_sign = -1 if curr_yaw > 0 else 1
        else:
            yaw_goal = self.utils.pi()
            dir_sign = -1 if curr_yaw < 0 else 1

        epsilon = 0.02 #around 1 degree

        msg.angular.z = dir_sign*0.2 #moving very slowly
            
        while not rospy.is_shutdown() and abs(self.pose.pose.pose.orientation.z - yaw_goal) > epsilon:
            self.cmd_vel_pub.publish(msg)
            curr_yaw = self.pose.pose.pose.orientation.z
        
    def capture(self):

        path = self.base_path + 'acquisitions/' + self.store_name + '/' + str(self.num_trajectory) + '/'
        self.utils.dir_exists( path + str(self.goal_cnt) )
        
        try:
            tl_rgb_img = self.bridge.imgmsg_to_cv2(self.tl_rgb_img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # try:
        #     tl_d_img = self.bridge.imgmsg_to_cv2(self.tl_d_img_msg, "mono16")
        # except CvBridgeError as e:
        #     print(e)

        try:
            tr_rgb_img = self.bridge.imgmsg_to_cv2(self.tr_rgb_img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # try:
        #     tr_d_img = self.bridge.imgmsg_to_cv2(self.tr_d_img_msg, "mono16")
        # except CvBridgeError as e:
        #     print(e)

        try:
            bl_rgb_img = self.bridge.imgmsg_to_cv2(self.bl_rgb_img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # try:
        #     bl_d_img = self.bridge.imgmsg_to_cv2(self.bl_d_img_msg, "mono16")
        # except CvBridgeError as e:
        #     print(e)

        try:
            br_rgb_img = self.bridge.imgmsg_to_cv2(self.br_rgb_img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # try:
        #     br_d_img = self.bridge.imgmsg_to_cv2(self.br_d_img_msg, "mono16")
        # except CvBridgeError as e:
        #     print(e)

        
        self.utils.save_image( path + str(self.goal_cnt) + '/top_left_rgb.jpg', tl_rgb_img)
        # self.utils.save_image(path + str(self.goal_cnt) + '/top_left_depth', tl_d_img)
        self.utils.save_image(path + str(self.goal_cnt) + '/top_right_rgb.jpg', tr_rgb_img)
        # self.utils.save_image(path + str(self.goal_cnt) + '/top_right_depth', tr_d_img)
        self.utils.save_image(path + str(self.goal_cnt) + '/bottom_left_rgb.jpg', bl_rgb_img)
        # self.utils.save_image(path + str(self.goal_cnt) + '/bottom_left_depth', bl_d_img)
        self.utils.save_image(path + str(self.goal_cnt) + '/bottom_right_rgb.jpg', br_rgb_img)
        # self.utils.save_image(path + str(self.goal_cnt) + '/bottom_right_depth', br_d_img)

        #save pose
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        yaw = self.pose.pose.pose.orientation.z
        self.utils.save_pose(path + str(self.goal_cnt) + '/pose.yaml',rospy.get_time(),x,y,yaw)
        rospy.logdebug("Saved waypoint number " + str(self.goal_cnt) + " images and pose.")



    def feedback_cb(self,feedback): #returns the robot position
        # rospy.loginfo("Feedback:%s" % str(feedback))
        a = 3

    def done_cb(self, status, result):
        #status gives info on preemptions etc on the goal
        print("status: ", status)
        print("result:" ,result)

    def laser_cb(self,data):
        self.laser_msg = data

    def pose_cb(self,data):
        self.pose = data
    
    def tl_rgb_img_cb(self, data):
        self.tl_rgb_img_msg = data

    def tl_d_img_cb(self, data):
        self.tl_d_img_msg = data

    def tr_rgb_img_cb(self, data):
        self.tr_rgb_img_msg = data

    def tr_d_img_cb(self, data):
        self.tr_d_img_msg = data

    def bl_rgb_img_cb(self, data):
        self.bl_rgb_img_msg = data

    def bl_d_img_cb(self, data):
        self.bl_d_img_msg = data

    def br_rgb_img_cb(self, data):
        self.br_rgb_img_msg = data

    def br_d_img_cb(self, data):
        self.br_d_img_msg = data


def main():
    
    args = rospy.myargv()
    
    if len(args) != 5:
        Utils.print_usage(1)
    rospy.init_node('wpoints_generator', anonymous=True)
    
    robot_navigation = trajectoryGenerator(args, True)
    robot_navigation.run()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
