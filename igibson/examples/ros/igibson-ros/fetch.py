#!/usr/bin/env python3
import logging
import os
from pickletools import uint8

import numpy as np
import rospkg
import rospy
import sensor_msgs.msg as sensor_msgs
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rospy.impl.registration import Registration, get_registration_listeners, get_topic_manager, set_topic_manager
from rospy.impl.statistics import SubscriberStatisticsLogger
from rospy.impl.tcpros import DEFAULT_BUFF_SIZE, get_tcpros_handler
from rospy.impl.tcpros_pubsub import QueuedConnection
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import Joy, PointCloud, PointCloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from igibson.envs.igibson_env import iGibsonEnv
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from igibson.tasks.social_nav_random_task import SocialNavRandomTask
import math
import pybullet as p

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]

    header = Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), #7
        row_step=(itemsize * 3 * points.shape[0]), #7
        data=data
    )



class SimNode(object):
    def __init__(self):
        rospy.init_node("igibson_sim")
        rospack = rospkg.RosPack()
        path = rospack.get_path("igibson-ros")
        config_filename = os.path.join(path, "fetch.yaml")
        # config_filename = os.path.join(path, "fetch_motion_planning.yaml")

        self.cmdx = 0.0
        self.cmdy = 0.0

        self.human_vel = {
            "lin_vel": 0.0,
            "ang_vel": 0.0,
        }

        self.yaw = 0.0

        self.image_pub = rospy.Publisher("/gibson_ros/camera/rgb/image", ImageMsg, queue_size=10)
        self.depth_pub = rospy.Publisher("/gibson_ros/camera/depth/image", ImageMsg, queue_size=10)
        self.lidar_pub = rospy.Publisher("/gibson_ros/lidar/points", PointCloud2, queue_size=10)
        self.depth_raw_pub = rospy.Publisher("/gibson_ros/camera/depth/image_raw", ImageMsg, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.gt_pose_pub = rospy.Publisher("/ground_truth_odom", Odometry, queue_size=10)
        self.camera_info_pub = rospy.Publisher("/gibson_ros/camera/depth/camera_info", CameraInfo, queue_size=10)

        #TODO: Add CoHAN ROS bridge to send pedestrian position and twists  
        rospy.Subscriber("/cmd_vel", Twist, self.human_vel_callback) 


        #Add a publisher to publish the human torso pose and twist to ROS
        self.tracked_agents_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=10) 
 
        # Add humans for other planners
        self.identify_humans_pub = rospy.Publisher("/gibson_ros/lidar/points_test", PointCloud2, queue_size=1)
        self.p  = Point32()

 
 
#TODO: Add CoHAN ROS bridge to send pedestrian position and twists  
 
#         tracked_agents = TrackedAgents()
#         self.num_ped = 4 #SocialNavRandomTask(self).read_num_pedestrians(self)
#         for agent_id in range(1,self.num_ped+1):  #TODO:fins number of humans 
# #            if self.ns == "human"+str(agent_id):
# #                continue
#             agent_segment = TrackedSegment()
#             self.Segment_Type = TrackedSegmentType.TORSO
#             agent_segment.type = self.Segment_Type
#             self.task = SocialNavRandomTask(self)
#             agent_segment.pose.pose, agent_segment.twist.twist = self.task.read_ped_next_pos(self)    #msg[agent_id-1].pose.pose         #TODO:data of human
#             #agent_segment.twist.twist  #= msg[agent_id-1].twist.twist     #TODO:twist of human
#             tracked_agent = TrackedAgent()     
#             tracked_agent.type = AgentType.HUMAN
#             tracked_agent.name = "human"+str(agent_id)
#             tracked_agent.segments.append(agent_segment)
#             tracked_agents.agents.append(tracked_agent)
# #        if(tracked_agents.agents):
# #            self.agents = tracked_agents
# #            self.sig_1 = True

#         #Add a publisher to publish the human torso pose and twist to ROS
#         self.pedestrian_ros_bridge = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=10)




        rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.cmd_callback)
        rospy.Subscriber("/reset_pose", PoseStamped, self.tp_robot_callback)

        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        self.env = iGibsonEnv(
            config_file=config_filename,mode="gui_non_interactive", use_pb_gui=True, action_timestep=1 / 30.0
        )  # assume a 30Hz simulation

        self.num_hum = self.env.task.num_pedestrians
        self.ns = None
    #    self.tracked_agents_pub = [].pose.pose.position.
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.sig_1 = False
        self.sig_2 = False

        self.env.reset()

        self.tp_time = None

    def human_vel_callback(self, msg):
        """Get the velocity from joystick"""
        # Person meshes are offset by 90 by default

        self.yaw = self.env.task.pedestrians[-1].get_yaw()


        #msg.linear.y * math.cos(self.yaw), -msg.linear.x * math.sin(self.yaw)
        self.human_vel["lin_vel"] = [msg.linear.x * math.cos(self.yaw), msg.linear.x * math.sin(self.yaw), 0]     # linear velocity along X and Y # msg.linear.y , -msg.linear.x 
        self.human_vel["ang_vel"] = [msg.angular.x, msg.angular.y , msg.angular.z]    # angular velocity along Y

    def run(self):
        while not rospy.is_shutdown():
            action = np.zeros(11)
            # [self.cmdx, self.cmdy]
            action[0] = self.cmdx
            action[1] = self.cmdy
            obs, _, _, _ = self.env.step(action)
            #print("in step: ",self.cmdx,self.cmdy)
            rgb = (obs["rgb"] * 255).astype(np.uint8)
            normalized_depth = obs["depth"].astype(np.float32)
            depth = normalized_depth * self.env.sensors["vision"].depth_high
            depth_raw_image = (obs["depth"] * 1000).astype(np.uint16)

            image_message = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            depth_message = self.bridge.cv2_to_imgmsg(depth, encoding="passthrough")
            depth_raw_message = self.bridge.cv2_to_imgmsg(depth_raw_image, encoding="passthrough")

            now = rospy.Time.now()

            image_message.header.stamp = now
            depth_message.header.stamp = now
            depth_raw_message.header.stamp = now
            image_message.header.frame_id = "camera_depth_optical_frame"
            depth_message.header.frame_id = "camera_depth_optical_frame"
            depth_raw_message.header.frame_id = "camera_depth_optical_frame"

            self.image_pub.publish(image_message)
            self.depth_pub.publish(depth_message)
            self.depth_raw_pub.publish(depth_raw_message)

            msg = CameraInfo(
                height=256,
                width=256,
                distortion_model="plumb_bob",
                D=[0.0, 0.0, 0.0, 0.0, 0.0],
                K=[128, 0.0, 128, 0.0, 128, 128, 0.0, 0.0, 1.0],
                R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                P=[128, 0.0, 128, 0.0, 0.0, 128, 128, 0.0, 0.0, 0.0, 1.0, 0.0],
            )
            msg.header.stamp = now
            msg.header.frame_id = "camera_depth_optical_frame"
            self.camera_info_pub.publish(msg)

            # tracked_agents = TrackedAgents()
            self.num_hum = self.env.task.num_pedestrians
            tracked_agents = TrackedAgents()
            self.Segment_Type = TrackedSegmentType.TORSO
            #self.env.task.pedestrians[-1].set_velocity(self.human_vel["lin_vel"], self.human_vel["ang_vel"]) # Only one human

#            for i in range(3):

                # print("position out", pos)
                # print("orientation out", orn)
            for agent_id in range(1,self.num_hum+1):
                # print("agent_id", agent_id)
                # print("num hum", self.num_hum)

                linear, angular = self.env.task.pedestrians[-1].get_velocity(agent_id-1)

                self.p.x    = self.env.task.current_pos[0]
                self.p.y    = self.env.task.current_pos[1]
                self.p.z    = 0 # self.env.task.current_pos[2]

                pos,orn =  self.env.task.pedestrians[-1].get_base_pos_and_orientation(agent_id-1)
                euler_orientation = p.getEulerFromQuaternion(orn)



                agent_segment = TrackedSegment()
                agent_segment.type = self.Segment_Type
                agent_segment.pose.pose.position.x = pos[0]  # self.env.task.current_pos[0]
                agent_segment.pose.pose.position.y = pos[1]  # self.env.task.current_pos[1]
                agent_segment.pose.pose.position.z = pos[2]  # self.env.task.current_pos[2]
                quat = quaternion_from_euler(0,0,euler_orientation[2]-1.57)

                print("old angles", self.env.task.orientation)
                angles = euler_from_quaternion(orn)
                print("new angles", angles)    
                print(" old orientation", quat)            
                print("orientation", orn)

                agent_segment.pose.pose.orientation.x = quat[0]
                agent_segment.pose.pose.orientation.y = quat[1]
                agent_segment.pose.pose.orientation.z = quat[2]
                agent_segment.pose.pose.orientation.w = quat[3]
                agent_segment.twist.twist.linear.x = linear[0] # self.env.task.desired_vel[0]
                agent_segment.twist.twist.linear.y = linear[1] #self.env.task.desired_vel[1]
                agent_segment.twist.twist.angular.z = angular[2] #self.env.task.desired_vel[2]

                tracked_agent = TrackedAgent()
                tracked_agent.type = AgentType.HUMAN
                tracked_agent.name = "human"+str(agent_id)
                tracked_agent.track_id = agent_id
                tracked_agent.segments.append(agent_segment)
                tracked_agents.agents.append(tracked_agent)
            if(tracked_agents.agents):
                self.agents = tracked_agents
                self.agents.header.stamp = rospy.Time.now()
                self.agents.header.frame_id = "map"
                # print("\033[2;31;43m message type of orient \n", type(agent_segment.pose.pose.orientation))
                # print("\033[2;31;43m message data of orient \n", agent_segment.pose.pose.orientation)
                self.tracked_agents_pub.publish(tracked_agents)





# Adding human to lidar for normal planners
















            if (self.tp_time is None) or (
                (self.tp_time is not None) and ((rospy.Time.now() - self.tp_time).to_sec() > 1.0)
            ):
                scan = obs["scan"]
                lidar_header = Header()
                lidar_header.stamp = now
                lidar_header.frame_id = "laser_link"

                laser_linear_range = self.env.sensors["scan_occ"].laser_linear_range
                laser_angular_range = self.env.sensors["scan_occ"].laser_angular_range
                min_laser_dist = self.env.sensors["scan_occ"].min_laser_dist
                n_horizontal_rays = self.env.sensors["scan_occ"].n_horizontal_rays

                laser_angular_half_range = laser_angular_range / 2.0
                angle = np.arange(
                    -np.radians(laser_angular_half_range),
                    np.radians(laser_angular_half_range),
                    np.radians(laser_angular_range) / n_horizontal_rays,
                )
                unit_vector_laser = np.array([[np.cos(ang), np.sin(ang), 0.0] for ang in angle])
                lidar_points = unit_vector_laser * (scan * (laser_linear_range - min_laser_dist) + min_laser_dist)

                lidar_message = pc2.create_cloud_xyz32(lidar_header, lidar_points.tolist())
                self.lidar_pub.publish(lidar_message)

            # Odometry
            odom = [
                np.array(self.env.robots[0].get_position()) - np.array(self.env.task.initial_pos),
                np.array(self.env.robots[0].get_rpy()) - np.array(self.env.task.initial_orn),
            ]

            self.br.sendTransform(
                (odom[0][0], odom[0][1], 0),
                tf.transformations.quaternion_from_euler(0, 0, odom[-1][-1]),
                rospy.Time.now(),
                "base_link",
                "odom",
            )
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = odom[0][0]
            odom_msg.pose.pose.position.y = odom[0][1]
            (
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w,
            ) = tf.transformations.quaternion_from_euler(0, 0, odom[-1][-1])

            odom_msg.twist.twist.linear.x = self.cmdx #(self.cmdx + self.cmdy) * 5
            odom_msg.twist.twist.angular.z = self.cmdy #(self.cmdy - self.cmdx) * 5 * 8.695652173913043
            self.odom_pub.publish(odom_msg)

            # Ground truth pose
            gt_pose_msg = Odometry()
            gt_pose_msg.header.stamp = rospy.Time.now()
            gt_pose_msg.header.frame_id = "ground_truth_odom"
            gt_pose_msg.child_frame_id = "base_link"

            xyz = self.env.robots[0].get_position()
            rpy = self.env.robots[0].get_rpy()

            gt_pose_msg.pose.pose.position.x = xyz[0]
            gt_pose_msg.pose.pose.position.y = xyz[1]
            gt_pose_msg.pose.pose.position.z = xyz[2]
            (
                gt_pose_msg.pose.pose.orientation.x,
                gt_pose_msg.pose.pose.orientation.y,
                gt_pose_msg.pose.pose.orientation.z,
                gt_pose_msg.pose.pose.orientation.w,
            ) = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

            gt_pose_msg.twist.twist.linear.x = self.cmdx  #(self.cmdx + self.cmdy) * 5
            gt_pose_msg.twist.twist.angular.z = self.cmdy #(self.cmdy - self.cmdx) * 5 * 8.695652173913043
            self.gt_pose_pub.publish(gt_pose_msg)

    def cmd_callback(self, data):
        self.cmdx =  data.linear.x # / 10.0 - data.angular.z / (10 * 8.695652173913043)
        self.cmdy =  data.angular.z #/ 10.0 + data.angular.z / (10 * 8.695652173913043)
        # print(data)

    def tp_robot_callback(self, data):
        rospy.loginfo("Teleporting robot")
        position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        orientation = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]
        self.env.robots[0].reset_new_pose(position, orientation)
        self.tp_time = rospy.Time.now()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    node = SimNode()
    node.run()
