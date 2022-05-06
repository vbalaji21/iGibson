#!/usr/bin/env python3
import logging
import os

import numpy as np
import rospkg
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from igibson.envs.igibson_env import iGibsonEnv
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from igibson.tasks.social_nav_random_task import SocialNavRandomTask


class SimNode(object):
    def __init__(self):
        rospy.init_node("igibson_sim")
        rospack = rospkg.RosPack()
        path = rospack.get_path("igibson-ros")
        config_filename = os.path.join(path, "fetch.yaml")
        # config_filename = os.path.join(path, "fetch_motion_planning.yaml")

        self.cmdx = 0.0
        self.cmdy = 0.0

        self.image_pub = rospy.Publisher("/gibson_ros/camera/rgb/image", ImageMsg, queue_size=10)
        self.depth_pub = rospy.Publisher("/gibson_ros/camera/depth/image", ImageMsg, queue_size=10)
        self.lidar_pub = rospy.Publisher("/gibson_ros/lidar/points", PointCloud2, queue_size=10)
        self.depth_raw_pub = rospy.Publisher("/gibson_ros/camera/depth/image_raw", ImageMsg, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.gt_pose_pub = rospy.Publisher("/ground_truth_odom", Odometry, queue_size=10)
        self.camera_info_pub = rospy.Publisher("/gibson_ros/camera/depth/camera_info", CameraInfo, queue_size=10)

 
 
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
        self.env.reset()

        self.tp_time = None

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

            if (self.tp_time is None) or (
                (self.tp_time is not None) and ((rospy.Time.now() - self.tp_time).to_sec() > 1.0)
            ):
                scan = obs["scan"]
                lidar_header = Header()
                lidar_header.stamp = now
                lidar_header.frame_id = "scan_link"

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
