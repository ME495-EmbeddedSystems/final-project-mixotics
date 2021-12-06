#!/usr/bin/env python
""" Creates a unittest node that tests out the step service in the mover node"""
import unittest
import sys
from math import pi
import moveit_commander
from tf_conversions import transformations
from geometry_msgs.msg import Quaternion, Vector3
from final_project_mixotics.srv import Refresh
import rospy
import rospy
import tf2_ros
import moveit_commander
import json
from rospkg import RosPack
import rosbag
from tf_bag import BagTfTransformer
import logging

class TagNode(unittest.TestCase):
    def __init__(self, *args):
        super(TagNode, self).__init__(*args)
        rospy.init_node("tester")

    def test_refresh_service(self):
        rospy.wait_for_service("/update_scene/refresh")
        refresh = rospy.ServiceProxy("/update_scene/refresh", Refresh)
        rospy.sleep(1)
        resp = refresh() # store response

        bag_path = RosPack().get_path("final_project_mixotics") + "/bag/tag_data.bag"
        bag = rosbag.Bag(bag_path)
        bag_transformer = BagTfTransformer(bag)
        first_transform_time = bag_transformer.waitForTransform("world", "ingredient04_tag", start_time=None)
        pos, ang = bag_transformer.lookupTransform("world", "ingredient04_tag", first_transform_time)

        # Construct input
        i = [resp.all_found, pos[0], pos[1], pos[2]]

        # Get updated output
        scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(4)
        p = scene.get_object_poses(["ingredient04"])["ingredient04"]

        # Construct output
        o = [False, p.position.x, p.position.y, p.position.z]

        self.assertEqual(i,o)
    

if __name__ == "__main__":
    import rostest
    rospy.sleep(10)
    rostest.rosrun('final_project_mixotics', "tag_node", TagNode)