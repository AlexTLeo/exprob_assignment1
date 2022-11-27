#!/usr/bin/env python

"""
.. module::robot_state
   :platform: Ubuntu 20.04
   :snyopsis: This module manages information between nodes regarding the robot state.

.. moduleauthor:: Alex Thanaphon Leonardi <thanaphon.leonardi@gmail.com>

This module interfaces the robot's state with the other nodes. Specifically, it
shares information regarding the robot pose, the next goal and the battery. It
also accepts pose updates to update the current pose in the ontology, and accepts
requests to change the battery mode to charging or discharging.

ROS Parameters:
  **/robot_state/battery_tick** how frequently to change the battery\n
  **/robot_state/battery_charge_per_tick** how much charge per tick\n
  **/robot_state/battery_discharge_per_tick** how much discharge per tick\n
  **/robot_state/battery_publish** rate at which the battery level is published

Publishes to:
  **/state/battery_level** broadcasts the robot battery level.\n
  **/state/map/get_goal** broadcasts the next goal.\n
  **/state/get_pose** broadcasts the current pose (location).\n
  **/owl_interface/set_pose** sends the requested pose setting to :mod:`owl_interface`.

Subscribes to:
  **/owl_interface/goal** gets the next goal from :mod:`owl_interface`.\n
  **/owl_interface/get_pose** gets the current pose from :mod:`owl_interface`.\n
  **/state/set_pose** receive incoming pose requests.

Service:
  **/state/battery/set_mode** changes battery mode to either 'charging' or 'discharging' by asking :mod:`owl_interface`.
"""

import threading
import random
import rospy
from assignment1 import utils
from assignment1.msg import Battery, Location
from assignment1.srv import BatteryMode
from std_msgs.msg import Empty

LOG_TAG = 'robot_state'

class RobotState:
  """
  This class implements the aforementioned functionalities.
  """
  # Defining class attributes
  _battery = Battery()
  _battery_mode = 'discharging'

  def __init__(self):
    # Initialisation
    rospy.init_node('robot_state', anonymous=True)
    self._pose = None
    self._battery.battery_level = 50.0 # battery can go from 0.0 to 100.0

    # Set default parameter values, in case not specified by launch file
    if not rospy.has_param('robot_state/battery_tick'):
      rospy.set_param('robot_state/battery_tick', 1)
    if not rospy.has_param('robot_state/battery_charge_per_tick'):
      rospy.set_param('robot_state/battery_charge_per_tick', 10)
    if not rospy.has_param('robot_state/battery_discharge_per_tick'):
      rospy.set_param('robot_state/battery_discharge_per_tick', 1)
    if not rospy.has_param('robot_state/battery_publish'):
      rospy.set_param('robot_state/battery_publish', 1)

    # Subscribe to the /owl_interface/goal topic and get the next goal
    # Then republish it
    self._sub_next_goal = rospy.Subscriber('/owl_interface/goal', Location,
                                          self._subscribe_goal_callback, queue_size = 1)

    # Subscribe to the /owl_interface/get_pose topic and get the current pose
    # Then republish it
    self._sub_get_pose = rospy.Subscriber('/owl_interface/get_pose', Location,
                                          self._subscribe_get_pose_callback, queue_size = 1)

    # Subscribe to the /state/set_pose topic and republish it to the
    # owl_interface
    self._sub_set_pose = rospy.Subscriber('/state/set_pose', Location,
                                          self._subscribe_set_pose_callback, queue_size = 1)

    # Publish battery on a separate thread to /state/battery_level
    thread_pub_battery = threading.Thread(target=self._publish_battery)
    thread_pub_battery.start()

    # Service to change battery mode between charging, discharging
    self._srv_battery_set_mode = rospy.Service('/state/battery/set_mode', BatteryMode, self._battery_set_mode)

    # Simulate battery drain on a separate thread
    thread_sim_battery = threading.Thread(target=self._simulate_battery)
    thread_sim_battery.start()

    # Log
    log_msg = ('Initialised node: robot_state')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

  def _subscribe_goal_callback(self, goal):
    """
    Subscribes to /owl_interface/goal and gets the next goal. Then, republishes the goal on the /state/map/get_goal topic.

    Args:
      goal (Location): the current goal
    """
    pub_next_goal = rospy.Publisher('/state/map/get_goal', Location, queue_size=1)
    pub_next_goal.publish(goal)
    log_msg = (f'Goal received and republished: {goal.name}')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

  def _subscribe_get_pose_callback(self, pose):
    """
    Subscribes to /state/get_pose and gets the current pose. Then, republishes the pose on the /state/get_pose topic.

    Args:
      pose (Location): the current pose
    """
    current_location = Location()
    current_location = pose
    pub_get_pose = rospy.Publisher('/state/get_pose', Location, queue_size=1)
    pub_get_pose.publish(current_location)
    log_msg = (f'Current pose received and republished: {current_location.name}')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

  def _subscribe_set_pose_callback(self, pose):
    """
    Subscribes to /state/set_pose and gets the requested pose. Then, republishes the pose on the /owl_interface/set_pose topic, which will be used by the :mod:`owl_interface` module to change the pose in the ontology.

    Args:
      pose (Location): the new pose to set
    """
    new_location = Location()
    new_location = pose
    pub_set_pose = rospy.Publisher('/owl_interface/set_pose', Location, queue_size=1)
    pub_set_pose.publish(new_location)
    log_msg = (f'New pose received and republished: {new_location.name}')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

  def _battery_set_mode(self, data):
    """
    Sets the battery mode to 'charging' or 'discharging':\n
    * 'charging': in this mode, the battery increases over time.\n
    * 'discharging': instead, in this mode the battery decreases over time.\n

    Args:
      data (BatteryMode): 'charging' or 'discharging'
    """
    if (data.battery_mode == 'charging'):
      self._battery_mode = 'charging'
    else:
      self._battery_mode = 'discharging'

    return Empty()

  def _simulate_battery(self):
    """
    Simulates battery drain/charge over time. Both the rate and the quantity per tick can be defined by parameters:\n
    * robot_state/battery_tick defines the rate of change.\n
    * robot_state/battery_charge_per_tick defines the amount charged per tick.\n
    * robot_state/battery_discharge_per_tick defines the amount discharged per tick.\n
    """
    # Get drain rate from parameter server
    rate = rospy.Rate(rospy.get_param('robot_state/battery_tick'))
    charge_per_tick = rospy.get_param('robot_state/battery_charge_per_tick')
    discharge_per_tick = rospy.get_param('robot_state/battery_discharge_per_tick')

    while not rospy.is_shutdown():
      battery_level = self._battery.battery_level
      battery_mode = self._battery_mode

      if (battery_mode == 'discharging'):
        battery_level -= discharge_per_tick
        if (battery_level < 0):
          battery_level = 0
      elif (battery_mode == 'charging'):
        battery_level += charge_per_tick
        if (battery_level > 100):
          battery_level = 100

      self._battery.battery_level = battery_level

      rate.sleep()

  def _publish_battery(self):
    """
    Publishes the current battery level to /state/battery_level at a rate defined by the /robot_state/battery_publish parameter.
    """
    # Publisher
    pub_battery = rospy.Publisher('/state/battery_level', Battery, queue_size = 1, latch = True)
    # Get publish rate from parameter server
    rate = rospy.Rate(rospy.get_param('robot_state/battery_publish'))

    while not rospy.is_shutdown():
      battery = Battery()
      battery.battery_level = self._battery.battery_level
      pub_battery.publish(battery)
      rospy.loginfo(utils.tag_log(f'Published current battery level: {battery.battery_level: 1f}', LOG_TAG))
      rate.sleep()

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
