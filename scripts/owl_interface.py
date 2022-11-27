#!/usr/bin/env python

"""
.. module::owl_interface
   :platform: Ubuntu 20.04
   :snyopsis: This module interfaces with the ontology, i.e. an OWL representation of
   the current situation.

.. moduleauthor::Alex Thanaphon Leonardi <thanaphon.leonardi@gmail.com>

This module interfaces with the OWL ontology representing both the map and the
current situation (robot battery, timestamps, etc.). It exposes multiple publishers,
subscribers and services to allow other nodes to interact with the ontology
whilst hiding the implementation details.

The `armor_py_api library <https://github.com/EmaroLab/armor_py_api>`_ is used
to facilitate operations.

ROS Parameters:
  **/owl_interface/map_file_name** the name of the .owl file containing the ontology.\n
  **/battery/threshold_low** the threshold below which the battery should be considered as 'low'.

Publishes to:
  **/owl_interface/get_pose** the current robot location.\n
  **/owl_interface/goal** the current goal.

Subscribes to:
  **/owl_interface/set_pose** receive a new pose to set.\n
  **/state/battery_level** receive the robot's current battery level.

Service:
  **/owl_interface/get_goal** immediately queries the ontology for the next goal.\n
  **/owl_interface/get_pose** immediately queries the ontology for the current pose.\n
  **/owl_interface/update_visited** immediately updates the passed location's
  "visitedAt" timestamp in the ontology.
"""

import rospy
import threading
import time
import re
from assignment1 import utils
from os.path import dirname, realpath
from armor_api.armor_client import ArmorClient
from assignment1.msg import Location, Battery
from assignment1.srv import UpdateVisitedAt, GetGoal, GetPose
from std_msgs.msg import Empty

LOG_TAG = 'owl_interface'
RATE_SEARCH = 0.5 # Hz
RATE_GOAL = 0.5 # Hz
RATE_GET_POSE = 0.5 # Hz
RATE_CHECK_BATTERY = 0.5 # Hz

class OwlInterface:
  """
  This class implements the aforementioned functionalities.
  """
  # Defining class attributes
  _current_goal = Location()

  def __init__(self, robot_name):
    # Initialisation
    rospy.init_node('owl_interface', anonymous=True)

    # Log
    log_msg = ('Initialised node: owl_interface')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

    self.robot_name = robot_name
    self._current_goal.name = ''
    self._is_battery_low = False

    # Set default parameter values, in case not specified by launch file
    if not rospy.has_param('owl_interface/map_file_name'):
      rospy.set_param('owl_interface/map_file_name', 'assignment1_map.owl')
    if not rospy.has_param('battery/threshold_low'):
      rospy.set_param('battery/threshold_low', 40)

    # Maps should be contained within the "maps" folder
    map_dir = dirname(realpath(__file__)) + "/../maps/"
    map_name = rospy.get_param('owl_interface/map_file_name')

    # Start the armor client
    self._armor_client = self._init_armor_client(map_dir, map_name,
        'assignment1_armor_client', 'assignment1_armor_reference')

    # Subscribe to /state/battery_level and get the battery level
    self.sub_get_battery = rospy.Subscriber('/state/battery_level', Battery,
                                          self._subscribe_get_battery, queue_size = 1)

    # Service to get the next goal
    self._srv_get_goal = rospy.Service('/owl_interface/get_goal',
                              GetGoal, self._service_get_goal)

    # Service to get the current pose
    self._srv_get_pose = rospy.Service('/owl_interface/get_pose',
                              GetPose, self._service_get_pose)

    # This function regularly updates the robot's internal clock
    thread_update_robot_time = threading.Thread(target=self._update_robot_time)
    thread_update_robot_time.start()

    # Publish the next goal
    thread_get_goal = threading.Thread(target=self._search_next_goal)
    thread_get_goal.start()

    # Give some time to find the first goal...
    rospy.sleep(1)

    # Publish the current pose
    thread_pub_pose = threading.Thread(target=self._publish_pose)
    thread_pub_pose.start()

    # Subscribe to /owl_interface/set_pose requests and set a new pose in the ontology
    self._sub_set_pose = rospy.Subscriber('/owl_interface/set_pose', Location,
                                          self._subscribe_set_pose_callback,
                                          queue_size = 1)
    # Publish the goal
    thread_pub_goal = threading.Thread(target=self._publish_goal)
    thread_pub_goal.start()

    # Service to update the visitedAt data property
    self._srv_update_visited = rospy.Service('/owl_interface/update_visited',
                              UpdateVisitedAt, self._update_visited)

    # Log
    log_msg = ('Node setup complete')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

    rospy.spin()

  def _init_armor_client(self, map_dir, file_name, client_id, ref_name):
    """
    Initialises the armor client and reads the .owl file containing the
    environment map and its current situation.

    Args:
      map_dir (str): The directory containing all the maps.
      file_name (str): The name of the map to load.
      ref_name (str): The name of the armor reference.

    Returns:
      armor_client (ArmorClient): A handler for the armor client.
    """
    armor_client = ArmorClient(client_id, ref_name)
    armor_client.utils.load_ref_from_file(map_dir + file_name,
                    'http://bnc/exp-rob-lab/2022-23', True, "PELLET", True, False)
    armor_client.utils.mount_on_ref()
    armor_client.utils.set_log_to_terminal(True)

    rospy.loginfo(utils.tag_log('Initialised armor client', LOG_TAG))

    return armor_client

  def _update_robot_time(self):
    """
    Updates the robot's internal clock in the ontology once every second.
    """
    armor_client = self._armor_client
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
      # Get current time and old time
      new_time = str(int(time.time()))
      old_time = armor_client.query.dataprop_b2_ind('now', self.robot_name)[0]
      # Trim the value because the ontology returns extra details
      old_time = re.findall(r'"(.+?)"', old_time)[0]

      # Update the time
      armor_client.manipulation.remove_dataprop_from_ind('now', self.robot_name, 'LONG', old_time)
      armor_client.manipulation.add_dataprop_to_ind('now', self.robot_name, 'LONG', new_time)
      armor_client.utils.sync_buffered_reasoner()
      armor_client.utils.apply_buffered_changes()

      rate.sleep()

  def _subscribe_get_battery(self, data):
    """
    Subscribes to /state/battery_level and gets the current battery level. If it gets
    too low (specified by the /battery/threshold_low parameter), then this node
    will start prioritising locations equipped with a charging station as goal points.

    Args:
      data (Battery): the current battery level
    """
    battery_msg = Battery()
    battery_msg = data
    battery_level = battery_msg.battery_level
    battery_threshold_low = rospy.get_param('battery/threshold_low')

    # Check battery status
    if (battery_level <= battery_threshold_low):
      # LOW_BATTERY: recharge as soon as it is convenient
      self._is_battery_low = True
      rospy.loginfo(utils.tag_log('LOW battery level detected, searching for charging station...', LOG_TAG))
    else:
      # CHARGED: operate normally
      self._is_battery_low = False

  def _service_get_goal(self, req):
    """
    Queries the ontology to get the next goal by calling the /owl_interface/get_goal
    service and returns it to the service caller.

    The rule for finding the next goal is the following: only adjacent locations
    are considered 'reachable'. Among these, the most urgent ones are the ones
    that were visited a long time ago. If no urgent location is found, a random
    one is selected, and corridors are preferred.
    If the battery is low, only rooms with a charging station will be selected
    as acceptable goals.

    This should be used when precise synchronization is required, as the equivalent
    publisher might be slightly slower.

    Args:
      req (Empty): This is ignored.
    """
    goal = Location()
    armor_client = self._armor_client
    # Find reachable locations
    locations_reachable = armor_client.query.objectprop_b2_ind('canReach', self.robot_name)

    # If battery level is low, prioritise charging station, otherwise operate
    # normally
    if self._is_battery_low == False:
      # Get all urgent locations
      locations_urgent = armor_client.query.ind_b2_class('URGENT')
      # Check for reachable urgent locations, if any
      locations_urgent_reachable = list(set(locations_reachable).intersection(locations_urgent))

      # If there are no urgent locations, pick a random reachable location
      if (len(locations_urgent_reachable) == 0):
        # First, prefer corridors
        locations_corridors = armor_client.query.ind_b2_class('CORRIDOR')
        locations_reachable_corridors = list(set(locations_reachable).intersection(locations_corridors))
        if (len(locations_reachable_corridors) > 0):
          goal.name = locations_reachable_corridors[0]
        else:
          # Otherwise, get any location
          goal.name = locations_reachable[0]
        rospy.loginfo(utils.tag_log(f'Picking reachable location: {goal.name}', LOG_TAG))
      else:
        goal.name = locations_urgent_reachable[0]
        rospy.loginfo(utils.tag_log(f'Picking urgent location: {goal.name}', LOG_TAG))
    else:
      # If exists, get closest charging station, otherwise get a random location
      # and try again next search.
      # In this simulation, we know that E is a charging station.
      location_charger = ['E']

      # First of all, check if already in 'E'
      location_current = armor_client.query.objectprop_b2_ind('isIn', self.robot_name)

      if (location_current[0] == location_charger[0]):
        # We are already here! Just send the same goal again.
        goal.name = location_current[0]
        rospy.loginfo(utils.tag_log(f'Already at charging station. Sending same location: {goal.name}', LOG_TAG))

      else:
        # If not there already, then check if E is among the reachable locations.
        locations_reachable_charger = list(set(locations_reachable).intersection(location_charger))

        if (len(locations_reachable_charger) == 0):
          # If no charger locations reachable, pick a random reachable location
          goal.name = locations_reachable[0]
          rospy.loginfo(utils.tag_log(f'Charging station not reachable. Picking reachable location: {goal.name}', LOG_TAG))
        else:
          # Charger found
          goal.name = locations_reachable_charger[0]
          rospy.loginfo(utils.tag_log(f'Picking reachable charging station: {goal.name}', LOG_TAG))

    return goal

  def _search_next_goal(self):
    """
    Looks for the next goal and saves it internally. It will be published by the
    /owl_interface/goal publisher.

    The rule for finding the next goal is the following: only adjacent locations
    are considered 'reachable'. Among these, the most urgent ones are the ones
    that were visited a long time ago. If no urgent location is found, a random
    one is selected. If the battery is low, only rooms with a charging station
    will be selected as acceptable goals.

    If it is strictly necessary to receive the latest goal, then use the service
    /owl_interface/get_goal instead.
    """
    rate = rospy.Rate(RATE_SEARCH)
    goal = Location()

    while not rospy.is_shutdown():
      armor_client = self._armor_client
      # Find reachable locations
      locations_reachable = armor_client.query.objectprop_b2_ind('canReach', self.robot_name)

      # If battery level is low, prioritise charging station, otherwise operate
      # normally
      if self._is_battery_low == False:
        # Get all urgent locations
        locations_urgent = armor_client.query.ind_b2_class('URGENT')
        # Check for reachable urgent locations, if any
        locations_urgent_reachable = list(set(locations_reachable).intersection(locations_urgent))

        # If there are no urgent locations, pick a random reachable location
        if (len(locations_urgent_reachable) == 0):
          goal.name = locations_reachable[0]
          rospy.loginfo(utils.tag_log(f'Picking reachable location: {goal.name}', LOG_TAG))
        else:
          goal.name = locations_urgent_reachable[0]
          rospy.loginfo(utils.tag_log(f'Picking urgent location: {goal.name}', LOG_TAG))
      else:
        # If exists, get closest charging station, otherwise get a random location
        # and try again next search.
        # In this simulation, we know that E is a charging station.
        location_charger = ['E']

        # First of all, check if already in 'E'
        location_current = armor_client.query.objectprop_b2_ind('isIn', self.robot_name)

        if (location_current[0] == location_charger[0]):
          # We are already here! Just send the same goal again.
          goal.name = location_current[0]
          rospy.loginfo(utils.tag_log(f'Already at charging station. Sending same location: {goal.name}', LOG_TAG))

        else:
          # If not there already, then check if E is among the reachable locations.
          locations_reachable_charger = list(set(locations_reachable).intersection(location_charger))

          if (len(locations_reachable_charger) == 0):
            # If no charger locations reachable, pick a random reachable location
            goal.name = locations_reachable[0]
            rospy.loginfo(utils.tag_log(f'Charging station not reachable. Picking reachable location: {goal.name}', LOG_TAG))
          else:
            # Charger found
            goal.name = locations_reachable_charger[0]
            rospy.loginfo(utils.tag_log(f'Picking reachable charging station: {goal.name}', LOG_TAG))

      self._current_goal = goal

      rate.sleep()

  def _publish_pose(self):
    """
    Publishes the current pose to the /owl_interface/get_pose topic at a rate
    defined by the RATE_GET_POSE global variable.
    """
    pub_pose = rospy.Publisher('/owl_interface/get_pose', Location, queue_size = 1, latch = True)
    rate = rospy.Rate(RATE_GET_POSE)
    current_location = Location()

    while not rospy.is_shutdown():
      armor_client = self._armor_client
      # Get current robot location
      current_location.name = armor_client.query.objectprop_b2_ind('isIn', self.robot_name)[0]

      pub_pose.publish(current_location)

      rospy.loginfo(utils.tag_log(f'Published current pose: {current_location.name}', LOG_TAG))

      rate.sleep()

  def _service_get_pose(self, req):
    """
    Queries the ontology to get the current pose by calling the /owl_interface/get_pose
    service and returns it to the service caller.

    This should be used when precise synchronization is required, as the equivalent
    publisher might be slightly slower.

    Args:
      req (Empty): This is ignored.
    """
    armor_client = self._armor_client

    # Get current robot location
    current_location = Location()
    current_location.name = armor_client.query.objectprop_b2_ind('isIn', self.robot_name)[0]

    rospy.loginfo(utils.tag_log(f'Published current pose: {current_location.name}', LOG_TAG))

    return current_location

  def _subscribe_set_pose_callback(self, pose):
    """
    Subscribes to /owl_interface/set_pose and receives new pose requests.
    Then, updates the ontology with that new pose.

    Args:
      pose (Location): the new pose.
    """
    armor_client = self._armor_client
    new_location = Location()
    new_location = pose
    current_location = Location() # Needed by armor_py_api
    current_location.name = armor_client.query.objectprop_b2_ind('isIn', self.robot_name)[0]

    # Update isIn object property
    armor_client.manipulation.replace_objectprop_b2_ind('isIn', self.robot_name,
                                      new_location.name, current_location.name)
    armor_client.utils.sync_buffered_reasoner()
    armor_client.utils.apply_buffered_changes()

    rospy.loginfo(utils.tag_log(f'Set new pose: {new_location.name}', LOG_TAG))

  def _publish_goal(self):
    """
    Publishes current goal to /owl_interface/goal at a rate defined by the
    RATE_GOAL global variable.

    This runs on a separate thread from the goal search, to avoid slowing down the
    publisher in case of lengthy computation times.
    """
    pub_goal = rospy.Publisher('/owl_interface/goal', Location, queue_size = 1, latch = True)
    rate = rospy.Rate(RATE_GOAL)
    goal_msg = Location()

    while not rospy.is_shutdown():
      goal_msg = self._current_goal
      pub_goal.publish(goal_msg)

      rospy.loginfo(utils.tag_log(f'Published goal: {goal_msg.name}', LOG_TAG))

      rate.sleep()

  def _update_visited(self, data):
    """
    Updates the visitedAt data property of the passed location through the
    /owl_interface/update_visited service.

    Args:
      data (Location): the location to update.
    """
    armor_client = self._armor_client

    # Get current time and old time
    new_time = str(int(time.time()))
    old_time = armor_client.query.dataprop_b2_ind('visitedAt', data.location.name)[0]
    # Trim the value because the ontology returns extra details
    old_time = re.findall(r'"(.+?)"', old_time)[0]

    # Update the time
    armor_client.manipulation.remove_dataprop_from_ind('visitedAt', data.location.name, 'LONG', old_time)
    armor_client.manipulation.add_dataprop_to_ind('visitedAt', data.location.name, 'LONG', new_time)
    armor_client.utils.sync_buffered_reasoner()
    armor_client.utils.apply_buffered_changes()

    rospy.loginfo(utils.tag_log(f'Updated data property visitedAt for location {data.location.name}', LOG_TAG))

    return Empty()

if __name__ == "__main__":
    OwlInterface('Robot1')
