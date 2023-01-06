#!/usr/bin/env python

"""
.. module::controller
   :platform: Ubuntu 20.04
   :snyopsis: This module receives a set of waypoints and physically moves
   the robot through them.

.. moduleauthor:: Alex Thanaphon Leonardi <thanaphon.leonardi@gmail.com>

This module receives a set of waypoints from the :mod:`behaviour` module (that
in turn received it from the :mod:`planner` module) and
physically moves the robot through them. Once the final destination is reached,
or upon failure, the :mod:`behaviour` module is notified.
This module also manages the robot's battery charging.

In this simulated program, only adjacent locations are considered reachable,
therefore the controller only ever has one waypoint to go towards.

The `move_base package <http://wiki.ros.org/move_base>`_ is used to navigate
the robot, and the `gmapping package <http://wiki.ros.org/gmapping>`_ is used
to map the surroundings.

Publishes to:
  **/owl_interface/set_pose** sets the pose (i.e. robot location) in the ontology.

Subscribes to:
  **/state/battery_level** gets the current battery level.

ServiceProxy:
  **/state/battery/set_mode** sets the battery to either charge or discharge.\n
  **/owl_interface/update_visited** updates the timestamp associated to a location.

Action Server:
  **/motion/controller/scan** given a scan mode, starts camera scanning\n
  **/motion/controller/move** given a set of viapoints, moves the robot through them.\n
  **/battery/controller/charge** charges the robot.
"""

import rospy
import threading
import actionlib
from rospy.exceptions import ROSException
from exprob_assignment1 import utils
from exprob_assignment1.msg import *
from exprob_assignment1.srv import *
from std_msgs.msg import Bool, Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

LOG_TAG = 'controller'

class ControllerAction():
  """
  This class represents the controller node, and is comprised of two Action
  Servers:

  * the /motion/controller/move action server deals with moving the robot through
    each waypoint up until the final goal.
  * the /battery/controller/charge action server deals with setting the robot to
    'charging' mode until the battery reaches 100%, at which point the battery is set back to 'discharging' mode.
  """

  def __init__(self):
    """
    Initialise the ros_node, define the publishers, subscribers and services, and define the action servers and start them.
    """
    rospy.init_node('controller', anonymous=True)

    log_msg = ('Initialised node: controller')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

    # Helper variable used to determine if battery is full or not
    self._is_fully_charged = False

    # Publisher: /owl_interface/set_pose
    self._pub_set_pose = rospy.Publisher('/owl_interface/set_pose', Location, queue_size=1)

    # Get current battery level from /state/battery_level
    rospy.Subscriber('/state/battery_level', Battery,
          self._subscribe_battery_callback, queue_size = 1)

    # Service: /state/battery/set_mode
    rospy.wait_for_service('/state/battery/set_mode')
    self._srv_set_battery_mode = rospy.ServiceProxy('/state/battery/set_mode', BatteryMode)

    # Service: /owl_interface/update_visited
    rospy.wait_for_service('/owl_interface/update_visited')
    self._srv_update_visited = rospy.ServiceProxy('/owl_interface/update_visited', UpdateVisitedAt)

    # Service: /camera_manager/start_scan
    rospy.wait_for_service('/camera_manager/start_scan')
    self._srv_start_scan = rospy.ServiceProxy('/camera_manager/start_scan', StartScan)

    # Define the SCANNING action server and start it
    self._as_scan = actionlib.SimpleActionServer('/motion/controller/scan',
                                            ScanAction,
                                            execute_cb = self._execute_scan_cb,
                                            auto_start = False)
    self._as_scan.start()

    # Define the MOVEMENT action server and start it
    self._as_move = actionlib.SimpleActionServer('/motion/controller/move',
                                            ControlAction,
                                            execute_cb = self._execute_move_cb,
                                            auto_start = False)
    self._as_move.start()

    # Define the CHARGING action server and start it
    self._as_charge = actionlib.SimpleActionServer('/battery/controller/charge',
                                            ChargeAction,
                                            execute_cb = self._execute_charge_cb,
                                            auto_start = False)
    self._as_charge.start()

  def _execute_scan_cb(self, type):
    """
    Callback function for the /motion/controller/scan action server.\n
    Based on the received request, the robot's camera will either:\n
    - "marker": perform four complete rotations (upper clockwise, upper
      counter-clockwise, lower clockwise, lower counter-clockwise). Then,
      build the ontology through the owl_interface
    - "room": perform one complete rotation

    Args:
      type (ScanAction): Type of scan to perform.

    Feedback:
      ScanFeedback: This should be ignored.

    Result:
      ScanResult: std_msgs/Empty
    """
    # Define messages that are used to publish feedback and result
    feedback = ScanFeedback()
    result = ScanResult()

    # Execute action and send feedback
    # Initiate scan
    if (type.type == "marker"):
      rospy.loginfo(utils.tag_log(f'Initiating marker scan...', LOG_TAG))
      result = self._srv_start_scan("marker")

    elif (type.type == "room"):
      rospy.loginfo(utils.tag_log(f'Initiating room scan...', LOG_TAG))
      result = self._srv_start_scan("room")

    # Send feedback (EMPTY)
    self._as_scan.publish_feedback(feedback)

    self._as_scan.set_succeeded(result)

    rospy.loginfo(utils.tag_log('Scanning complete.', LOG_TAG))

  def _execute_move_cb(self, plan):
    """
    Callback function for the /motion/controller/move action server.\n
    The given plan is comprised of waypoints/viapoints which will be traversed
    one-by-one, until the final one is reached.

    Args:
      plan (ControlAction): a set of Location variables (viapoints).

    Feedback:
      ControlFeedback: the viapoints reached so far.

    Result:
      ControlResult:\n
      True -- success, reached final destination.\n
      False -- failure, final destination not reached.

    Raises:
      ROSException
    """
    # Define messages that are used to publish feedback and result
    feedback = ControlFeedback()
    result = ControlResult()
    feedback.via_points_reached = []

    # Execute action and send feedback
    final_goal = Location()
    for current_goal in plan.via_points:
      final_goal = current_goal
      rospy.loginfo(utils.tag_log(f'Moving robot to {current_goal.name}', LOG_TAG))
      # Moves the robot to the goal by sending it to the move_base action server
      result_move_base = self._move_to_goal(current_goal)
      # Update the map (ontology) by publishing to the /state/set_pose topic
      rospy.loginfo(utils.tag_log(f'Setting new pose to {current_goal}', LOG_TAG))
      self._pub_set_pose.publish(current_goal)
      # Send feedback
      feedback.via_points_reached.append(current_goal)
      self._as_move.publish_feedback(feedback)

    # Check if the final goal has been reached
    if not result_move_base:
      result.success.data = False
      log_msg = (f'Controller failed to reach final goal.' +
                 f' Current position: {current_location}')
      self._as_move.set_aborted(result)
      rospy.logerr(utils.tag_log('Failed to get the current pose', LOG_TAG))
    else:
      # Goal reached
      result.success.data = True
      # Update location timestamp
      self._update_location_visited(final_goal)
      self._as_move.set_succeeded(result)
      log_msg = (f'Controller successfully reached final goal.' +
                 f' Current position: {final_goal.name}')
      rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

  def _execute_charge_cb(self, empty):
    """
    Callback function for the /battery/controller/charge action server.\n
    The robot will be set to 'charging' mode until the battery reaches 100%, at
    which point it will be set back to 'discharging' mode.

    Args:
      empty (ChargeAction): This is ignored.

    Feedback:
      ChargeFeedback: This should be ignored.

    Result:
      ChargeResult:\n
      True -- success, fully charged.\n
      False -- failure, could not change charging mode.

    Raises:
      rospy.ServiceException - indicates failure to call service.
    """
    # Define messages that are used to publish feedback and result
    feedback = ChargeFeedback()
    result = ChargeResult()

    # Execute action and send feedback
    # Charge robot battery
    rospy.loginfo(utils.tag_log(f'Charging the battery...', LOG_TAG))
    self._battery_charge_start()

    # Send feedback (EMPTY)
    self._as_charge.publish_feedback(feedback)

    # Wait until fully charged
    while (self._is_fully_charged == False):
      rospy.sleep(1)

    self._battery_charge_stop()

    self._as_charge.set_succeeded(result)

    rospy.loginfo(utils.tag_log('Charging has stopped', LOG_TAG))

  def _move_to_goal(self, goal_recv):
    """
    Calls the move_base action server to move the robot to the coordinates
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_recv.x
    goal.target_pose.pose.position.y = goal_recv.y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr(utils.tag_log(f'Failed to move robot: move_base not available!', LOG_TAG))
    else:
        rospy.loginfo(utils.tag_log(f'move_base reports completion', LOG_TAG))
        return client.get_result()

  def _battery_charge_start(self):
    """
    Set the robot to charging mode by calling the /state/battery/set_mode service.

    Raises:
      rospy.ServiceException indicates failure to call service.

    Note:
      This function should only be used by :func:`_execute_charge_cb`.
    """
    try:
      mode = BatteryMode()
      mode = 'charging'
      return self._srv_set_battery_mode(mode)
    except rospy.ServiceException as e:
      rospy.loginfo(utils.tag_log(f'Battery charge failed: {e}', LOG_TAG))
      self._as_charge.set_aborted(ChargeResult())

  def _battery_charge_stop(self):
    """
    Set the robot to discharging mode by calling the /state/battery/set_mode service.

    Raises:
      rospy.ServiceException indicates failure to call service.

    Note:
      This function should only be used by :func:`_execute_charge_cb`.
    """
    try:
      mode = BatteryMode()
      mode = 'discharging'
      return self._srv_set_battery_mode(mode)
    except rospy.ServiceException as e:
      rospy.loginfo(utils.tag_log(f'Failed to abort charging mode: {e}', LOG_TAG))
      self._as_charge.set_aborted(ChargeResult())

  def _update_location_visited(self, curr_loc):
    """
    Updates the visitedAt data property of a location in the ontology.

    Raises:
      rospy.ServiceException indicates failure to call service.

    Note:
      This function should only be used by :func:`_execute_move_cb`.
    """
    try:
      loc_to_update = UpdateVisitedAtRequest()
      loc_to_update = curr_loc
      return self._srv_update_visited(loc_to_update)
    except rospy.ServiceException as e:
      rospy.loginfo(utils.tag_log(f'Location time visited update failed: {e}', LOG_TAG))

  def _subscribe_battery_callback(self, data):
    """
    Subscriber callback to the /state/battery_level topic which gets the current
    battery level.

    This callback remains active the entire time, and never shuts down.

    Args:
      data (Battery): the current battery level.
    """
    battery_msg = Battery()
    battery_msg = data
    battery_level = battery_msg.battery_level

    if (battery_level < 100):
      self._is_fully_charged = False
    else:
      self._is_fully_charged = True

if __name__ == "__main__":
    ControllerAction()
    rospy.spin()
