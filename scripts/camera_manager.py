"""
.. module::camera_manager
   :platform: Ubuntu 20.04
   :snyopsis: This module manages the camera joint and scans for surrounding
   markers.

.. moduleauthor:: Alex Thanaphon Leonardi <thanaphon.leonardi@gmail.com>

This module manages the camera joint and scans for surrounding
markers.
"""

import rospy
import threading
from rospy.exceptions import ROSException
from exprob_assignment1 import utils
from exprob_assignment1.msg import *
from exprob_assignment1.srv import BatteryMode, UpdateVisitedAt
from std_msgs.msg import Bool, Float64

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

    # Helper variable used to rotate hokuyo laser
    self._hokuyo_curr_rot = 0

    # Publisher: /state/set_pose
    self._pub_set_pose = rospy.Publisher('/state/set_pose', Location, queue_size=10)

    # Publish commands on a separate thread to /Giskard/joint_hokuyo_controller/command
    thread_pub_hokuyo_command = threading.Thread(target=self._publish_hokuyo_command)
    thread_pub_hokuyo_command.start()

    # Get current battery level from /state/battery_level
    # (runs on a separate thread)
    rospy.Subscriber('/state/battery_level', Battery,
          self._subscribe_battery_callback, queue_size = 1)

    # Service: /state/battery/set_mode
    rospy.wait_for_service('/state/battery/set_mode')
    self._srv_set_battery_mode = rospy.ServiceProxy('/state/battery/set_mode', BatteryMode)

    # Service: /owl_interface/update_visited
    rospy.wait_for_service('/owl_interface/update_visited')
    self._srv_update_visited = rospy.ServiceProxy('/owl_interface/update_visited', UpdateVisitedAt)

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

  def _publish_hokuyo_command(self):
    """
    Publishes commands to the /Giskard/joint_hokuyo_controller/command to rotate
    the laser scanner by 360 degrees multiple times every second
    """
    # Publisher
    pub_hokuyo_command = rospy.Publisher('/Giskard/joint_hokuyo_controller/command',
                                      Float64, queue_size = 60, latch = True)
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
      command = Float64()

      # Rotate the hokuyo 360 degrees
      self._hokuyo_curr_rot = (self._hokuyo_curr_rot + 1.57) % 6.28

      command.data = self._hokuyo_curr_rot
      pub_hokuyo_command.publish(command)
      rate.sleep()


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
    # Define messages that are used to publish feedback/result
    feedback = ControlFeedback()
    result = ControlResult()
    feedback.via_points_reached = []

    # Execute action and send feedback
    # Note: the controller is supposed to physically move the robot. Setting
    # the pose is considered a mere software operation of updating the map to
    # reflect the current reality. Setting the pose does not actually change
    # the robot's pose; the physical movement happens here, by the controller.
    # In reality, this should take a while, but in this simulation the controller
    # does not actually do any computation, so computation is simulated
    for current_goal in plan.via_points:
      # Simulate robot movement
      rospy.loginfo(utils.tag_log(f'Moving robot to {current_goal.name}', LOG_TAG))
      self._simulate_computation()
      # Update the map (ontology) by publishing to the /state/set_pose topic
      self._pub_set_pose.publish(current_goal)
      # Send feedback
      feedback.via_points_reached.append(current_goal)
      self._as_move.publish_feedback(feedback)

    # Check if the final goal has been reached
    current_location = Location()
    current_location.name = 'unknown'
    try:
      rospy.sleep(1) # Wait for pose to update
      current_location = rospy.wait_for_message('/state/get_pose', Location, timeout=15)
    except ROSException:
      rospy.loginfo(utils.tag_log('Failed to get the current pose', LOG_TAG))

    if (current_location == feedback.via_points_reached[-1]):
      # Goal reached
      result.success.data = True
      # Update location timestamp
      self._update_location_visited(current_location.name)
      log_msg = (f'Controller successfully reached final goal.' +
                 f' Current position: {current_location.name}')
      self._as_move.set_succeeded(result)

    else:
      # Failure
      result.success.data = False
      log_msg = (f'Controller failed to reach final goal.' +
                 f' Current position: {current_location}')
      self._as_move.set_aborted(result)

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
    # Define messages that are used to publish feedback/result
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

  def _simulate_computation(self):
    """
    2 second busy waiting to simulate computation, used by :func:`_execute_move_cbs`
    """
    rospy.sleep(2)

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
      loc_to_update = UpdateVisitedAt()
      loc_to_update.name = curr_loc
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
