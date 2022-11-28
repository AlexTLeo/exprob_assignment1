#!/usr/bin/env python

"""
.. module::behaviour
   :platform: Ubuntu 20.04
   :snyopsis: This module manages the underlying state machine of the program.

.. moduleauthor::Alex Thanaphon Leonardi <thanaphon.leonardi@gmail.com>

This module manages the underlying Finite State Machine (FSM) of the program.
The `ROS SMACH library <http://wiki.ros.org/smach>`_ is used to represent the FSM.
States are non-concurrent and updated at a rate defined by the
STATE_UPDATE_PERIOD global variable.

ROS Parameters:
  **/battery/threshold_low** the threshold below which the battery should be considered as 'low'.\n
  **/battery/threshold_critical** the threshold below which the battery should be considered as 'critical'.

Subscribes to:
  **/state/battery_level** receive the robot's current battery level.

Service Proxy:
  **/owl_interface/get_goal** get the next goal.\n
  **/owl_interface/get_pose** get the current pose.

Action Client:
  **/battery/controller/charge** requests the robot go into charging mode.\n
  **/motion/planner** request a plan for the given goal.\n
  **/motion/controller/move** move the robot.

Note:
  Some variables may be concurrently accessed and are therefore protected by
  `locks <https://docs.python.org/3/library/threading.html>`_.
  Extra care should be used if touching these variables (is_battery_low and
  is_battery_critical).
"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
from threading import Lock
from std_msgs.msg import Empty
from assignment1 import utils
from assignment1.msg import *
from assignment1.srv import GetGoal, GetPose
from os.path import dirname, realpath

LOG_TAG = 'behaviour'

# A lock used to protect shared memory
lock = Lock()

# How many seconds to wait in-between each state update
STATE_UPDATE_PERIOD = 1 # s
# How many seconds to wait in-between each battery check
BATTERY_STATUS_PERIOD = 2

# The list of all states
STATE_CHARGING = 'CHARGING'
STATE_PLAN_GOAL = 'PLAN_GOAL'
STATE_MOVING = 'MOVING'
STATE_WAITING = 'WAITING'

# The list of all transitions
TRANS_PLAN_GOAL = 'plan_goal'
TRANS_LOW_BATTERY = 'low_battery'
TRANS_MOVE_TO_GOAL = 'move_to_goal'
TRANS_GOAL_REACHED = 'goal_reached'

# The list of names that identify variables passed across the nodes of the Finite State Machine.
VAR_PLAN_TO_GOAL = 'plan_to_goal'

# Global variable to keep track of battery status across all states
is_battery_low = False
is_battery_critical = False

# state: CHARGING
class Charging(smach.State):
  """
  This class represents the CHARGING state of the state machine, in which the
  robot is actively recharging its battery at charging station.

  The robot has three battery thresholds: charged, low_battery, critical.\n
  * If charged, the robot can operate normally.\n
  * If low_battery, the robot keeps working until the current task is completed,
    and then goes to a charging state immediately after.\n
  * If critical, the robot interrupts whatever it is doing and immediately goes
    to a charging station.
  """

  global is_battery_low
  global is_battery_critical

  # Action client to charge robot
  _act_client_charge = actionlib.SimpleActionClient('/battery/controller/charge', ChargeAction)

  def __init__(self):
    self._sub_battery_active = False

    smach.State.__init__(self,
                         outcomes=[TRANS_PLAN_GOAL,
                                   TRANS_LOW_BATTERY])

    # Service: /owl_interface/get_pose
    rospy.wait_for_service('/owl_interface/get_pose')
    self._srv_get_pose = rospy.ServiceProxy('/owl_interface/get_pose', GetPose)

  def _charge_battery(self):
    """
    Requests the controller to charge the robot, via the /battery/controller/charge
    action server.
    """
    goal = ChargeGoal()
    self._act_client_charge.wait_for_server()
    self._act_client_charge.send_goal(goal)

    # Wait until the current goal is completed before sending another
    rospy.loginfo(utils.tag_log('Sent charging request to action server, waiting...', LOG_TAG))
    self._act_client_charge.wait_for_result()

    rospy.loginfo(utils.tag_log('Charger has stopped', LOG_TAG))
    self._act_client_charge.get_result()

  def execute(self, userdata):
    """
    Every STATE_UPDATE_PERIOD, check if the battery is low or not. If so, then
    check if the current room has a charging station. If so, transition to state
    CHARGING. Otherwise, if current room has no charging station, or if the
    robot has been fully charged, transition to PLAN_GOAL.
    """
    rospy.sleep(STATE_UPDATE_PERIOD)

    with lock:
      ibl = is_battery_low

    if ibl == True:
      # Get current location
      try:
        current_location = self._srv_get_pose(Empty()).pose
      except rospy.ServiceException as e:
        rospy.loginfo(utils.tag_log(f'Could not get current pose: {e}', LOG_TAG))

      if current_location.name == 'E':
        # If battery low and in room 'E', send request to controller to charge (or to keep charging)
        self._charge_battery()
      else:
        # Otherwise, go to the charging room!
        rospy.loginfo(utils.tag_log('Could not start charging: no charger present in current location', LOG_TAG))
        return TRANS_PLAN_GOAL

      return TRANS_LOW_BATTERY
    else:
      # Otherwise, continue operations
      return TRANS_PLAN_GOAL

# state: PLAN_GOAL
class PlanGoal(smach.State):
  """
  This class represents the PLAN_GOAL state of the state machine, in which the
  robot looks for the next location to move towards.
  Once chosen, the robot will plan a path to it by finding the shortest known path,
  which it infers from the ontology.
  """

  # An Action Client passes the goal to the planner
  _act_client = actionlib.SimpleActionClient('/motion/planner', PlanAction)

  def __init__(self):
    smach.State.__init__(self,
                         outcomes=[TRANS_MOVE_TO_GOAL,
                                   TRANS_LOW_BATTERY,
                                   TRANS_PLAN_GOAL],
                         output_keys=[VAR_PLAN_TO_GOAL])
    self._current_plan = None

    # Service: /owl_interface/get_goal
    rospy.wait_for_service('/owl_interface/get_goal')
    self._srv_get_goal = rospy.ServiceProxy('/owl_interface/get_goal', GetGoal)

  def _request_plan(self, goal):
    """
    Requests a plan from the Planner by sending it a goal, through the
    /motion/planner/ action server.

    Args:
      goal (Location): the final location to reach.
    """
    self._act_client.wait_for_server()
    self._act_client.send_goal(goal)

    # Wait until the current goal is completed before sending another
    rospy.loginfo(utils.tag_log(f'Sent goal {goal.goal} to the Planner, waiting...',
                                    LOG_TAG))
    self._act_client.wait_for_result()

    rospy.loginfo(utils.tag_log(f'Plan received from Planner:\n' +
                                f'{self._act_client.get_result()}', LOG_TAG))
    self._current_plan = self._act_client.get_result()

  def _srv_get_goal(self):
    """
    Get the next goal by calling the /owl_interface/get_goal service.

    Raises:
      rospy.ServiceException - indicates failure to call service.
    """
    try:
      req = GetGoal()
      return self._srv_get_goal(req)
    except rospy.ServiceException as e:
      rospy.loginfo(utils.tag_log(f'Failed to get goal: {e}', LOG_TAG))
      return ''

  def execute(self, userdata):
    """
    Request a plan for the current goal. When received, transition to the MOVING
    state.
    """
    # Get next goal
    goal = PlanGoal()
    goal = self._srv_get_goal()

    # Start the planner action server
    self._request_plan(goal)

    rospy.sleep(STATE_UPDATE_PERIOD)

    # If plan has been calculated, then change state to MOVE_TO_GOAL, otherwise
    # stay in current state
    if not self._current_plan:
      return TRANS_PLAN_GOAL
    else:
      # Pass the plan to the controller via userdata
      userdata.plan_to_goal = self._current_plan
      return TRANS_MOVE_TO_GOAL

# state: MOVING
class Moving(smach.State):
  """
  This class represents the MOVING state of the state machine, in which the robot,
  having received a plan from the Planner, physically moves towards the goal via
  the Controller.
  """

  global is_battery_low
  global is_battery_critical

  # An Action Client passes the plan to the controller
  _act_client = actionlib.SimpleActionClient('/motion/controller/move', ControlAction)
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=[TRANS_GOAL_REACHED,
                                   TRANS_LOW_BATTERY,
                                   TRANS_MOVE_TO_GOAL,
                                   TRANS_PLAN_GOAL],
                         input_keys=[VAR_PLAN_TO_GOAL])

  def _activate_controller_move(self, plan):
    """
    Pass the current plan to the Controller via the /motion/controller/move
    action server.
    If the battery is low, transition to the CHARGING state.
    """
    # Send a request to the Controller Action Server
    goal = ControlGoal()
    goal = plan
    self._act_client.wait_for_server()
    # Send all via_points to the controller
    rospy.loginfo(utils.tag_log('Now sending all via_points to the controller...', LOG_TAG))
    self._act_client.send_goal(goal)

    self._act_client.wait_for_result()

    rospy.loginfo(utils.tag_log(f'Controller has stopped', LOG_TAG))
    self._controller_success = self._act_client.get_result().success.data # Bool msg

    return self._controller_success

  def execute(self, userdata):
    """
    Activate the movement controller and send it the plan. If the battery is low,
    transition to state CHARGING. If the goal is not reached, transition to
    state PLAN_GOAL. If te goal is successfully reached, transition to state
    WAITING.
    """
    self._activate_controller_move(userdata.plan_to_goal)

    rospy.sleep(STATE_UPDATE_PERIOD)

    if (self._controller_success == True):
      rospy.loginfo(utils.tag_log(f'Controller signals SUCCESS: final goal has been reached', LOG_TAG))

      # Check if battery needs to be charged
      with lock:
        ibl = is_battery_low

      if (ibl == True):
        return TRANS_LOW_BATTERY

      return TRANS_GOAL_REACHED
    else:
      rospy.loginfo(utils.tag_log(f'Controller signals FAILURE: final goal not reached', LOG_TAG))
      return TRANS_PLAN_GOAL

# state: WAITING
class Waiting(smach.State):
  """
  This class represents the WAITING state of the state machine, in which the robot,
  having reached its goal, waits for a random amount of time before going back
  to the MOVING state.
  """

  global is_battery_low
  global is_battery_critical

  def __init__(self):
    smach.State.__init__(self,
                         outcomes=[TRANS_PLAN_GOAL,
                                   TRANS_LOW_BATTERY,
                                   TRANS_GOAL_REACHED])

  def _simulate_work(self):
    """
    Simulate work by sleeping for some seconds.
    This is split up into multiple smaller waits so that it may be preempted
    in case of critically low battery.
    """
    total_time = random.randint(50, 200)
    for i in range(total_time):
      with lock:
        ibc = is_battery_critical

      if (ibc == True):
        break
      else:
        rospy.loginfo(utils.tag_log(f'({i} -> {total_time}) Working...', LOG_TAG))
        rospy.sleep(1)

  def execute(self, userdata):
    """
    While waiting, keep checking if the battery gets too low. If it goes below
    the critical threshold, then interrupt the waiting and transition to the
    CHARGING state.

    When done waiting, transition to the PLAN_GOAL state.
    """
    rospy.sleep(STATE_UPDATE_PERIOD)

    # Check battery status
    with lock:
      ibl = is_battery_low
      ibc = is_battery_critical

    if (ibl == True):
      # Is the battery level critical?
      if (ibc == False):
        # If battery not critical, continue working but attempt charging immediately afterwards
        self._simulate_work()

      # Now attempt charging in this room
      return TRANS_LOW_BATTERY

    else:
      # If battery is good, then operate normally
      self._simulate_work()

    return TRANS_PLAN_GOAL

def init_state_machine():
  """
  This function initialises, configures, and runs the state machine.
  The Introspection Server is also started to allow the state machine to be
  visualised via the `ROS SMACH library <http://wiki.ros.org/smach>`_.

  Returns:
    introspec_server (IntrospectionServer): the introspection server handler.
  """
  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['container_interface'])

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add(STATE_CHARGING, Charging(),
                           transitions={TRANS_PLAN_GOAL:STATE_PLAN_GOAL,
                                        TRANS_LOW_BATTERY:STATE_CHARGING})
    smach.StateMachine.add(STATE_PLAN_GOAL, PlanGoal(),
                           transitions={TRANS_MOVE_TO_GOAL:STATE_MOVING,
                                        TRANS_LOW_BATTERY:STATE_CHARGING,
                                        TRANS_PLAN_GOAL:STATE_PLAN_GOAL})
    smach.StateMachine.add(STATE_MOVING, Moving(),
                           transitions={TRANS_GOAL_REACHED:STATE_WAITING,
                                        TRANS_LOW_BATTERY:STATE_CHARGING,
                                        TRANS_MOVE_TO_GOAL:STATE_MOVING,
                                        TRANS_PLAN_GOAL:STATE_PLAN_GOAL})
    smach.StateMachine.add(STATE_WAITING, Waiting(),
                           transitions={TRANS_PLAN_GOAL:STATE_PLAN_GOAL,
                                        TRANS_LOW_BATTERY:STATE_CHARGING,
                                        TRANS_GOAL_REACHED:STATE_WAITING})

  # Create and start the introspection server for visualisation
  introspec_server = smach_ros.IntrospectionServer('introspection_server',
                                                      sm, '/SM_ASSIGNMENT1')
  introspec_server.start()

  # Execute the state machine
  outcome = sm.execute()

  return introspec_server

def subscribe_battery_callback(data):
  """
  Subscribes to the /state/battery_level topic and gets the current battery level.

  This callback remains active the entire time, and never shuts down. State
  transitions do not affect it (the alternative would be to have the same
  function repeated in each and every state, which is unnecessary and bad).
  """
  global is_battery_low
  global is_battery_critical

  battery_msg = Battery()
  battery_msg = data
  battery_level = battery_msg.battery_level
  battery_threshold_low = rospy.get_param('battery/threshold_low')
  battery_threshold_critical = rospy.get_param('battery/threshold_critical')

  # Check battery status
  if (battery_level <= battery_threshold_low):
    # LOW_BATTERY: recharge as soon as it is convenient
    with lock:
      is_battery_low = True
    if (battery_level <= battery_threshold_critical):
      # CRITICAL: recharge immediately
      rospy.loginfo(utils.tag_log('CRITICAL battery level detected, sending robot to charging station immediately', LOG_TAG))
      with lock:
        is_battery_critical = True
    else:
      rospy.loginfo(utils.tag_log('LOW battery level detected, instructing robot to recharge as soon as it is most convenient', LOG_TAG))
  else:
    # CHARGED: operate normally
    with lock:
      is_battery_low = False
      is_battery_critical = False

  rospy.sleep(BATTERY_STATUS_PERIOD)

def main():
  """
  This represents the main function, in which the state machine is initialised,
  configured and run. The Introspection Server is also started to allow the
  state machine to be visualised via the `ROS SMACH library <http://wiki.ros.org/smach>`_.
  """
  rospy.init_node('behaviour', anonymous=True)

  # Log info
  log_msg = ('Initialised node: behaviour')
  rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

  # Set default parameter values, if not specified
  if not rospy.has_param('battery/threshold_low'):
    rospy.set_param('battery/threshold_low', 40)
  if not rospy.has_param('battery/threshold_critical'):
    rospy.set_param('battery/threshold_critical', 20)

  # Get current battery level from /state/battery_level
  # (runs on a separate thread)
  rospy.Subscriber('state/battery_level', Battery,
        subscribe_battery_callback, queue_size = 1)

  # Starts the state machine and the introspection server to visualise it
  introspec_server = init_state_machine()

  # Wait for ctrl-c to stop
  rospy.spin()
  introspec_server.stop() # turn off the introspection server

if __name__ == '__main__':
  main()
