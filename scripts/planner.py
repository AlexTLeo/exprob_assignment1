"""
.. module::planner
   :platform: Ubuntu 20.04
   :snyopsis: This module calculates the shortest path to reach the goal

.. moduleauthor::Alex Thanaphon Leonardi <thanaphon.leonardi@gmail.com>

This module receives the next goal and calculates the shortest path to reach that goal, i.e.
a set of viapoints. These viapoints are then sent to the controller module.

In this simulated program, only adjacent movements are made, therefore the planner does not
actually have to compute anything and instead simulates work by busy waiting.

Action Server:
  **/motion/planner** given a goal, calculates a path to it.\n

"""

import rospy
import threading
import actionlib
from assignment1 import utils
from assignment1.msg import Location, PlanAction, PlanFeedback, PlanResult

LOG_TAG = 'planner'

class PlannerAction():
  """
  This class represents the planner and its /motion/planner action server.
  """
  def __init__(self):
    """
    Initialise the ros_node, define the action server and start it.
    """
    rospy.init_node('planner', anonymous=True)

    # Define the action server and start it
    self._as = actionlib.SimpleActionServer('/motion/planner',
                                            PlanAction,
                                            execute_cb=self._execute_cb,
                                            auto_start = False)
    self._as.start()

    # Log info
    log_msg = ('Initialised node: planner')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

  def _execute_cb(self, goal):
    """
    Callback function for the Planner Action Server. In this simulation, it
    simply bounces the goal back to the behaviour node, which in turn will send
    it to the /motion/controller/move action server.\n

    Args:
      goal (PlanGoal): the goal location

    Note:
      In a real scenario, this callback would actually use a graph-search algorithm
      to find the optimal path towards the goal.
    """
    success = True

    # Define messages that are used to publish feedback/result
    feedback = PlanFeedback()
    result = PlanResult()
    feedback.via_points = []

    # Execute action and send feedback
    # In reality, this should take a while, but in this simulation the planner
    # does not actually do any computation, so computation is simulated
    self._simulate_computation()
    feedback.via_points.append(goal.goal)
    self._as.publish_feedback(feedback)

    if success:
      result = feedback
      log_msg = (f'Planner successfully publised goal: {result}')
      rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))
      self._as.set_succeeded(result)

  def _simulate_computation(self):
    rospy.loginfo(utils.tag_log('Computing path...', LOG_TAG))
    rospy.sleep(2)

if __name__ == "__main__":
    PlannerAction()
    rospy.spin()
