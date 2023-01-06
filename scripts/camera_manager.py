#!/usr/bin/env python

"""
.. module::camera_manager
   :platform: Ubuntu 20.04
   :snyopsis: This module manages the camera joint and scans for surrounding
   markers.

.. moduleauthor:: Alex Thanaphon Leonardi <thanaphon.leonardi@gmail.com>

This module manages the camera joint and scans for surrounding
markers. The camera can either scan the room for markers, "marker" mode, or scan
a room normally doing a full rotation, "room" mode.

Publises to:
  **/Giskard/joint_camera_controller/command**: publishes commands to the camera z-axis rotational joint.\n
  **/Giskard/joint_camera_vertical_controller/command**: publishes commands to the camera z-axis prismatic joint.

Subscribes to:
  **/marker_publisher/marker_id**: gets all marker IDs when in "marker" scan mode.

Service:
  **/camera_manager/start_scan**: given a scan mode, starts scanning.
"""

import rospy
import threading
from exprob_assignment1 import utils
from exprob_assignment1.msg import *
from exprob_assignment1.srv import *
from assignment2.srv import RoomInformation
from std_msgs.msg import Float64, Int32

LOG_TAG = 'camera_manager'
lock = threading.Lock()

class CameraManager():
  """
  Class representing the camera manager.
  """

  def __init__(self):
    """
    Initialises the node, sets some helper variables, and starts the service to
    receive scan requests.
    """
    rospy.init_node('camera_manager', anonymous=True)

    log_msg = ('Initialised node: camera_manager')
    rospy.loginfo(utils.tag_log(log_msg, LOG_TAG))

    # Helper variables
    self._camera_curr_rot = 0
    self._detected_marker_ids = set() # sets are fastest for lookup

    # Service: /camera_manager/start_scan
    rospy.Service('/camera_manager/start_scan', StartScan, self._service_start_scan)

  def _service_start_scan(self, type):
    """
    Starts the scan, based on requested type (either "marker" or "room")
    """
    scan_type = type.type
    self._camera_curr_rot = 0

    # Build the ontology once the marker scanning is complete
    thread_build_ontology = threading.Thread(target=self._build_ontology)

    # Get all marker IDs and store them in an array by subscribing to
    # /marker_publisher/marker_id
    self._sub_marker_id = rospy.Subscriber('/marker_publisher/marker_id', Int32,
                                          self._subscribe_marker_id_callback,
                                          queue_size = 100)

    if (scan_type == "marker"):
      # Four full rotations: clockwise up, counter-clockwise up, clockwise down,
      # counter-clockwise down
      self._publish_marker_scan(thread_build_ontology)

    elif (scan_type == "room"):
      # One single full rotation
      self._publish_room_scan(thread_build_ontology)

    return True

  def _publish_marker_scan(self, thread_build_ontology):
    """
    Publishes commands to the /Giskard/joint_camera_controller/command topic and
    also to the /Giskard/joint_camera_vertical_controller/command topic
    """
    # Publisher
    pub_camera_command = rospy.Publisher('/Giskard/joint_camera_controller/command',
                                      Float64, queue_size = 60, latch = True)
    pub_camera_vertical_command = rospy.Publisher('/Giskard/joint_camera_vertical_controller/command',
                                      Float64, queue_size = 60, latch = True)
    rate = rospy.Rate(2)

    # Total number of full rotations
    num_rot = 0;

    # Raise camera up
    command_prismatic = 1.5
    pub_camera_vertical_command.publish(command_prismatic)

    while not rospy.is_shutdown():
      command = Float64()
      command_prismatic = Float64()
      rot_step = 0.19625*4

      # Rotate the camera 360 degrees
      if (num_rot % 2 == 0):
        self._camera_curr_rot = self._camera_curr_rot - rot_step
      else:
        self._camera_curr_rot = self._camera_curr_rot + rot_step

      if abs(self._camera_curr_rot) > (6.28 + rot_step):
        # Full rotation completed (plus a margin for extra rotation)
        num_rot += 1;
        self._camera_curr_rot = 0.0
        command.data = self._camera_curr_rot
        pub_camera_command.publish(command)

        # If first rotation complete, lower the camera
        if (num_rot == 1):
          command_prismatic = 0.5
          pub_camera_vertical_command.publish(command_prismatic)
        elif (num_rot == 2):
          # Second rotation is final so reset and stop the joints
          command_prismatic = 0
          pub_camera_vertical_command.publish(command_prismatic)
          # Start the ontology building after a short wait to sync
          rospy.sleep(3)
          thread_build_ontology.start()
          rospy.loginfo(utils.tag_log(f"Marker scanning complete."
                                       "Ontology building initiated.", LOG_TAG))
          return
      else:
        command.data = self._camera_curr_rot
        pub_camera_command.publish(command)

      rate.sleep()

  def _publish_room_scan(self, thread_build_ontology):
    """
    Publishes commands to the /Giskard/joint_camera_controller/command topic and
    also to the /Giskard/joint_camera_vertical_controller/command topic.
    """
    # Publisher
    pub_camera_command = rospy.Publisher('/Giskard/joint_camera_controller/command',
                                      Float64, queue_size = 60, latch = True)
    pub_camera_vertical_command = rospy.Publisher('/Giskard/joint_camera_vertical_controller/command',
                                      Float64, queue_size = 60, latch = True)
    rate = rospy.Rate(2)

    # Raise camera up
    command_prismatic = 0.5
    pub_camera_vertical_command.publish(command_prismatic)

    while not rospy.is_shutdown():
      command = Float64()
      rot_step = 0.19625*2

      # Rotate the camera 360 degrees
      self._camera_curr_rot = self._camera_curr_rot + rot_step

      if abs(self._camera_curr_rot) > (6.28 + rot_step):
        # Full rotation completed (plus a margin for extra rotation)
        self._camera_curr_rot = 0.0
        command.data = self._camera_curr_rot
        pub_camera_command.publish(command)
        rospy.loginfo(utils.tag_log(f"Room scanning complete.", LOG_TAG))

        # Lower camera
        command_prismatic = 0.0
        pub_camera_vertical_command.publish(command_prismatic)

        return
      else:
        command.data = self._camera_curr_rot
        pub_camera_command.publish(command)

      rate.sleep()

  def _subscribe_marker_id_callback(self, data):
    """
    Subscribes to /marker_publisher/marker_id and gets the detected marker IDs,
    which are added to an array which will contain all detected IDs.

    Args:
      data (Int32): the marker ID
    """
    marker_id = Int32()
    marker_id = data.data

    # Python set will automatically avoid duplicates
    with lock:
      self._detected_marker_ids.add(marker_id)

  def _build_ontology(self):
    """
    Sends the detected marker IDs to the marker server, gets the corresponding
    information and builds the ontology by interfacing with the owl_interface
    """
    rospy.wait_for_service('/room_info')
    service_get_room_info = rospy.ServiceProxy('/room_info', RoomInformation)
    rospy.wait_for_service('/owl_interface/add_room')
    service_add_room = rospy.ServiceProxy('/owl_interface/add_room', AddRoom)

    with lock:
      detected_marker_ids = self._detected_marker_ids

    rospy.loginfo(utils.tag_log(f'detected marker ids: {detected_marker_ids}', LOG_TAG))
    rospy.loginfo(utils.tag_log("Sending room information to owl_interface", LOG_TAG))

    for id in detected_marker_ids:
      room_info = AddRoomRequest()

      ret = service_get_room_info(id)

      # If response is invalid, skip this ID
      if ret.room == "no room associated with this marker id":
        rospy.loginfo(utils.tag_log(f'Invalid ID detected ({id}), skipping', LOG_TAG))
        continue

      room_info.room = ret.room
      room_info.x = ret.x
      room_info.y = ret.y
      room_info.connections = ret.connections

      service_add_room(room_info)

      rospy.loginfo(utils.tag_log(f"Request sent for {room_info.room}", LOG_TAG))

if __name__ == "__main__":
    CameraManager()
    rospy.spin()
