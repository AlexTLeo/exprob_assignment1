#!/usr/bin/env python
import rospy

"""
Labels each log with a produer tag
@author Luca Buoncompagni
"""
def tag_log(msg, producer_tag):
  return f'@{producer_tag} >> {msg}'
