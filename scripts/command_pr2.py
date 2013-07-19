#!/usr/bin/ipython -i
from pr2.ArmPlannerPR2 import PlannerPR2
import rospy
rospy.init_node("command_pr2", disable_signals = True)
pr2 = PlannerPR2()