#! /usr/bin/env python

import rospy
import actionlib
import os

from mir_plan_loader_msgs.msg import LoadPlanAction, LoadPlanResult
from rosplan_dispatch_msgs.msg import CompletePlan, ActionDispatch
from diagnostic_msgs.msg import KeyValue

class LoadPlanServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('load_plan', LoadPlanAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        result = LoadPlanResult()
        try:
            lines = self.loadLines(goal.file)
            args = self.cleanLines(lines)
            plan = self.createPlanMsg(args)
            result.plan = plan
            result.success = True
            self.server.set_succeeded(result)
        except:
            result.success = False
            self.server.set_aborted(result)

    def createPlanMsg(self, args):
        complete_plan = CompletePlan()
        for i, action in enumerate(args):
            msg = ActionDispatch()
            msg.action_id = i
            msg.name = action.pop(0).lower()
            #print action
            for j, parameter in enumerate(action):
                msg.parameters.append(KeyValue(str(j + 1), parameter.lower()))
            #pub.publish(msg)
            complete_plan.plan.append(msg)
        return complete_plan

    def cleanLines(self, lines):
        args = []
        for line in lines:
            a = line[1:-1].split(' ')
            args.append(a)
        changed = True
        while changed:
            changed = False
            if len(args) < 2:
                break
            for i in range(0, len(args)-1):
                ca = args[i][0]
                na = args[i+1][0]
                if ca.upper() != "MOVE_BASE" or na.upper() != "MOVE_BASE":
                    continue
                changed = True
                args[i][3] = args[i+1][3]
                del args[i+1]
                break
        for a in args:
            print a
        return args

    def loadLines(self, filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
        lines = [x.strip() for x in lines]
        lines = filter(None, lines)
        return lines

if __name__ == '__main__':
    rospy.init_node('do_dishes_server')
    server = LoadPlanServer()
    rospy.spin()
