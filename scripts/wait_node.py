#!/usr/bin/env python

import rospy
import actionlib
from wait_action.msg import WaitAction
from wait_action.msg import WaitFeedback
from datetime import *
from std_srvs.srv import Empty, EmptyResponse



class WaitServer(object):
    def __init__(self, name='wait_action'):
        self._name = name
        self._server = actionlib.SimpleActionServer(self._name,
                                                   WaitAction,
                                                   self._execute, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()

    def _preempt_cb(self):
        """
        overwrite this callback to receive preemption notification
        """
        return

    def _end_wait(self, req):
        if self._server.is_active():
            rospy.loginfo("Preempting sleep")
            self._server.preempt_request = True
            self._server.set_preempted()
        return EmptyResponse()



    def _execute(self, goal):
        end_wait_srv = rospy.Service('/wait_action/end_wait',
                                     Empty, self._end_wait)
        try:
            now = rospy.get_rostime()
            target = goal.wait_until
            if target.secs == 0:
                target = now + goal.wait_duration

            if target == now:
                # 72 hours
                a_really_long_time = rospy.Duration(60 * 60 * 24 * 3)
                target = now + a_really_long_time
                # rospy.loginfo("waiting a really long time")

            # for testing overruns
            # if True:
            #     a_really_long_time = rospy.Duration(60 * 60 * 24 * 3)
            #     target = now + a_really_long_time

            rospy.loginfo("target wait time: %s"
                          % datetime.fromtimestamp(target.secs))



            # how often to provide feedback
            feedback_secs = 5.0
            feedback = WaitFeedback()
            # how long to wait
            wait_duration_secs = target.secs - now.secs
            

            # the rate to publish feedback at/check time at
            r = rospy.Rate(1.0/feedback_secs)


            count = 1.0
            feedback_steps = wait_duration_secs/feedback_secs
            while not rospy.is_shutdown() and not self._server.is_preempt_requested() and rospy.get_rostime() < target:
                feedback.percent_complete = count/feedback_steps
                self._server.publish_feedback(feedback)
                count += 1
                r.sleep()

            rospy.loginfo("waited until: %s"
                          % datetime.fromtimestamp(rospy.get_rostime().secs))
        finally:
            end_wait_srv.shutdown()

        if self._server.is_preempt_requested():
            self._server.set_preempted()
        else:
            self._server.set_succeeded()


if __name__ == '__main__':

    rospy.init_node("wait_action")

    interruptible = rospy.get_param("~interruptible", True)
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    waiter = WaitServer(name=rospy.get_name())

    rospy.spin()
