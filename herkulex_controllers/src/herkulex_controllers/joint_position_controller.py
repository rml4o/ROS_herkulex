# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'
__credits__ = 'Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import rospy
import math

from sensor_msgs.msg import JointState
from herkulex_controllers.joint_controller import JointController

class JointPositionController(JointController):
    def __init__(self, hkx_io, controller_namespace, port_namespace):
        JointController.__init__(self, hkx_io, controller_namespace, port_namespace)
        
        self.motor_id = rospy.get_param(self.controller_namespace + '/motor/id')
        self.offset_rad = rospy.get_param(self.controller_namespace + '/motor/offset_rad')        
        self.flipped = rospy.get_param(self.controller_namespace + '/motor/flipped')
        self.angle_max_rad = rospy.get_param(self.controller_namespace + '/motor/max_angle_rad')
        self.angle_min_rad = rospy.get_param(self.controller_namespace + '/motor/min_angle_rad')
        #check if min/max make sense
        if self.angle_max_rad < self.angle_min_rad:
            self.angle_min_rad, self.angle_max_rad =self.angle_max_rad, self.angle_min_rad
            rospy.set_param(self.controller_namespace + '/motor/min_angle_rad', self.angle_min_rad)
            rospy.set_param(self.controller_namespace + '/motor/max_angle_rad', self.angle_max_rad)
            rospy.logwarn('The lower and upper software angle limits for joint %s are in the wrong order, and have been adjusted accordingly.' %(self.joint_name))
        
        self.joint_state = JointState(name=(self.joint_name,))

    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('herkulex/%s/connected_ids' % self.port_namespace, [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False
            
        #check consistency of the max/min angle_rad (ofter offset and flip) with HW angle limits
        HW_min_angle_limit_in_rad =self.deg_to_rad( rospy.get_param('herkulex/%s/%s/min_HW_angle' %(self.port_namespace,self.motor_id)))
        HW_max_angle_limit_in_rad =self.deg_to_rad( rospy.get_param('herkulex/%s/%s/max_HW_angle' %(self.port_namespace,self.motor_id)))
        
        if self.flipped:
            HW_max_angle_limit_in_rad, HW_min_angle_limit_in_rad = HW_min_angle_limit_in_rad, HW_max_angle_limit_in_rad

        rad_safety_margin=0.015
        
        if self.angle_min_rad < ( HW_min_angle_limit_in_rad + rad_safety_margin):
            rospy.logwarn('The lower software angle limit for joint %s (%s rad) is below the implied hardware limit (%s) plus safety margin (%s), and has been adjusted accordingly.' %(self.joint_name, self.angle_min_rad, HW_min_angle_limit_in_rad, rad_safety_margin))
            self.angle_min_rad = HW_min_angle_limit_in_rad + rad_safety_margin
            rospy.set_param(self.controller_namespace + '/motor/min_angle_rad', self.angle_min_rad)

        if self.angle_max_rad > (HW_max_angle_limit_in_rad - rad_safety_margin):
            rospy.logwarn('The upper software angle limit for joint %s (%s rad) is above the implied hardware limit (%s) minus safety margin (%s), and has been adjusted accordingly.' %(self.joint_name, self.angle_max_rad, HW_max_angle_limit_in_rad, rad_safety_margin))
            self.angle_max_rad = HW_max_angle_limit_in_rad - rad_safety_margin
            rospy.set_param(self.controller_namespace + '/motor/max_angle_rad', self.angle_max_rad)

        if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)
            
#TODO
#        self.joint_max_speed = rospy.get_param(self.controller_namespace + '/joint_max_speed', self.MAX_VELOCITY)
#        
#        if self.joint_max_speed < self.MIN_VELOCITY: self.joint_max_speed = self.MIN_VELOCITY
#        elif self.joint_max_speed > self.MAX_VELOCITY: self.joint_max_speed = self.MAX_VELOCITY
#        
#        if self.joint_speed < self.MIN_VELOCITY: self.joint_speed = self.MIN_VELOCITY
#        elif self.joint_speed > self.joint_max_speed: self.joint_speed = self.joint_max_speed
        
#        self.set_speed(self.joint_speed)
        
        return True
    #more than just a rad<->deg conversion, this function also handles flipping and offset
    def rad_to_deg(self, angle):
        angle_deg = (angle + self.offset_rad) * 180 / math.pi
        return -angle_deg if self.flipped else angle_deg
        
    #more than just a rad<->deg conversion, this function also handles flipping and offset
    def deg_to_rad(self, deg):
        rad = deg * math.pi / 180
        rad = -rad if self.flipped else rad
        return rad

    def set_torque_enable(self, torque_enable):
        mcv = (self.motor_id,)
        if torque_enable:
            self.hkx_io.enable_torque(mcv)
        else:
            self.hkx_io.disable_torque(mcv)

    def set_speed(self, speed):
        self.joint_speed = speed


    def set_torque_limit(self, max_torque):
        pass
        #TODO: reinstate check?
#        if max_torque > 1: max_torque = 1.0         # use all torque motor can provide
#        elif max_torque < 0: max_torque = 0.0       # turn off motor torque
#        raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
#        mcv = (self.motor_id, raw_torque_val)
#        self.hkx_io.set_torque_limit([mcv])

    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                self.joint_state.position = (self.deg_to_rad(state.position),)
                self.joint_state.velocity = (state.speed * math.pi / 180,)
                self.joint_state.effort = (state.load,)
                self.joint_state_pub.publish(self.joint_state)

    def process_command(self, msg):
        angle = max(min(msg.data, self.angle_max_rad), self.angle_min_rad)
        if msg.data != angle:
            rospy.logwarn('Goal position for joint %s modified to remain within limits (%s rad done vs %s rad requested)' %(self.joint_name, angle, msg.data))
        mcv = self.rad_to_deg(angle)
        if self.joint_speed > 0:
            exectime = 2 #TODO: compute based on speed
            values = ((mcv, exectime, 0),)
            ids = (self.motor_id,)
            gh = dict(list(zip(ids, values)))
            self.hkx_io.set_joint_jog_load(gh)

    def process_clear_errors(self, req):
        self.hkx_io.clear_errors((self.motor_id,))
        return []