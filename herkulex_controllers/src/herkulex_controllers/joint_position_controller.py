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
        self.initial_position_deg = rospy.get_param(self.controller_namespace + '/motor/init')
        self.min_angle_deg = rospy.get_param(self.controller_namespace + '/motor/min')
        self.max_angle_deg = rospy.get_param(self.controller_namespace + '/motor/max')
        
        self.flipped = self.min_angle_deg > self.max_angle_deg
        
        self.joint_state = JointState(name=(self.joint_name,))

    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('herkulex/%s/connected_ids' % self.port_namespace, [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False
            
        #set angle limits in hardware TODO: find a way to have the value updated in the params / controller_manager as well...
        #values = ((min(self.min_angle_deg, self.max_angle_deg), max(self.min_angle_deg, self.max_angle_deg)),)
        #ids = (self.motor_id,)
        #gh = dict(list(zip(ids, values)))
        #self.hkx_io.set_angle_limit(gh)
        
        
        if self.flipped:
            self.min_angle = (self.initial_position_deg - self.min_angle_deg) * math.pi / 180
            self.max_angle = (self.initial_position_deg - self.max_angle_deg) * math.pi / 180
        else:
            self.min_angle = (self.min_angle_deg - self.initial_position_deg) * math.pi / 180
            self.max_angle = (self.max_angle_deg - self.initial_position_deg) * math.pi / 180
            
        if self.compliance_slope is not None: self.set_compliance_slope(self.compliance_slope)
        if self.compliance_margin is not None: self.set_compliance_margin(self.compliance_margin)
        if self.compliance_punch is not None: self.set_compliance_punch(self.compliance_punch)
        if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)
            
#TODO: should send the angle limit to the servo ? or to be done by the controller manager ?
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

    def pos_rad_to_deg(self, pos_rad):
        if pos_rad < self.min_angle: pos_rad = self.min_angle
        elif pos_rad > self.max_angle: pos_rad = self.max_angle
        return self.rad_to_deg(pos_rad, self.initial_position_deg, self.flipped)

    def set_torque_enable(self, torque_enable):
        mcv = (self.motor_id,)
        if torque_enable:
            self.hkx_io.enable_torque(mcv)
        else:
            self.hkx_io.disable_torque(mcv)

    def set_speed(self, speed):
        self.joint_speed = speed

    def set_compliance_slope(self, slope):
        pass
#        #TODO: reinstate check?
#        if slope < DXL_MIN_COMPLIANCE_SLOPE: slope = DXL_MIN_COMPLIANCE_SLOPE
#        elif slope > DXL_MAX_COMPLIANCE_SLOPE: slope = DXL_MAX_COMPLIANCE_SLOPE
#        mcv = (self.motor_id, slope, slope)
#        self.hkx_io.set_compliance_slope([mcv])

    def set_compliance_margin(self, margin):
        pass
#        #TODO: reinstate check?
#        if margin < DXL_MIN_COMPLIANCE_MARGIN: margin = DXL_MIN_COMPLIANCE_MARGIN
#        elif margin > DXL_MAX_COMPLIANCE_MARGIN: margin = DXL_MAX_COMPLIANCE_MARGIN
#        else: margin = int(margin)
#        mcv = (self.motor_id, margin, margin)
#        self.hkx_io.set_multi_compliance_margins([mcv])

    def set_compliance_punch(self, punch):
        pass
#        #TODO: reinstate check?
#        if punch < DXL_MIN_PUNCH: punch = DXL_MIN_PUNCH
#        elif punch > DXL_MAX_PUNCH: punch = DXL_MAX_PUNCH
#        else: punch = int(punch)
#        mcv = (self.motor_id, punch)
#        self.hkx_io.set_punch([mcv])

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
                self.joint_state.position = (self.deg_to_rad(state.position, self.initial_position_deg, self.flipped),)
                self.joint_state.velocity = (state.speed * math.pi / 180,)
                self.joint_state.effort = (state.load,)
                self.joint_state_pub.publish(self.joint_state)

    def process_command(self, msg):
        angle = msg.data
        mcv = self.pos_rad_to_deg(angle)
        spdeg = self.joint_speed * 180 / math.pi
        if spdeg > 0:
            exectime = 1 #TODO: compute based on speed
            values = ((mcv, exectime, 0),)
            ids = (self.motor_id,)
            gh = dict(list(zip(ids, values)))
            self.hkx_io.set_joint_jog_load(gh)