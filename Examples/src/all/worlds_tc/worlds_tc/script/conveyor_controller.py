#!/usr/bin/env python

# Software License Agreement (Apache License)
#
# Copyright 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from osrf_gear.srv import ConveyorBeltControl, ConveyorBeltControlRequest, ConveyorBeltControlResponse
from osrf_gear.srv import PopulationControl, PopulationControlRequest, PopulationControlResponse


class ConveyorController(object):

    def __init__(self):

        self.start_conveyor()
        self.start_population()


    def start_population(self):
        population_service_name = '/conveyor/population/control'
        rospy.wait_for_service(population_service_name)
        try:
            start_population = rospy.ServiceProxy(population_service_name, PopulationControl)
            request = PopulationControlRequest()
            request.action = "resume"
            response = start_population(request)
            return response.success
        except rospy.ServiceException as ex:
            print("Service call failed: %s" % ex)

    def start_conveyor(self):

        conveyor_service_name = '/conveyor/conveyor/control'
        rospy.wait_for_service(conveyor_service_name)
        try:
            start_conveyor = rospy.ServiceProxy(conveyor_service_name, ConveyorBeltControl)
            request = ConveyorBeltControlRequest()
            request.state.power = 100.0
            response = start_conveyor(request)
            return response.success
        except rospy.ServiceException as ex:
            print("Service call failed: %s" % ex)

def main(sysargv=None):
    rospy.init_node("conveyor_controller_node")

    obj = ConveyorController()


if __name__ == '__main__':
    main()
