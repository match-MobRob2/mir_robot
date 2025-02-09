#!/usr/bin/env python3
# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Sönke Niemann

import time
import sys

import rclpy
from rclpy.node import Node

import mir_restapi.mir_restapi_lib
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


class MirRestAPIServer(Node):
    def __init__(self):
        super().__init__('mir_restapi_server')
        self.get_logger().info("started")

        # parameters: hostname, api_token
        self.declare_parameter('mir_hostname', "")
        self.hostname = self.get_parameter('mir_hostname').get_parameter_value().string_value
        self.declare_parameter('mir_restapi_auth', "")
        self.auth = self.get_parameter('mir_restapi_auth').get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.api_handle = None

        self.setup_api_handle()

        if self.api_handle is None:
            self.get_logger().warn(
                """
            Hostname and API token are not set! Run as follows:

            ros2 run mir_restapi mir_restapi_server
            --ros-args -p mir_hostname:='MIR_IP_ADDR' -p mir_restapi_auth:='YOUR_API_KEY'
            """
            )

    def setup_api_handle(self):
        if self.hostname != "" and self.auth != "":
            self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(self.get_logger(), self.hostname, self.auth)
            self.get_logger().info("created MirRestAPI handle")
            self.create_services()
            self.get_logger().info("created services")

    def parameters_callback(self, params):
        for param in params:
            if param.name == "mir_restapi_auth":
                self.get_logger().info("Received auth token")
                self.auth = param.value
            if param.name == "mir_hostname":
                self.get_logger().info("Set mir hostname")
                self.hostname = param.value
        self.setup_api_handle()
        return SetParametersResult(successful=True)

    def create_services(self):
        self.create_service(Trigger, 'mir_sync_time', self.sync_time_callback)
        self.get_logger().info("Listening on 'mir_sync_time'")

        self.create_service(Trigger, 'mir_get_status', self.get_status_callback)
        self.get_logger().info("Listening on 'mir_get_status'")

        self.create_service(Trigger, 'mir_get_sounds', self.get_sounds_callback)
        self.get_logger().info("Listening on 'mir_get_sounds'")

        self.create_service(Trigger, 'mir_is_emergency_halt', self.is_emergency_halt_callback)
        self.get_logger().info("Listening on 'mir_is_emergency_halt'")

        self.create_service(Trigger, 'mir_get_missions', self.get_missions_callback)
        self.get_logger().info("Listening on 'mir_get_missions'")

        self.create_service(Trigger, 'mir_honk', self.honk_callback)
        self.get_logger().info("Listening on 'mir_honk'")

        self.create_service(Trigger, 'mir_get_system_info', self.get_system_info_callback)
        self.get_logger().info("Listening on 'mir_get_system_info'")

        self.create_service(Trigger, 'mir_get_settings', self.get_settings_callback)
        self.get_logger().info("Listening on 'mir_get_settings'")

    def test_api_connection(self):
        if self.api_handle is None:
            return -1

        self.get_logger().info('REST API: Waiting for connection')
        i = 1
        while not self.api_handle.is_connected():
            if not rclpy.ok():
                sys.exit(0)
            if i > 5:
                self.get_logger().error('REST API: Could not connect, giving up')
                return 0
            i += 1
            time.sleep(1)
        return 1

    def reponse_api_handle_not_exists(self, response):
        response.success = False
        response.message = 'API token and/or hostname not set yet'
        self.get_logger().error(response.message)
        return response

    def call_restapi_function(self, service_fct, request, response, args=None):
        if self.test_api_connection() == -1:
            response = self.reponse_api_handle_not_exists(response)
            return response
        if self.api_handle.is_connected(print=False):
            if args is None:
                response.message = str(service_fct())
            else:
                response.message = str(service_fct(args))
            if "Error" in response.message:
                response.success = False
            else:
                response.success = True
            return response
        else:
            response.success = False
            response.message = "ERROR: Couldn't connect to REST API"
        self.get_logger().error(response.message)
        return response

    def sync_time_callback(self, request, response):
        self.get_logger().info('Syncing host time with REST API...')
        response = self.call_restapi_function(self.api_handle.sync_time, request, response)
        return response

    def get_status_callback(self, request, response):
        self.get_logger().info('Getting status from REST API...')
        response = self.call_restapi_function(self.api_handle.get_status, request, response)
        return response

    def get_sounds_callback(self, request, response):
        self.get_logger().info('Getting sounds from REST API...')
        response = self.call_restapi_function(self.api_handle.get_sounds, request, response)
        return response

    def is_emergency_halt_callback(self, request, response):
        self.get_logger().info('Checking REST API for emergency halt...')
        response = self.call_restapi_function(self.api_handle.get_state_id, request, response)

        if response.success:
            state_id = int(response.message)
            #  self.get_logger().info("Returned state_id as %i" % state_id)
            STATE_ID_EMERGENCY = 10
            if state_id == STATE_ID_EMERGENCY:
                response.message = str(True)
                self.get_logger().info("Emergency Halt")
            else:
                response.message = str(False)
                # self.get_logger().info("no emergency halt")
        return response

    def get_missions_callback(self, request, response):
        self.get_logger().info('Getting missions from REST API...')
        response = self.call_restapi_function(self.api_handle.get_missions, request, response)
        return response

    def honk_callback(self, request, response):
        self.get_logger().info('Honking horn over REST API...')

        mission_name = "honk"

        queue_success, mission_queue_id = self.api_handle.add_mission_to_queue(mission_name)
        if not queue_success:
            response.message = "Honking failed due to mission queue error"
            self.get_logger().error(response.message)
            response.success = False
            return response
        self.get_logger().info("Put honk mission into queue with mission_queue_id={}".format(mission_queue_id))

        emerg_response = self.is_emergency_halt_callback(request, response)
        if emerg_response.message == str(True):
            response.message = "Can't honk, emergency halt"
            self.get_logger().error(response.message)
            response.success = False
        else:
            response.message = "Honking"
            self.get_logger().info(response.message)
            STATE_ID_RUN_MISSION = 3
            STATE_ID_PAUSE = 4

            self.api_handle.set_state_id(STATE_ID_RUN_MISSION)

            while not self.api_handle.is_mission_done(mission_queue_id):
                time.sleep(1)

            self.api_handle.set_state_id(STATE_ID_PAUSE)
            self.api_handle.http.__del__()
            response.success = True
        return response

    def get_system_info_callback(self, request, response):
        self.get_logger().info('Getting system info from REST API...')
        response = self.call_restapi_function(self.api_handle.get_system_info, request, response)
        return response

    def get_settings_callback(self, request, response):
        self.get_logger().info('Getting settings from REST API...')
        response = self.call_restapi_function(self.api_handle.get_all_settings, request, response)
        return response


def main(args=None):
    rclpy.init(args=args)

    mir_restapi_server = MirRestAPIServer()

    rclpy.spin(mir_restapi_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
