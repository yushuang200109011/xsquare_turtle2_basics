# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import time
import traceback
import requests
import inspect
from typing import List, Any, Tuple, Dict
# from turtle2_controller.Turtle2Controller import Turtle2Controller

import requests
import json
from typing import Any, Dict, Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotControllerClient: 
    def __init__(self, server_url: str = "http://localhost:8000"):
        self.server_url = server_url.rstrip('/')
        self.session = requests.Session()
        
        # Get available methods from server
        self.available_methods = self._get_available_methods()
        
        # Create dynamic methods
        self._create_dynamic_methods()
    
    def _get_available_methods(self) -> Dict[str, Any]:
        try:
            response = self.session.get(f"{self.server_url}/api/methods")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to get method list: {e}")
            return {}
    
    def _create_dynamic_methods(self):
        for method_name, method_info in self.available_methods.items():
            # Create dynamic method
            self._create_method(method_name, method_info)
    
    def _create_method(self, method_name: str, method_info: Dict[str, Any]):
        signature = method_info.get('signature', {})
        
        def dynamic_method(*args, **kwargs):
            if args:
                param_names = list(signature.keys())
                for i, arg in enumerate(args):
                    if i < len(param_names):
                        kwargs[param_names[i]] = arg
            
            return self._call_remote_method(method_name, kwargs)
        
        dynamic_method.__doc__ = method_info.get('doc', f'Call remote method: {method_name}')
        dynamic_method.__name__ = method_name
        
        setattr(self, method_name, dynamic_method)
    
    def _call_remote_method(self, method_name: str, params: Dict[str, Any]) -> Any:
        try:
            if not params:
                response = self.session.get(f"{self.server_url}/api/{method_name}")
            else:
                response = self.session.post(
                    f"{self.server_url}/api/{method_name}",
                    json=params,
                    headers={'Content-Type': 'application/json'}
                )
            
            response.raise_for_status()
            result = response.json()
            
            if result.get('success', False):
                return result.get('result')
            else:
                raise Exception(f"Failed to call remote method: {result}")
                
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to call remote method {method_name}: {e}")
            raise Exception(f"Network request failed: {e}")
    
    def get_available_methods(self) -> Dict[str, Any]:
        return self.available_methods
    
    def health_check(self) -> Dict[str, Any]:
        try:
            response = self.session.get(f"{self.server_url}/api/health")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            logger.error(f"Health check failed: {e}")
            return {"status": "unhealthy", "error": str(e)}

if __name__ == "__main__":
    client = RobotControllerClient("http://localhost:8000")
    
    print("Health check:", client.health_check())
    
    print("\nAvailable methods:")
    for method_name, method_info in client.get_available_methods().items():
        print(f"  {method_name}: {method_info['doc']}")
    
    try:
        print("===== Test all status get functions =====")

        # 1. test head control
        print("\n1. Test head control:")
        print("Set head position pitch=0.5, yaw=0.0")
        head_result = client.head_control([0.1, 0.0])
        print("Current head position:", head_result)
        
        # 2. test lift control
        print("\n2. Test lift control:")
        print("Set lift height to 0.4")
        lift_result = client.lift_control(0.4)
        print("Current lift height:", lift_result)
        
        # 3. test arms control
        print("\n3. test arms control:")
        # 3.1 test arms control directly
        print("Set left arm position [0.1,0.1,0.1,0.0,0.0,0.0,1.0], right arm position [0.1,-0.1,0.1,0.0,0.0,0.0,1.0]")
        client.arms_control([0.01, 0.01, 0.01, 0.0, 0.0, 0.0, 1.0], [0.01, -0.01, 0.01, 0.0, 0.0, 0.0, 1.0])
        
        # 3.2 test arms zero
        print("\n3.2 test arms zero:")
        print("Double arms zero")
        client.arms_zero()
        print("Zero done")
        
        # 4. test virtual zero point setting
        print("\n4. Test virtual zero point setting:")
        print("Set current chassis position as virtual zero point")
        client.chassis_set_current_pose_as_virtual_zero()
        print("Virtual zero point setting done")
        
        # 5. test all setting results
        print("\n5. test all setting results:")
        # 5.1 test head status
        head_data = client.head_data()
        print(f"Head position - pitch: {head_data[0]}, yaw: {head_data[1]}")
        
        # 5.2 test lift status
        lift_data = client.lift_data()
        print(f"Lift height: {lift_data}")
        
        # 5.3 test arms status
        arm_l, arm_r = client.arms_data()
        print(f"Left arm end: {arm_l}")
        print(f"Right arm end: {arm_r}")
        
        # 5.4 test chassis relative position
        rel_pose = client.chassis_rel_pose_data()
        print(f"Chassis relative position: x={rel_pose[0]}, y={rel_pose[1]}, yaw={rel_pose[2]}")

        print("\n===== All setting position functions test done =====")

    except Exception as e:
        stack_trace = traceback.format_exc()
        print(f"Error in test: {str(e)}")
        print(stack_trace)
        print(f"\nTest failed: {str(e)}")