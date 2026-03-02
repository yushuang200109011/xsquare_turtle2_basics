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

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Any, Dict, List, Optional
import inspect
import asyncio
import json

import os
import traceback
import traceback
from turtle2_basic.turtle2_controller.Turtle2Controller import Turtle2Controller
import uvicorn
import logging
from datetime import datetime

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotControllerServer:
    
    def __init__(self, controller_class, controller_init_args=None, controller_init_kwargs=None):
        self.controller_class = controller_class
        self.controller_init_args = controller_init_args or []
        self.controller_init_kwargs = controller_init_kwargs or {}
        
        # Create FastAPI application
        self.app = FastAPI(
            title="Robot Controller API",
            description="Automatically generated robot controller API",
            version="1.0.0"
        )
        
        # Initialize controller instance
        self.controller = None
        self._initialize_controller()
        
        # Automatically discover and register all methods
        self._register_methods()
        
        # Add base routes
        self._add_base_routes()
    
    def _initialize_controller(self):
        """Initialize controller instance"""
        try:
            self.controller = self.controller_class(
                *self.controller_init_args, 
                **self.controller_init_kwargs
            )
            logger.info(f"Controller {self.controller_class.__name__} initialized successfully")
        except Exception as e:
            logger.error(f"Controller initialization failed: {e}")
            raise
    
    def _get_method_signature(self, method):
        """Get method signature information"""
        sig = inspect.signature(method)
        params = {}
        
        for param_name, param in sig.parameters.items():
            if param_name == 'self':
                continue
                
            param_info = {
                'name': param_name,
                'required': param.default == inspect.Parameter.empty,
                'default': param.default if param.default != inspect.Parameter.empty else None,
                'annotation': str(param.annotation) if param.annotation != inspect.Parameter.empty else 'Any'
            }
            params[param_name] = param_info
        
        return params
    
    def _register_methods(self):
        """Automatically discover and register all public methods of the controller"""
        print(f"Starting to get methods...")
        methods = inspect.getmembers(self.controller, predicate=inspect.ismethod)
        for method_name, method in methods:
            if method_name.startswith('_'):
                continue
            
            method_signature = self._get_method_signature(method)
            
            self._create_dynamic_route(method_name, method, method_signature)
            
            print(f"Registering method: {method_name}")
    
    def _create_dynamic_route(self, method_name, method, method_signature):
        """Create dynamic route for each method"""
        
        # Create request model
        request_fields = {}
        for param_name, param_info in method_signature.items():
            if param_info['required']:
                request_fields[param_name] = (Any, ...)
            else:
                request_fields[param_name] = (Any, param_info['default'])
        
        # Dynamically create Pydantic model
        RequestModel = None
        if request_fields:
            RequestModel = type(f"{method_name.capitalize()}Request", (BaseModel,), {
                '__annotations__': {k: v[0] for k, v in request_fields.items()},
                **{k: v[1] for k, v in request_fields.items() if v[1] is not ...}
            })
        
        # Create route handler function
        async def route_handler(request: RequestModel = None):
            try:
                # Prepare parameters
                if request:
                    kwargs = request.dict()
                    # Filter out None values (for optional parameters)
                    kwargs = {k: v for k, v in kwargs.items() if v is not None}
                else:
                    kwargs = {}
                
                # Call actual method
                if asyncio.iscoroutinefunction(method):
                    result = await method(**kwargs)
                else:
                    result = method(**kwargs)
                print(type(result))
                # Special handling for binary results
                if isinstance(result, bytes):
                    from fastapi.responses import Response
                    return Response(
                        content=result,
                        media_type="application/octet-stream"  # or "image/jpeg"
                    )
                
                # Return result
                return {
                    "success": True,
                    "result": result,
                    "method": method_name,
                    "timestamp": datetime.now().isoformat()
                }
                

            except Exception as e:
                # Record complete error stack
                logger.error(f"Error calling method {method_name}", exc_info=True)
                
                # Get stack trace and ensure it is serializable
                stack_trace = traceback.format_exc()
                print(stack_trace)
                
                # Ensure all error details are basic types
                error_detail = {
                    "error": str(e),
                    "type": type(e).__name__,
                    "method": method_name,
                    "timestamp": datetime.now().isoformat()
                }
                
                # Only return stack trace in debug mode
                if os.getenv("DEBUG", "").lower() in ("true", "1", "t"):
                    error_detail["stack_trace"] = stack_trace
                
                raise HTTPException(
                    status_code=500,
                    detail=error_detail
                )
        
        # Register route
        if RequestModel:
            self.app.post(f"/api/{method_name}")(route_handler)
        else:
            # For methods with no parameters, use GET request
            async def get_handler():
                return await route_handler()
            self.app.get(f"/api/{method_name}")(get_handler)
    
    def _add_base_routes(self):
        """Add base routes"""
        
        @self.app.get("/")
        async def root():
            return {"message": "Robot Controller API Server", "status": "running"}
        
        @self.app.get("/api/methods")
        async def get_methods():
            """Get all available methods and their signatures"""
            methods = inspect.getmembers(self.controller, predicate=inspect.ismethod)
            method_info = {}
            
            for method_name, method in methods:
                if method_name.startswith('_'):
                    continue
                
                method_signature = self._get_method_signature(method)
                method_info[method_name] = {
                    "signature": method_signature,
                    "doc": inspect.getdoc(method) or "No documentation",
                    "endpoint": f"/api/{method_name}"
                }
            
            return method_info
        
        @self.app.get("/api/health")
        async def health_check():
            """Health check"""
            return {
                "status": "healthy",
                "controller": self.controller_class.__name__,
                "timestamp": datetime.now().isoformat()
            }

# Example usage
if __name__ == "__main__":
    server = RobotControllerServer(
        controller_class=Turtle2Controller,
        controller_init_kwargs={'init_node': True}
    )
    
    uvicorn.run(
        server.app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )