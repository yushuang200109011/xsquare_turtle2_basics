# turtle2_basic

Turtle2 robot basic controllers and HTTP API, packaged as a standalone Python library.

## Installation

```bash
pip install .
```

Or install directly from a git URL:

```bash
pip install git+https://github.com/<your-org>/turtle2_basic.git
```

### ROS dependencies

`turtle2_controller` relies on ROS packages (`rospy`, `cv_bridge`, `sensor_msgs`, `std_msgs`).
These are not installable via pip — they must be provided by the ROS environment (ROS 1 / catkin).
The `api` subpackage (`server` and `client`) has no ROS dependency and can be used standalone.

## Usage

```python
# Top-level shortcuts
from turtle2_basic import Turtle2Controller, robot_controller_access

# Controller subpackage
from turtle2_basic.turtle2_controller import Turtle2Controller, MovingController
from turtle2_basic.turtle2_controller.kinematics import kinematics
from turtle2_basic.turtle2_controller.logger import logger

# HTTP API
from turtle2_basic.api.client import RobotControllerClient
from turtle2_basic.api.server import RobotControllerServer

# Factory (reads ROBOT_TYPE env var, returns Turtle2Controller or MovingController)
robot = robot_controller_access()

# HTTP client
client = RobotControllerClient(base_url="http://<robot-ip>:8000")
client.health_check()
```

## Package structure

```
turtle2_basic/
├── __init__.py               # re-exports top-level symbols
├── turtle2_controller/       # hardware controller (requires ROS)
│   ├── Turtle2Controller.py  # RobotController, Turtle2Controller, MovingController
│   ├── controllers.py        # sub-system controllers (head, arms, chassis, lift, ...)
│   ├── kinematics.py         # FK / pose utilities
│   ├── logger.py             # rotating file + console logger
│   ├── sensors.py            # Camera (ROS image subscriber)
│   └── utils.py              # quaternion / pose helpers
└── api/                      # HTTP API (no ROS dependency)
    ├── server.py             # FastAPI server (auto-generates routes from controller)
    └── client.py             # HTTP client (auto-generates methods from server)
```

## License

Apache 2.0 — see [LICENSE](https://www.apache.org/licenses/LICENSE-2.0).
