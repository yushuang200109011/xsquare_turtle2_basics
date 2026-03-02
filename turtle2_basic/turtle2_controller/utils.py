import math

def quaternion_to_euler(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def pose_transformation(pos, att,radius=0.30):
    yaw = quaternion_to_euler(att)[2]
    cosx = math.cos(yaw)
    sinx = math.sin(yaw)
    ax = -cosx * radius
    ay = -sinx * radius
    return [pos[0] + ax, pos[1] + ay, yaw]

