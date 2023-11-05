import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

class UR5ControlService(Node):
def **init**(self):
super().**init**('ur5_control_service')
self.ik_service = self.create_client(GetPositionIK, 'compute_ik')
while not self.ik_service.wait_for_service(timeout_sec=1.0):
self.get_logger().info('Service "compute_ik" not available. Waiting...')

```
    self.service = self.create_service(String, 'ur5_control', self.control_ur5)

def control_ur5(self, request, response):
    pose = self.compute_pose(request.data)
    if pose:
        self.move_to_pose(pose)
        response.data = "Success"
    else:
        response.data = "Failed to compute IK solution"
    return response

def compute_pose(self, input_str):
    # Parse the input string to extract the position and orientation data
    # Here, you should parse the input_str and create a PoseStamped message
    # with the desired position and orientation.

    # Example: For a pose at (x=0.5, y=0.1, z=0.6) and orientation (roll=0, pitch=0, yaw=0)
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0.1
    pose.pose.position.z = 0.6
    pose.pose.orientation.w = 1.0

    return pose

def move_to_pose(self, pose):
    # Use MoveIt to plan and execute the motion to the desired pose
    ik_request = GetPositionIK.Request()
    ik_request.group_name = "manipulator"  # Adjust this to match your MoveIt configuration
    ik_request.pose_stamped = pose

    future = self.ik_service.call_async(ik_request)
    rclpy.spin_until_future_complete(self, future)
    if future.result() is not None:
        self.get_logger().info("IK solution found")
        # Code to execute the planned motion using a MoveIt planning and execution interface

    else:
        self.get_logger().info("No IK solution found")

```

def main(args=None):
rclpy.init(args=args)
ur5_control_service = UR5ControlService()
rclpy.spin(ur5_control_service)
ur5_control_service.destroy_node()
rclpy.shutdown()

if **name** == '**main**':
main()