from gz.msgs10.pose_pb2 import Pose


from gz.msgs10.stringmsg_pb2 import StringMsg
from tf_transformations import quaternion_from_euler
from gz.transport13 import Node
 
def main():
    node = Node()
    service_name = "/world/empty/set_pose_vector"
    msg = Pose()
    # request.data = "Hello world"
    response = StringMsg()
    timeout = 5000
    quaternion = quaternion_from_euler(0, 0, 0)

    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = 5
    msg.pose.pose.position.y = 5
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2] 
    msg.pose.pose.orientation.w = quaternion[3]

 
    result, response = node.request(service_name, msg, Pose, StringMsg, timeout)
    print("Result:", result)