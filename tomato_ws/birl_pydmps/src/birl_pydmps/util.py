import rospy
from visualization_msgs.msg import (
    Marker
)
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Point,Pose
import numpy as np
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import ipdb

def plot_3d_demo(mat):
    fig = plt.figure(0,figsize=(10,8))
    ax = fig.gca(projection='3d')
    ax.plot(mat[:, 0], mat[:, 1], mat[:, 2], linewidth=2, linestyle='-')
    ax.scatter(mat[0, 0], mat[0, 1], mat[0, 2], marker="o",s=400,color="g")
    ax.scatter(mat[-1, 0], mat[-1, 1], mat[-1, 2], marker="^",s=400,color="r")
    plt.show()

def filter_static_points(mat):
    last = mat[0]
    new_mat = [last]
    for idx in range(mat.shape[0]):
        if np.linalg.norm(mat[idx]-last) < 0.01:
            pass
        else:
            new_mat.append(mat[idx])
            last = mat[idx] 
    return np.array(new_mat)

def set_trajectory_speed(traj, speed):
       # Create a new trajectory object
       new_traj = RobotTrajectory()
       
       # Initialize the new trajectory to be the same as the input trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
       
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
       
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
        
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)
       
       # Cycle through all points and joints and scale the time from start,
       # as well as joint speed and acceleration
       time_list = []
       for i in range(n_points):
           point = JointTrajectoryPoint()
           
           # The joint positions are not scaled so pull them out first
           point.positions = traj.joint_trajectory.points[i].positions

           # Next, scale the time_from_start for this point
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start
           t = rospy.Time(point.time_from_start.secs, point.time_from_start.nsecs)
        #    ipdb.set_trace()
           time_list.append(t.to_time())

           # Initialize the joint velocities for this point
           point.velocities = [speed] * n_joints
           
           # Get the joint accelerations for this point
           point.accelerations = [speed / 4.0] * n_joints
        
           # Store the scaled trajectory point
           points[i] = point

       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points
    #    ipdb.set_trace()
       # plt.plot(time_list)
       # plt.show()
       # Return the new trajecotry
       return new_traj
   

def send_traj_point_marker(marker_pub,frameid, pose, id, rgba_tuple):
    marker = Marker()
    marker.header.frame_id = frameid
    marker.header.stamp = rospy.Time.now()
    marker.ns = "traj_point" 
    marker.id = id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = rgba_tuple[0]
    marker.color.g = rgba_tuple[1]
    marker.color.b = rgba_tuple[2]
    marker.color.a = rgba_tuple[3]
    marker.lifetime = rospy.Duration()
    marker_pub.publish(marker) 
    while marker_pub.get_num_connections < 1:
        rospy.sleep(1)
        pass

def execute_plan(group,plan):
    group.set_start_state_to_current_state()
    group.execute(plan,wait=True)

def fK_point_calculate(group,JointAngle):
    group.set_joint_value_target(JointAngle)
    group.set_start_state_to_current_state()
    plan = group.plan()
    return plan


def iK_point_calculate(group,pose):
    group.set_pose_target(pose)
    group.set_start_state_to_current_state()
    plan = group.plan()
    return plan 

def iK_point_calculate(group,point):
    pose =Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]
    pose.orientation.x = point[3]
    pose.orientation.y = point[4]
    pose.orientation.z = point[5]
    pose.orientation.w = point[6]
    group.set_pose_target(pose)
    group.set_start_state_to_current_state()
    plan = group.plan()
    return plan 

def ik_cartesain_path(group, mat):
    wpose = Pose()
    waypoints = []  
    waypoints.append(group.get_current_pose().pose)
    for idx in xrange(mat.shape[0]):
        wpose = Pose()
        wpose.position.x = mat[idx,0]
        wpose.position.y = mat[idx,1]
        wpose.position.z = mat[idx,2]
        wpose.orientation.x = mat[idx,3]
        wpose.orientation.y = mat[idx,4]
        wpose.orientation.z = mat[idx,5]
        wpose.orientation.w = mat[idx,6]
        waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    
    return plan

def speed_set(group,factor):
    group.set_max_velocity_scaling_factor(factor)
    group.set_max_acceleration_scaling_factor(factor)

def fk_compute_service(position):
    rospy.wait_for_service('compute_fk',timeout=3)
    try:
        req = GetPositionFKRequest()
        rs = RobotState()
        req.fk_link_names = ["tool0"]
        joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
        rs.joint_state.name = joint_names
        rs.joint_state.position = position
        req.robot_state = rs
        client = rospy.ServiceProxy('compute_fk', GetPositionFK)
        res = client(req)
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s"%e)
    # pdb.set_trace()
    return res.pose_stamped[0].pose

def tool0TgripperTransform(tool0Pose):
    pos = tool0Pose.position
    ori = tool0Pose.orientation
    transform_mat = np.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z,ori.w)))
    # pdb.set_trace()
    rot_1 = np.array((
        [[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, z_offset],
        [0, 0, 0, 1]]
        ), dtype=np.float64)

    # rotate z axis 45 degree and move z_offset
    aa = (2**0.5)/2
    rot_2 = np.array((
        [[-aa, -aa, 0, 0],
        [aa, -aa, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
        ), dtype=np.float64)
    # pdb.set_trace()   
    new_mat = np.dot(np.dot(transform_mat,rot_1), rot_2)
    # new_mat = np.dot(transform_mat,rot_1)
    new_trans = translation_from_matrix(new_mat)
    new_quat = quaternion_from_matrix(new_mat)   
    
    new_pose = Pose()
    new_pose.position.x = new_trans[0]
    new_pose.position.y = new_trans[1]
    new_pose.position.z = new_trans[2]
    new_pose.orientation.x = new_quat[0]
    new_pose.orientation.y = new_quat[1]
    new_pose.orientation.z = new_quat[2]
    new_pose.orientation.w = new_quat[3]
    # broadcaster = tf.TransformBroadcaster()
    # broadcaster.sendTransform(
    #         new_trans,
    #         new_quat,
    #         rospy.Time.now(),
    #         'gripper_pose',
    #         'base', 
    #         )
    return new_pose