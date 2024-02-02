#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, AccelWithCovarianceStamped, Vector3Stamped, TwistStamped, Vector3
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Imu

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from drone_model import export_drone_ode_model
from utils import plot_pendulum, plot_drone
import numpy as np
import scipy.linalg
import scipy.interpolate
from casadi import vertcat

# variables

current_state = State()

curr_state = np.zeros(9)

N_horizon = 50
Tf = 5


x0 = curr_state

aMax = 5
vMax = 5










class OCP:
    def __init__(self, N_horizon, Tf, aMax, vMax):
        self.N_horizon = N_horizon
        self.Tf = Tf
        self.aMax = aMax
        self.vMax = vMax
        
        
        
        
    


def setup(x0, aMax, vMax, N_horizon, Tf):
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_drone_ode_model()
    ocp.model = model

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.dims.N = N_horizon

    # set cost module
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    
    Q_mat = np.eye((9))
    Q_mat[0,0] = 2
    Q_mat[1,1] = 2
    Q_mat[2,2] = 2
    
    R_mat = np.eye(3)

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x
    
    
    
    yref = np.zeros((ny, ))
    ocp.cost.yref  = yref
    
    yref_e = np.zeros((ny_e, ))
    ocp.cost.yref_e = yref_e

    

    # set constraints
    ocp.constraints.lbu = np.array([-aMax, -aMax, -aMax])
    ocp.constraints.ubu = np.array([+aMax, +aMax, +aMax])
    
    ocp.constraints.lbx = np.array([-vMax, -vMax, -vMax])
    ocp.constraints.ubx = np.array([+vMax, +vMax, +vMax])
    
    ocp.constraints.lbx_e = np.array([-vMax, -vMax, -vMax])
    ocp.constraints.ubx_e = np.array([+vMax, +vMax, +vMax])
    
 
    ocp.constraints.x0 = x0
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.idxbx = np.array([3, 4, 5])
    
    ocp.constraints.idxbx_e = np.array([3,4,5])


    ## slack for constraints
    ocp.constraints.lsbx = np.array([-1,-1,-1])
    ocp.constraints.usbx = np.array([+1,+1,+1])
    ocp.constraints.idxsbx = np.array([0,1,2])
    #
    ns = 3
    ocp.cost.zl = 10e-1 * np.ones((ns,)) # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
    ocp.cost.Zl = np.ones((ns,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
    ocp.cost.zu = 10e-1 * np.ones((ns,))    
    ocp.cost.Zu = np.ones((ns,))  
    

    
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    

    ocp.solver_options.qp_solver_cond_N = N_horizon

    # set prediction horizon
    ocp.solver_options.tf = Tf

    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)


    return acados_ocp_solver




ocp_solver = setup(x0, aMax, vMax, N_horizon, Tf)



def calculate_intermediate_points(point1, point2, distance):
    
    
    # Calculate the vector from point1 to point2
    vector = point2 - point1
    
    # Calculate the total distance between point1 and point2
    total_distance = np.linalg.norm(vector)
    
    if total_distance <= distance:
        return [point2]
    
    # Normalize the vector
    normalized_vector = vector / total_distance
    
    # Calculate the number of intermediate points
    num_points = int(total_distance / distance)
    
    # Generate intermediate points
    intermediate_points = [point1 + i * distance * normalized_vector for i in range(1, num_points)]
    intermediate_points.append(point2)
    return intermediate_points


def calculate_number_intermediate_points(point1, point2, n):
    
    

    # Create an array with the start and end points
    points = np.vstack((point1, point2))


    # Create a spline curve
    tck, u = scipy.interpolate.splprep(points.T, s=0, k=1)

    # Get the intermediate points
    new_points = scipy.interpolate.splev(np.linspace(0, 1, n), tck)

    
    p = np.asarray(new_points)
    return p.T

def interpolate_points(points, num_points):
    # Separate the points into individual components
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Create the interpolation functions for each axis
    fx = scipy.interpolate.interp1d(np.arange(points.shape[0]), x, kind='linear')
    fy = scipy.interpolate.interp1d(np.arange(points.shape[0]), y, kind='linear')
    fz = scipy.interpolate.interp1d(np.arange(points.shape[0]), z, kind='linear')

    # Generate the new interpolated points
    u_new = np.linspace(0, points.shape[0] - 1, num_points)
    x_new = fx(u_new)
    y_new = fy(u_new)
    z_new = fz(u_new)

    # Combine the interpolated points into a single array
    new_points = np.vstack((x_new, y_new, z_new)).T

    return new_points

def state_cb(msg):
    global current_state
    current_state = msg
    
def pose_cb(msg):
    global curr_state
    curr_state[0] = msg.pose.position.x
    curr_state[1] = msg.pose.position.y
    curr_state[2] = msg.pose.position.z

def vel_cb(msg):
    global curr_state
    curr_state[3] = msg.twist.linear.x
    curr_state[4] = msg.twist.linear.y
    curr_state[5] = msg.twist.linear.z

def accel_cb(msg):
    global curr_state
    #curr_state[6] = msg.accel.accel.linear.x
    #curr_state[7] = msg.accel.accel.linear.y
    #curr_state[8] = msg.accel.accel.linear.z - 9.81
    curr_state[6] = msg.linear_acceleration.x
    curr_state[7] = msg.linear_acceleration.y
    curr_state[8] = msg.linear_acceleration.z - 9.81
    
def pose_setpoint_cb(msg):
    global waypoints
    global curr_state
    global ocp_solver
    global N_horizon
    desired = np.zeros(3)
    desired[0] = msg.x
    desired[1] = msg.y
    desired[2] = msg.z
    
    set_mpc_target_pos(desired, 9, 3, N_horizon)

    
    
def set_mpc_target_pos(pos, nx, nu, N_horizon):
    
    
    global ocp_solver
    global curr_state
    
    
    yref = np.zeros((nx+nu, ))
    yref[0:3] = pos
    
    yref_e = np.zeros((nx, ))
    yref_e[0:3] = pos
    
    lbx = np.ones(3)*-2
    lbx_0 = np.zeros(9)
    lbx_0[0:3] = curr_state[0:3]
    lbx_0[3:6] = lbx
    lbx_0[6:9] = curr_state[6:9]
    
    
    ubx = np.ones(3) * 2
    ubx_0 = np.zeros(9)
    ubx_0[0:3] = curr_state[0:3]
    ubx_0[3:6] = ubx
    ubx_0[6:9] = curr_state[6:9]
    
    
    
    
    
    ocp_solver.cost_set(0, 'yref', yref)
    ocp_solver.constraints_set(0, 'lbx', lbx_0)
    ocp_solver.constraints_set(0, 'ubx', ubx_0)
    
    
    
    
    for i in range(1, N_horizon):
        ocp_solver.cost_set(i, 'yref', yref)
        ocp_solver.constraints_set(i, 'lbx', lbx)
        ocp_solver.constraints_set(i, 'ubx', ubx)
        
    
    ocp_solver.constraints_set(N_horizon, 'lbx', lbx)
    ocp_solver.constraints_set(N_horizon, 'ubx', ubx)
    ocp_solver.cost_set(N_horizon, 'y_ref', yref_e)
    
    


def set_mpc_target_pos2(points, nx, nu, N_horizon):
    
    
    global ocp_solver
    global curr_state
    if len(points) <= 1:
        yref = np.zeros((nx+nu, ))
        yref[0:3] = points[0]
        
        yref_e = np.zeros((nx, ))
        yref_e[0:3] = points[0]
        
        for i in range(N_horizon):
            ocp_solver.cost_set(i, 'yref', yref)
        ocp_solver.cost_set(N_horizon, 'y_ref', yref_e)
        
    else:
        trajectory = np.asarray( points)
        yrefs = interpolate_points(trajectory,  N_horizon)
        
        yref = np.zeros((nx+nu, ))
        
        yref_e = np.zeros((nx, ))
        yref_e[0:3] = points[0]
        
        for i in range(N_horizon):
            yref[0:3] = yrefs[i]
            ocp_solver.cost_set(i, 'yref', yref)
        ocp_solver.cost_set(N_horizon, 'y_ref', yref_e)

def check_distance(curr_point, next_point, dist_thresch):
    
    vector = next_point - curr_point
    total_distance = np.linalg.norm(vector)
    
    if total_distance <= dist_thresch:
        return True
    else:
        return False

if __name__ == "__main__":
    
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = pose_cb)
    vel_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, callback = vel_cb)
    #accel_sub = rospy.Subscriber("mavros/local_position/accel", AccelWithCovarianceStamped, callback = accel_cb)
    accel_sub = rospy.Subscriber("mavros/imu/data", Imu, callback = accel_cb)
    pose_setpoint_sub = rospy.Subscriber("input/poseSetpoint", Vector3, callback = pose_setpoint_cb)

    #local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_accel_pub = rospy.Publisher("mavros/setpoint_accel/accel", Vector3Stamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    
    


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(10)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    
    global waypoints
    
    waypoints = []
    checker = True
    accel = Vector3Stamped()
   
    
    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        local_accel_pub.publish(accel)
        #local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    
    
    pos = np.asarray(curr_state[0:3])
    pos[0] = pos[0]
    pos[1] = pos[1]
    pos[2] = pos[2]+5
          
          
    waypoints = [pos]       
            
    
    
    counter = 0
    
    
    while(not rospy.is_shutdown()):
    
        
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        counter += 1
        if counter == 20:
            print('Current position: ', np.round(curr_state[0:3], decimals=1))
            print('Current cost: ', ocp_solver.get_cost())
            
            counter = 0

        
        
        #if (len(waypoints)) == 0:
        #    if (check_distance(curr_state[0:3],final, 3 )):
        #       
        #        set_mpc_target_pos2(final, 9, 3, N_horizon, last=True)
        
        print('Current State: ', curr_state)
            
        #print('slack sl: ', ocp_solver.get(20, 'sl'))
        #print('slack su: ', ocp_solver.get(20, 'su'))
        #print(ocp_solver.get_from_qp_in(1, 'lbx'))
        #print(ocp_solver.get_from_qp_in(1, 'ubx'))
        U = ocp_solver.solve_for_x0(x0_bar = curr_state)
        
        #print('Acceleration Setpoint: ', U) 
        
        
        
        
        accel.vector.x = U[0]
        accel.vector.y = U[1]
        accel.vector.z = U[2]
        
        
        
        #local_pos_pub.publish(pose)
        local_accel_pub.publish(accel)
        
        
        rate.sleep()