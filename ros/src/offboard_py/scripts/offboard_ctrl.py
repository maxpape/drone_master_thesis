#! /usr/bin/env python

from typing import Any
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




class MPC:
    def __init__(self, N_horizon, Tf, aMax, vMax):
        self.N_horizon = N_horizon
        self.Tf = Tf
        self.nx = 9
        self.nu = 3
        self.aMax = aMax
        self.vMax = vMax
        self.current_state = np.zeros(9)
        self.ocp_solver = None
        self.pos_setpoint = np.zeros(3)
                
    def set_pos(self, pos):
        self.current_state[0] = pos[0]   
        self.current_state[1] = pos[1]
        self.current_state[2] = pos[2]
        
    def set_vel(self, vel):
        self.current_state[3] = vel[0]   
        self.current_state[4] = vel[1]
        self.current_state[5] = vel[2]
        
    def set_accel(self, accel):
        self.current_state[6] = accel[0]   
        self.current_state[7] = accel[1]
        self.current_state[8] = accel[2]
        
    def get_state(self):
        return self.current_state
    
    def setup_mpc(self):
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
        
        Q_mat = np.zeros((9,9))
        Q_mat[0,0] = 2
        Q_mat[1,1] = 2
        Q_mat[2,2] = 4
        
        R_mat = np.eye(3)

        ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
        ocp.cost.W_e = Q_mat

        ocp.model.cost_y_expr = vertcat(model.x, model.u)
        ocp.model.cost_y_expr_e = model.x

        yref = np.zeros((ny, ))
        ocp.cost.yref  = yref

        yref_e = np.zeros((ny_e, ))
        ocp.cost.yref_e = yref_e
        
        aMax = self.aMax
        vMax = self.vMax
        # set constraints
        ocp.constraints.lbu = np.array([-aMax, -aMax, -aMax])
        ocp.constraints.ubu = np.array([+aMax, +aMax, +aMax])
    
        ocp.constraints.lbx = np.array([-vMax, -vMax, -vMax, -aMax, -aMax, -aMax])
        ocp.constraints.ubx = np.array([+vMax, +vMax, +vMax, +aMax, +aMax, +aMax])
            
        ocp.constraints.x0 = self.current_state
        ocp.constraints.idxbu = np.array([0, 1, 2])
        ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8])
    
        


        # slack for constraints
        #ocp.constraints.lsbx = np.array([-2,-2,-2])
        #ocp.constraints.usbx = np.array([+2,+2,+2])
        #ocp.constraints.idxsbx = np.array([0,1,2])
        ##
        #ns = 3
        #ocp.cost.zl = 10e-1 * np.ones((ns,)) # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        #ocp.cost.Zl = np.ones((ns,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        #ocp.cost.zu = 10e-1 * np.ones((ns,))    
        #ocp.cost.Zu = np.ones((ns,))  
    
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        
        ocp.solver_options.qp_solver_cond_N = N_horizon

        # set prediction horizon
        ocp.solver_options.tf = self.Tf

        solver_json = 'acados_ocp_' + model.name + '.json'
        self.ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)

   
   
   
# variables

current_state = State()
N_horizon = 50
Tf = 5
aMax = 5
vMax = 5     
mpc = MPC(N_horizon, Tf, aMax, vMax)
mpc.setup_mpc()










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
    global mpc
    pos = np.zeros(3)
    pos[0] = msg.pose.position.x
    pos[1] = msg.pose.position.y
    pos[2] = msg.pose.position.z
    mpc.set_pos(pos)
    

def vel_cb(msg):
    global mpc
    vel = np.zeros(3)
    vel[0] = msg.twist.linear.x
    vel[1] = msg.twist.linear.y
    vel[2] = msg.twist.linear.z
    mpc.set_vel(vel)

def accel_cb(msg):
    global mpc
    accel = np.zeros(3)
    accel[0] = msg.linear_acceleration.x
    accel[1] = msg.linear_acceleration.y
    accel[2] = msg.linear_acceleration.z - 9.81
    mpc.set_accel(accel)
    
def pose_setpoint_cb(msg):
    global mpc
    setpoint = np.zeros(3)
    setpoint[0] = msg.x
    setpoint[1] = msg.y
    setpoint[2] = msg.z
    mpc.pos_setpoint = setpoint
    
    set_mpc_target_pos()
    

    
    
def set_mpc_target_pos2():
    
    global mpc
        
    yref = np.zeros((mpc.nx+mpc.nu, ))
    yref[0:3] = mpc.pos_setpoint
    
    yref_e = np.zeros((mpc.nx, ))
    yref_e[0:3] = mpc.pos_setpoint
    
    lbx = np.ones(6)*-mpc.vMax
    
    ubx = np.ones(6)*mpc.vMax
    
    
    #
    #lbx_0 = np.zeros(9)
    #lbx_0[0:3] = mpc.current_state[0:3]
    #lbx_0[3:6] = -mpc.vMax*np.ones(3)
    #lbx_0[6:9] = -mpc.aMax*np.ones(3)
    ##
    ##
    #ubx_0 = np.zeros(9)
    #ubx_0[0:3] = mpc.current_state[0:3]
    #ubx_0[3:6] = mpc.vMax*np.ones(3)
    #ubx_0[6:9] = mpc.aMax*np.ones(3)
    #
    mpc.ocp_solver.cost_set(0, 'yref', yref)
    mpc.ocp_solver.constraints_set(0, 'lbx', mpc.current_state)
    mpc.ocp_solver.constraints_set(0, 'ubx', mpc.current_state)
    
    
    for i in range(1, mpc.N_horizon):
        mpc.ocp_solver.cost_set(i, 'yref', yref)
        mpc.ocp_solver.constraints_set(i, 'lbx', lbx)
        mpc.ocp_solver.constraints_set(i, 'ubx', ubx)
        
    
    #mpc.ocp_solver.constraints_set(N_horizon, 'ubx', ubx)
    #mpc.ocp_solver.constraints_set(N_horizon, 'lbx', lbx)
    mpc.ocp_solver.cost_set(mpc.N_horizon, 'y_ref', yref_e)
    
def set_mpc_target_pos():
    
    global mpc
        
    yref = np.zeros((mpc.nx+mpc.nu, ))
    yref[0:3] = mpc.pos_setpoint
    
    yref_e = np.zeros((mpc.nx, ))
    yref_e[0:3] = mpc.pos_setpoint
    
    
    #print(yref, yref_e)
    for j in range(mpc.N_horizon):
        
        mpc.ocp_solver.set(j, "yref", yref)
    mpc.ocp_solver.set(mpc.N_horizon, "yref", yref_e)
    
    
    
    #lbx = np.asarray([-5, -5, -5, -5, -5, -5])
    #ubx = np.asarray([+5, +5, +5, +5, +5, +5])
    
    
        
    
    #mpc.ocp_solver.set(0, "lbx", mpc.current_state)
    #mpc.ocp_solver.set(0, "ubx", mpc.current_state)   
    #for j in range(1, N_horizon):
       # mpc.ocp_solver.set(j, "lbx", lbx)
        #mpc.ocp_solver.set(j, "ubx", ubx)
    
    
    


def set_mpc_target_pos3(points, nx, nu, N_horizon):
    
    
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


    
        
        

def main():
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = pose_cb)
    vel_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, callback = vel_cb)
    #accel_sub = rospy.Subscriber("mavros/local_position/accel", AccelWithCovarianceStamped, callback = accel_cb)
    accel_sub = rospy.Subscriber("mavros/imu/data", Imu, callback = accel_cb)
    pose_setpoint_sub = rospy.Subscriber("input/poseSetpoint", Vector3, callback = pose_setpoint_cb)

    #local_accel_pub = rospy.Publisher("mavros/setpoint_accel/accel", Vector3Stamped, queue_size=10)
    local_accel_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    
    
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    
    
    ros_rate = 10
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(ros_rate)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    
    
    global mpc

    

    
    
    
    #waypoints = []
    checker = True
    accel = Vector3()
    velocity = Vector3()
    target = PositionTarget()
    
    #ignore_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
    ignore_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        
    target.type_mask = ignore_mask
    target.coordinate_frame = 1
    target.acceleration_or_force = accel
    
    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        local_accel_pub.publish(target)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    
    
    v_x_set = 0
    v_y_set = 0
    v_z_set = 0        
    
    
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

        

        
        
        st = mpc.current_state
        
        
        
        
        lbx = np.asarray([-5, -5, -5, -5, -5, -5])
        ubx = np.asarray([+5, +5, +5, +5, +5, +5])
        
        mpc.ocp_solver.set(0, "lbx", st)
        mpc.ocp_solver.set(0, "ubx", st)
        for j in range(1, mpc.N_horizon):
            mpc.ocp_solver.set(j, "lbx", lbx)
            mpc.ocp_solver.set(j, "ubx", ubx)
        
        
        
        
        status = mpc.ocp_solver.solve()
        #U = mpc.ocp_solver.solve_for_x0(x0_bar = st)
        
        states = np.zeros((mpc.N_horizon, 9))
        
        for j in range(mpc.N_horizon):
            states[j] = mpc.ocp_solver.get(j, 'x')
        
            
        #print('state at 5:', states[5][3:7])
        #print('lam: ', mpc.ocp_solver.get_from_qp_in(5, 'lbx'))
            
        
        
        U = mpc.ocp_solver.get(0, 'u')
        #counter += 1
        #if counter == 20:
        #    print('Current pos_setpoint: ', np.round(mpc.pos_setpoint, decimals=2))
        #    print('Current state: ',        np.round(mpc.current_state, decimals=2))
        #    print('Current cost: ',         np.round(mpc.ocp_solver.get_cost(), decimals=2))
        #    print('Acceleration Setpoint: ', np.round(U, decimals=2))
        #    #mpc.ocp_solver.store_iterate(filename= 'result', overwrite=True)
        #    counter = 0
        
        
        print('Acceleration Setpoint: ', np.round(U, decimals=2))
        accel.x = U[0]
        accel.y = U[1]
        accel.z = U[2]
        
        #velocity.x = 0
        #velocity.y = 0
        #velocity.z = mpc.current_state[5] + U[2]/ros_rate
        
        
        target.coordinate_frame = 1
        ignore_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        target.type_mask = ignore_mask
        target.acceleration_or_force = accel
        #target.velocity = velocity
        
        
        
        
        local_accel_pub.publish(target)
        
        
        rate.sleep()  
    
        
        
if __name__ == "__main__":
    
    main()
    