# -*- coding: future_fstrings -*-
#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
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
# POSSIBILITY OF SUCH DAMAGE.;
#

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from drone_model import export_drone_ode_model
from utils import plot_pendulum, plot_drone
import numpy as np
import scipy.linalg
from casadi import vertcat

def setup(x0, aMax, vMax, N_horizon, Tf, RTI=False):
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

    
    Q_mat = np.zeros((9,9))
    Q_mat[0,0] = 2
    Q_mat[1,1] = 2
    Q_mat[2,2] = 2
    
    R_mat = np.eye(3)

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x
    desired = np.asarray([0, 0, 0])
    
    yref = np.zeros((ny, ))
    yref[0:3] = desired
    ocp.cost.yref  = yref
    
    yref_e = np.zeros((ny_e, ))
    yref_e[0:3] = desired
    ocp.cost.yref_e = yref_e

    

    # set constraints
    ocp.constraints.lbu = np.array([-aMax, -aMax, -aMax])
    ocp.constraints.ubu = np.array([+aMax, +aMax, +aMax])
        
    ocp.constraints.lbx = np.array([-vMax, -vMax, -vMax, -aMax, -aMax, -aMax])
    ocp.constraints.ubx = np.array([+vMax, +vMax, +vMax, +vMax, +aMax, +aMax])

    ocp.constraints.x0 = x0
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8])

    #ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    #ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    #ocp.solver_options.integrator_type = 'IRK'
    #ocp.solver_options.sim_method_newton_iter = 10

    
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    
    

    ocp.solver_options.qp_solver_cond_N = N_horizon

    # set prediction horizon
    ocp.solver_options.tf = Tf

    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)

    # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = solver_json)

    return acados_ocp_solver, acados_integrator


def main(use_RTI=False):

    x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
    aMax = 2
    vMax = 2

    Tf = 5
    N_horizon = 100

    ocp_solver, integrator = setup(x0, aMax, vMax, N_horizon, Tf, use_RTI)

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    Nsim = 100
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim+1, nu))

    simX[0,:] = x0

    
    t = np.zeros((Nsim))
    
    t_preparation = np.zeros((Nsim))
    t_feedback = np.zeros((Nsim))

    

    # closed loop
    for i in range(Nsim):

        
         
         
        
         
         
        if i == 1 :
            
            
            
            for j in range(N_horizon):
                yref = np.array([50, 50, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0])
                ocp_solver.set(j, "yref", yref)
                
            
            
            yref_N = np.array([50, 50, 50, 0, 0, 0, 0, 0, 0])
            ocp_solver.set(N_horizon, "yref", yref_N)  
         
           
        if i == 50 :
            
            lbx = np.asarray([-6, -6, -6, -10, -10, -10])
            ubx = np.asarray([6, 6, 6, 10, 10, 10])
            
            for j in range(N_horizon):
                yref = np.array([108, 108, 108, 0, 0, 0, 0, 0, 0, 0, 0, 0])
                ocp_solver.set(j, "yref", yref)
                
            
            ocp_solver.set(0, "lbx", simX[i, :])
            ocp_solver.set(0, "ubx", simX[i, :])   
            for j in range(1, N_horizon):
                ocp_solver.set(j, "lbx", lbx[0:6])
                ocp_solver.set(j, "ubx", ubx[0:6])
            yref_N = np.array([108, 108, 108, 0, 0, 0, 0, 0, 0])
            ocp_solver.set(N_horizon, "yref", yref_N)
        
        #ocp_solver.set(0, "lbx", simX[i, :])
        #ocp_solver.set(0, "ubx", simX[i, :])
        # solve ocp and get next control input
        ocp_solver.set(0, "lbx", simX[i, :])
        ocp_solver.set(0, "ubx", simX[i, :]) 
        status = ocp_solver.solve()
        #simU[i,:] = ocp_solver.solve_for_x0(x0_bar = simX[i, :])
        simU[i,:] = ocp_solver.get(0, "u")
        t[i] = ocp_solver.get_stats('time_tot')

        # simulate system
        simX[i+1, :] = integrator.simulate(x=simX[i, :], u=simU[i,:])

    

    # plot results
    

    plot_drone(Nsim, Tf, simX, simU)
    ocp_solver = None
    integrator = None


if __name__ == '__main__':
    #main(use_RTI=False)
    main(use_RTI=True)
