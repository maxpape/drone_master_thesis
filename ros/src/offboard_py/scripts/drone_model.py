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

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_drone_ode_model() -> AcadosModel:

    model_name = 'drone_ode'

    

    # set up states & controls
    
    p_x = SX.sym('p_x')
    p_y = SX.sym('p_y')
    p_z = SX.sym('p_z')
    v_x = SX.sym('v_x')
    v_y = SX.sym('v_y')
    v_z = SX.sym('v_z')
    a_x = SX.sym('a_x')
    a_y = SX.sym('a_y')
    a_z = SX.sym('a_z')

    x = vertcat(p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z)

    a_x_set = SX.sym('a_x_set')
    a_y_set = SX.sym('a_y_set')
    a_z_set = SX.sym('a_z_set')
    u = vertcat(a_x_set, a_y_set, a_z_set)

    # xdot
    
    x1_dot = SX.sym('x1_dot')
    x2_dot = SX.sym('x2_dot')
    x3_dot = SX.sym('x3_dot')
    x4_dot = SX.sym('x4_dot')
    x5_dot = SX.sym('x5_dot')
    x6_dot = SX.sym('x6_dot')
    x7_dot = SX.sym('x7_dot')
    x8_dot = SX.sym('x8_dot')
    x9_dot = SX.sym('x9_dot')

    xdot = vertcat(x1_dot, x2_dot, x3_dot, x4_dot, x5_dot, x6_dot, x7_dot, x8_dot, x9_dot)

    # dynamics
    
    f_expl = vertcat(v_x,
                     v_y,
                     v_z,
                     a_x,
                     a_y,
                     a_z,
                     (a_x_set - a_x - 0.404 * v_x)*3,
                     (a_y_set - a_y - 0.478 * v_y)*3,
                     (a_z_set - a_z - 0.6 * v_z)*3
                     )
    
    #f_expl = vertcat(v_x,
    #                 v_y,
    #                 v_z,
    #                 a_x,
    #                 a_y,
    #                 a_z,
    #                 (a_x_set - a_x*0.2)*0.1,
    #                 (a_y_set - a_y*0.2)*0.1,
    #                 (a_z_set - a_z*0.2)*0.1
    #                 )
    #
    #f_expl = vertcat(v_x,
    #                 v_y,
    #                 v_z,
    #                 a_x,
    #                 a_y,
    #                 a_z,
    #                 (a_x_set - a_x)*3,
    #                 (a_y_set - a_y)*3,
    #                 (a_z_set - a_z)*3
    #                 )
    

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model
