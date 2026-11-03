"""Independent symbolic differentiation used only for equation validation."""
import numpy as np
import casadi as ca

def geometry_symbols(ell):
    x, o = ca.SX.sym("x", 5), ca.SX.sym("o", 2)
    radius = ca.SX.sym("radius")
    n = ca.vertcat(ca.cos(x[2]), ca.sin(x[2]))
    perpendicular = ca.vertcat(-n[1], n[0])
    p = x[:2]+ell*n
    velocity = x[3]*n+ell*x[4]*perpendicular
    f = ca.vertcat(x[3]*n, x[4], 0, 0)
    return x, o, radius, p, velocity, f

def clf_function(ell, config):
    x, _, _, p, velocity, f = geometry_symbols(ell)
    goal = ca.SX.sym("goal", 2)
    difference = goal-p
    d2 = ca.sumsqr(difference)
    near = d2 <= 1e-18
    chart=config.get("heading_chart","literal_arctan")
    if chart=="literal_arctan":
        # Paper Eq14 is the principal arctan of a ratio, with no angle wrap.
        # Its vertical-ray boundary is rejected by the action interface.
        bearing=ca.atan(ca.if_else(near,0,difference[1])/ca.if_else(near,1,difference[0]))
        heading_error=ca.if_else(near,0,x[2]-bearing)
    elif chart=="wrapped_atan2":
        bearing=ca.atan2(ca.if_else(near,0,difference[1]),ca.if_else(near,1,difference[0]))
        heading_error=ca.if_else(near,0,ca.atan2(ca.sin(x[2]-bearing),ca.cos(x[2]-bearing)))
    else:
        raise ValueError("Unknown VOCBF heading chart: "+str(chart))
    heading_rate = x[4]+ca.if_else(near,0,(difference[0]*velocity[1]-difference[1]*velocity[0])/ca.fmax(d2,1e-18))
    desired_speed = config["desired_speed_gain"]*ca.sqrt(ca.fmax(d2,1e-18))
    position_error = p-goal+ca.DM(config["position_feedback"])*velocity
    values = ca.vertcat(ca.dot(ca.DM(config["position_coefficients"]),position_error**2),
                        (heading_error+config["heading_feedback"]*heading_rate)**2,
                        (x[3]-desired_speed)**2,x[4]**2)
    jac = ca.jacobian(values,x)
    return ca.Function("vocbf_four_clfs", [x,goal], [jac[:,3:5],jac@f+config["clf_rate"]*values,values])

def moving_vocbf_function(ell,gain):
    x,o,radius,p,velocity,f=geometry_symbols(ell)
    vp=ca.SX.sym("obstacle_velocity",2)
    relative=p-o;relative_velocity=velocity-vp
    angle=ca.asin(radius/ca.sqrt(ca.sumsqr(relative)))-np.pi/2
    columns=[]
    for sign in (1,-1):
        a=sign*angle
        normal=ca.vertcat(ca.cos(a)*relative[0]-ca.sin(a)*relative[1],
                         ca.sin(a)*relative[0]+ca.cos(a)*relative[1])
        value=ca.dot(relative_velocity,normal)
        jx=ca.jacobian(value,x);jo=ca.jacobian(value,o)
        partial_t=jo@vp  # prescribed o_dot=vp, v_dot_peer=0
        columns.append(ca.vertcat(jx[3:5].T,jx@f+partial_t+gain*value,value,partial_t))
    return ca.Function("moving_vocbf_rows",[x,o,radius,vp],[ca.horzcat(*columns)])
