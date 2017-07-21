#!/usr/bin/env python
"""@@This node uses simulated data of the actuators to compute the AUV dynamic
behavior. This node can be used to simulate real AUV behavior and its interaction
with the environtment. User can add currents
detection has been implemented.@@"""

# Basic ROS imports
import roslib
roslib.load_manifest('simulation')
import rospy
import PyKDL

# Import msgs
from nav_msgs.msg import Odometry
from control.msg import Setpoints

# More imports
import math
import numpy as np
import tf

# getRosParams:
#   obj: object for which to set its attributes
#   params: dictionary of {'key': 'value'} pairs defining the attribute
#      name that'll be set and the name of the ROS param that'll be
#      assigned to it
#   [node_name]: optional ROS node name of the node for which the params
#      are retrieved
def getRosParams(obj, params, node_name=None):
    valid_config = True
    for key in params:
        if rospy.has_param(params[key]):
            param_value = rospy.get_param(params[key])
            setattr(obj, key, param_value)
        else:
            valid_config = False
            if node_name == None:
                rospy.logfatal(params[key] + " parameter not found")
            else:
                rospy.logfatal("[" + node_name + "]: " + params[key] + " parameter not found")
    return valid_config

def wrapAngle(angle):  # Should be the default option, "wrap angle" is the proper term in English
    return (angle + ( 2.0 * pi * floor( ( pi - angle ) / ( 2.0 * pi ) ) ) )

class Dynamics :
    """ Simulates the dynamics of an AUV from thrusters rpm"""

    def __init__(self, name):
        """ Simulates the dynamics of an AUV """
        self.name = name

        # Load dynamic parameters
        self.get_config()

        # Initialize vars and matrices. They are not init. in the constructor,
        # but readability is improved
        self.initialize()

        # Create publisher
        self.pub_odom = rospy.Publisher(self.odom_topic_name,
                                        Odometry,
                                        queue_size = 2)

        # Create subscribers
        rospy.Subscriber(self.thrusters_topic,
                         Setpoints,
                         self.update_thrusters,
                         queue_size = 1)

        # Show message
        rospy.loginfo("[%s]: initialized", self.name)


    def initialize(self):
        """ Initialize vars and matrices """
        # Init pose, velocity and rate
        self.v = self.v_0
        self.p = self.p_0
        self.p_dot = np.zeros(len(self.p))
        self.v_dot = np.zeros(len(self.v))
        self.rate = 1.0 / self.period

        # Inertia Tensor. Principal moments of inertia,
        # and products of inertia [kg*m*m]
        Ixx = self.tensor[0]
        Ixy = self.tensor[1]
        Ixz = self.tensor[2]
        Iyx = self.tensor[3]
        Iyy = self.tensor[4]
        Iyz = self.tensor[5]
        Izx = self.tensor[6]
        Izy = self.tensor[7]
        Izz = self.tensor[8]
        m = self.mass
        xg = self.gravity_center[0]
        yg = self.gravity_center[1]
        zg = self.gravity_center[2]

        Mrb=[m,     0,      0,      0,      m*zg,       -m*yg,
             0,     m,      0,      -m*zg,  0,          m*xg,
             0,     0,      m,      m*yg,   -m*xg,      0,
             0,     -m*zg,  m*yg,   Ixx,    Ixy,        Ixz,
             m*zg,  0,      -m*xg,  Iyx,    Iyy,        Iyz,
             -m*yg, m*xg,   0,      Izx,    Izy,        Izz]
        Mrb = np.array(Mrb).reshape(6, 6)

        # Inertia matrix of the rigid body
        # Added Mass derivative
        Ma=[m/2,    0,      0,      0,      0,      0,
            0,      m/2,    0,      0,      0,      0,
            0,      0,      m/2,    0,      0,      0,
            0,      0,      0,      0,      0,      0,
            0,      0,      0,      0,      0,      0,
            0,      0,      0,      0,      0,      0]
        Ma = np.array(Ma).reshape(6, 6)

        # Mass matrix: Mrb + Ma
        self.M = Mrb + Ma
        self.IM = np.matrix(self.M).I

        # Init currents
        np.random.seed()
        self.e_vc = np.random.normal(self.current_mean, self.current_sigma)

        # Initial thrusters setpoint
        self.u = np.zeros(self.thrusters)
        self.old_u = self.u # Previous setpoints


    def update_thrusters(self, thrusters) :
        """ Thruster callback, input in rpm """
        self.old_u = self.u
        self.u = np.array( thrusters.setpoints ).clip(min=-1, max=1)

    def compute_currents(self):
        """ Water currents, returns a velocity """
        # Compute random currents
        e_vc = np.random.normal(self.current_mean,
                                self.current_sigma)
        for i in range(3):
            if e_vc[i] > self.current_max[i]:
                e_vc[i] = self.current_max[i]
            if e_vc[i] < self.current_min[i]:
                e_vc[i] = self.current_min[i]

        t = PyKDL.Vector(e_vc[0], e_vc[1], e_vc[2])
        O = PyKDL.Rotation.RPY(self.p[3], self.p[4], self.p[5])
        currents = O.Inverse() * t
        return np.array([currents[0], currents[1], currents[2], 0, 0, 0])


    def damping_matrix(self, vel):
        """ Damping matrix """
        # Linear hydrodynamic damping coeficients
        Xu = self.damping[0]
        Yv = self.damping[1]
        Zw = self.damping[2]
        Kp = self.damping[3]
        Mq = self.damping[4]
        Nr = self.damping[5]

        # Quadratic hydrodynamic damping coeficients
        Xuu = self.quadratic_damping[0]    #[Kg/m]
        Yvv = self.quadratic_damping[1]    #[Kg/m]
        Zww = self.quadratic_damping[2]    #[Kg/m]
        Kpp = self.quadratic_damping[3]    #[Kg*m*m]
        Mqq = self.quadratic_damping[4]    #[Kg*m*m]
        Nrr = self.quadratic_damping[5]    #[Kg*m*m]

        d = np.diag([Xu + Xuu*abs(vel[0]),
                     Yv + Yvv*abs(vel[1]),
                     Zw + Zww*abs(vel[2]),
                     Kp + Kpp*abs(vel[3]),
                     Mq + Mqq*abs(vel[4]),
                     Nr + Nrr*abs(vel[5])])
        return d


    def generalized_force(self, du):
        """ Compute the force of each thruster from rpm """
        # Build the signed (lineal/quadratic) thruster coeficient array
        # Signed square of each thruster setpoint
        du = du * abs(du)

        ct = np.zeros(len(du))
        i1 = np.nonzero(du >= 0.0)
        i2 = np.nonzero(du <= 0.0)
        ct[i1] = self.ctf
        ct[i2] = self.ctb
        b =  np.dot(self.thrusters_matrix, (np.identity(len(du)) * ct))

        # The value of t is the generalized force
        t = np.dot(b, du)

        # Transforms a matrix into an array
        t = np.squeeze(np.asarray(t))
        return t


    def coriolis_matrix(self, vel):
        """ Coriolis matrix """
        s1 = __s__( np.dot(self.M[0:3,0:3], vel[0:3]) +
                    np.dot(self.M[0:3,3:6], vel[3:6]) )
        s2 = __s__( np.dot(self.M[3:6,0:3], vel[0:3]) +
                    np.dot(self.M[3:6,3:6], vel[3:6]) )
        c = np.zeros((6, 6))
        c[0:3,3:6] = -s1
        c[3:6,0:3] = -s1
        c[3:6,3:6] = -s2
        return c


    def gravity(self, pos):
        """ Gravity and weight matrix """
        # Weight and Flotability
        W = self.mass * self.g # [N]

        # If the vehicle moves out of the water the flotability decreases
        if pos[2] < self.surface_radius:
            r = self.radius + ( pos[2] - self.surface_radius )
            if r < 0.0:
                r = 0.0
        else :
            r = self.radius
        F = ((4 * math.pi * pow(r,3))/3)*self.density*self.g

        # Gravity center position in the robot fixed frame (x',y',z') [m]
        zg = self.gravity_center[2]

        g = np.array([(W - F) * np.sin(pos[4]),
                   -(W - F) * np.cos(pos[4]) * np.sin(pos[3]),
                   -(W - F) * np.cos(pos[4]) * np.cos(pos[3]),
                   zg*W*np.cos(pos[4])*np.sin(pos[3]),
                   zg*W*np.sin(pos[4]),
                   0.0])
        return g


    def inverse_dynamic(self, pos, vel, u, f, current) :
        """ Given the setpoint for each thruster, the previous velocity
            and the previous position computes the v_dot """

        d = self.damping_matrix(vel+current)
        t = self.generalized_force(u)
        c = self.coriolis_matrix(vel)
        g = self.gravity(pos)
        c_v = np.dot((c-d), vel+current)
        v_dot = np.dot(self.IM, (t-f-c_v-g))

        # Transforms a matrix into an array
        v_dot = np.squeeze(np.asarray(v_dot))

        return v_dot


    def kinematics(self, pos, vel) :
        """ Given the current velocity and the previous
            position computes the p_dot """
        roll = pos[3]
        pitch = pos[4]
        yaw = pos[5]
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        rec = [cy*cp, -sy*cr+cy*sp*sr, sy*sr+cy*cr*sp,
               sy*cp, cy*cr+sr*sp*sy, -cy*sr+sp*sy*cr,
               -sp, cp*sr, cp*cr]
        rec = np.array(rec).reshape(3,3)

        to = [1.0, sr*np.tan(pitch), cr*np.tan(pitch),
              0.0, cr, -sr,
              0.0, sr/cp, cr/cp]
        to = np.array(to).reshape(3,3)

        p_dot = np.zeros(6)
        p_dot[0:3] = np.dot(rec, vel[0:3])
        p_dot[3:6] = np.dot(to, vel[3:6])
        return p_dot


    def step(self, pos, vel, u, f, current):
        """ Compute kinematics and inverse dynamics """
        return self.kinematics(pos, vel), self.inverse_dynamic(pos, vel, u, f, current)


    def iterate(self):
        """ Main loop operations """
        # Compute current
        current = self.compute_currents()

        # Runge-Kutta, 4th order
        k1_pos, k1_vel = self.step(self.p, self.v, self.old_u, self.old_f, current)
        k2_pos, k2_vel = self.step(self.p + self.period * 0.5 * k1_pos, self.v + self.period * 0.5 * k1_vel, 0.5 * (self.old_u + self.u), 0.5 * (self.old_f + self.f), current)
        k3_pos, k3_vel = self.step(self.p + self.period * 0.5 * k2_pos, self.v + self.period * 0.5 * k2_vel, 0.5 * (self.old_u + self.u), 0.5 * (self.old_f + self.f), current)
        k4_pos, k4_vel = self.step(self.p + self.period * k3_pos, self.v + self.period * k3_vel, self.u, self.f, current)

        self.p = self.p + self.period / 6.0 * ( k1_pos +
                                                2.0 * k2_pos +
                                                2.0 * k3_pos +
                                                k4_pos )
        self.v = self.v + self.period / 6.0 * ( k1_vel +
                                                2.0 * k2_vel +
                                                2.0 * k3_vel +
                                                k4_vel )

        self.p[3] = wrapAngle(self.p[3])
        self.p[4] = wrapAngle(self.p[4])
        self.p[5] = wrapAngle(self.p[5])

        # Publish odometry
        self.pub_odometry()


    def pub_odometry(self):
        """ Publish odometry message """
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.world_frame_id
        odom.child_frame_id = self.frame_id

        odom.pose.pose.position.x = self.p[0]
        odom.pose.pose.position.y = self.p[1]
        odom.pose.pose.position.z = self.p[2]

        orientation = tf.transformations.quaternion_from_euler(self.p[3],
                                                               self.p[4],
                                                               self.p[5],
                                                               'sxyz')
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]

        odom.twist.twist.linear.x = self.v[0]
        odom.twist.twist.linear.y = self.v[1]
        odom.twist.twist.linear.z = self.v[2]
        odom.twist.twist.angular.x = self.v[3]
        odom.twist.twist.angular.y = self.v[4]
        odom.twist.twist.angular.z = self.v[5]

        self.pub_odom.publish(odom)

    def get_config(self):
        """ Get config from config file """
        if rospy.has_param("vehicle_name") :
            self.vehicle_name = rospy.get_param('vehicle_name')
        else:
            rospy.logfatal("[%s]: vehicle_name parameter not found", self.name)
            exit(0)  # TODO: find a better way

        param_dict = {'thrusters': "dynamics/" + self.vehicle_name + "/number_of_thrusters",
                      'thrusters_topic': "dynamics/" + self.vehicle_name + "/thrusters_topic",
                      'thrusters_matrix': "dynamics/" + self.vehicle_name + "/thrusters_matrix",
                      'period': "dynamics/" + self.vehicle_name + "/period",
                      'mass': "dynamics/" + self.vehicle_name + "/mass",
                      'gravity_center': "dynamics/" + self.vehicle_name + "/gravity_center",
                      'g': "dynamics/" + self.vehicle_name + "/g",
                      'radius': "dynamics/" + self.vehicle_name + "/radius",
                      'surface_radius': "dynamics/" + self.vehicle_name + "/surface_radius",
                      'ctf': "dynamics/" + self.vehicle_name + "/ctf",
                      'ctb': "dynamics/" + self.vehicle_name + "/ctb",
                      'density': "dynamics/" + self.vehicle_name + "/density",
                      'tensor': "dynamics/" + self.vehicle_name + "/tensor",
                      'damping': "dynamics/" + self.vehicle_name + "/damping",
                      'quadratic_damping': "dynamics/" + self.vehicle_name + "/quadratic_damping",
                      'p_0': "dynamics/" + self.vehicle_name + "/initial_pose",
                      'v_0': "dynamics/" + self.vehicle_name + "/initial_velocity",
                      'odom_topic_name': "dynamics/" + self.vehicle_name + "/odom_topic_name",
                      'frame_id': "frames/base_link",
                      'world_frame_id': "frames/map",
                      'current_mean': "dynamics/current_mean",
                      'current_sigma': "dynamics/current_sigma",
                      'current_min': "dynamics/current_min",
                      'current_max': "dynamics/current_max"}

        if not getRosParams(self, param_dict, self.name):
            rospy.logfatal("[%s]: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way
        self.thrusters_matrix = np.array(self.thrusters_matrix).reshape(6, self.thrusters)
        self.current_mean = np.array(self.current_mean) * self.period
        self.current_sigma = np.array(self.current_sigma) * self.period


def __s__(x):
    """ Given a 3D vector computes a 3x3 matrix for .... ? """
    ret = np.array([0.0, -x[2], x[1], x[2], 0.0, -x[0], -x[1], x[0], 0.0 ])
    return ret.reshape(3,3)


if __name__ == '__main__':
    try:
        rospy.init_node('dynamics')
        dynamics = Dynamics(rospy.get_name())
        rate_it = rospy.Rate(dynamics.rate)
        while not rospy.is_shutdown():
            if (dynamics.pub_odom.get_num_connections() > 0):
                dynamics.iterate()
                rate_it.sleep()

    except rospy.ROSInterruptException:
        pass
