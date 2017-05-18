import numpy as np
from copy import copy

def flight_sim( x, u, w ):
    """ flight_sim returns change in state x if input u is applied, with wind w
       x : [n, e, h, chi, gamma, Va, phi]
           n: north position (m)
           e: east position (m)
           h: altitude (m)
           chi: course angle (rad) (inertial)
           gamma: pitch angle (rad) (inertial)
           Va: airspeed (m)
           phi: roll angle (rad)

       u : [Va_c phi_c gamma_c]
           commanded values for these parameters

       w : [wn, we, wd] (north and east components of the prevailing wind)

       returns: dx/dt"""
    n, e, h, chi, gamma, Va, phi = x
    Va_c, phi_c, gamma_c = u
    wn, we, wd = w

    Vw = np.sqrt(np.dot(w, w))
    b = 2*np.dot( [np.cos(chi)*np.cos(gamma), np.sin(chi)*np.cos(gamma), -np.sin(gamma)], w  )
    Vg = ( -b + np.sqrt(b**2 - 4*(Vw**2 - Va**2)) ) / 2

    gamma_a = np.arcsin( (Vg * np.sin(gamma) + wd) / Va )
    psi = chi - np.arcsin( 1/(Va * np.cos(gamma_a)) * np.dot( [ wn, we ], [ -np.sin(chi), np.cos(chi) ] ) )

    # Velocity to neh coordinates
    dn = Va*np.cos(psi)*np.cos(gamma_a) + wn
    de = Va*np.sin(psi)*np.cos(gamma_a) + we
    dh = Va*np.sin(gamma_a) - wd

    # Equations of motion for coordinated turns
    dchi = 9.81/Vg*np.tan(phi)*np.cos(chi - psi)

    # first order models of controlled systems, parameterized by constants below
    b_gamma = 1.
    b_Va = 1.
    b_phi = 1.
    dgamma = b_gamma*(gamma_c - gamma)
    dVa = b_Va*(Va_c - Va)
    dphi = b_phi*(phi_c - phi)

    return np.array([dn, de, dh, dchi, dgamma, dVa, dphi])

def wrapAngles(x):
    angle_is = [3,4,6]
    for i in angle_is:
        while x[i] < -np.pi:
            x[i] += 2*np.pi
        while x[i] > np.pi:
            x[i] -= 2*np.pi
    return x

def rollout(policy, x0, T, dt = 0.1):
    stats = {"t": [], "x": [], "u": []}
    x = x0
    t = 0
    while t < T:
        u = policy( x )
        w = np.zeros(3)

        # log stats
        stats["t"].append(copy(t))
        stats["x"].append(copy(x))
        stats["u"].append(copy(u))

        dx = flight_sim( x, u, w )
        x += dx*dt
        x = wrapAngles(x)

        t += dt

    return stats


def basic_policy(x):
    Va_c = 15
    h_c = 70
    chi_c = np.pi/4
    kp_alt = 0.02
    kp_course = 0.8
    gamma_c = kp_alt*(h_c - x[2])
    phi_c = kp_course*(chi_c - x[3])

    return np.array([Va_c, phi_c, gamma_c])


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from path_controller import *
    x0 = [0,0,40,0,0,15,0]
    mission = np.array([[200,100], [0,200], [100,15], [100,300]])
    mission2 = np.array([[100,300], [200,100], [100,15], [0,200] ])
    controller = PathController(mission)

    stats = rollout(controller.policy, x0, 100, 0.1)

    controller = PathController(mission2)
    stats2 = rollout(controller.policy, x0, 100, 0.1)

    t = stats["t"]
    x = np.vstack(stats["x"])
    x2 = np.vstack(stats2["x"])
    u = np.vstack(stats["u"])
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.plot(t, x[:,3])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Course Angle (s)")
    ax = fig.add_subplot(312)
    ax.plot(t, x[:,6])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll Angle (s)")
    ax = fig.add_subplot(313)
    ax.plot(t, x[:,2])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (s)")
    fig2 = plt.figure()
    ax = fig2.add_subplot(111)
    ax.plot(x[:,1], x[:,0], color='b')
    ax.plot(x2[:,1], x2[:,0], color='g')
    ax.plot(mission[:,1], mission[:,0], '+', color='r')
    ax.set_xlabel("East Position (m)")
    ax.set_ylabel("North Position (m)")
    for wp in mission:
        circle = plt.Circle([wp[1], wp[0]], 10, color='r', fill=False)
        ax.add_artist(circle)
    plt.show()
