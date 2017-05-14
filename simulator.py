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

        t += dt

    return stats


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    x0 = [0,0,40,0,0,15,0]

    def policy(x):
        Va_c = 15
        h_c = 70
        kp_alt = 0.02
        gamma_c = kp_alt*(h_c - x[2])
        phi_c = 0

        return np.array([Va_c, phi_c, gamma_c])

    stats = rollout(policy, x0, 30, 0.1)

    t = stats["t"]
    x = np.vstack(stats["x"])
    u = np.vstack(stats["u"])
    import pdb; pdb.set_trace()
    plt.plot(t, x[:,2])
    plt.show()
