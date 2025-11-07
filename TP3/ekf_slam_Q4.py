"""
Extended Kalman Filter SLAM example

author: Atsushi Sakai (@Atsushi_twi)

Modified : Goran Frehse, David Filliat
"""

import math

import matplotlib.pyplot as plt
import numpy as np

DT = 0.1  # time tick [s]
SIM_TIME = 100.0  # simulation time [s]
MAX_RANGE = 10.0  # maximum observation range
M_DIST_TH = 9.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]
KNOWN_DATA_ASSOCIATION = 1  # Whether we use the true landmarks id or not


# UDI parameters
UDI_R_LIST = [2.0, 4.0, 6.0, 8.0]        # guessed ranges along the ray (<= MAX_RANGE)
UDI_P_RADIAL = 9.0                        # big variance along the ray (m^2)
UDI_P_TANG   = 0.5                        # smaller across-ray variance
UDI_PRUNE_THRESH = 1e-4                   # prune hypothesis if its weight gets too small

# book-keeping for hypothesis weights (indexed like landmarks in state)
lm_weights = []   # one weight per landmark (same order as in xEst)
lm_groups  = []   # group id per landmark (hypotheses of same real feature share a group)
next_group_id = 0


# Simulation parameter
# noise on control input
Q_sim = (3 * np.diag([0.1, np.deg2rad(1)])) ** 2
# noise on measurement
Py_sim = (1 * np.diag([0.1, np.deg2rad(5)])) ** 2

# Kalman filter Parameters
# Estimated input noise for Kalman Filter
Q = 2 * Q_sim
# Estimated measurement noise for Kalman Filter
Py = np.array([[2 * Py_sim[1, 1]]])   # 1x1 (bearing variance only)

# Initial estimate of pose covariance
initPEst = 0.01 * np.eye(STATE_SIZE)
initPEst[2,2] = 0.0001  # low orientation error

# True Landmark id for known data association
trueLandmarkId =[]

# Init displays
show_animation = True
f, (ax1, ax2) = plt.subplots(1, 2, sharey=True, figsize=(14, 7))
ax3 = plt.subplot(3, 2, 2)
ax4 = plt.subplot(3, 2, 4)
ax5 = plt.subplot(3, 2, 6)


# --- Helper functions

def add_landmark_hypothesis(xEst, PEst, r, angle_meas, group_id):
    # hypothesized absolute position
    lx = xEst[0, 0] + r * math.cos(xEst[2, 0] + angle_meas)
    ly = xEst[1, 0] + r * math.sin(xEst[2, 0] + angle_meas)
    lm = np.array([[lx], [ly]])

    # append to state
    xEst = np.vstack((xEst, lm))

    # build oriented covariance for this landmark block
    c, s = math.cos(xEst[2, 0] + angle_meas), math.sin(xEst[2, 0] + angle_meas)
    Rdir = np.array([[c, -s], [s, c]])  # rotation from local (radial,tangential) to world
    Plm_local = np.diag([UDI_P_RADIAL, UDI_P_TANG])
    Plm_world = Rdir @ Plm_local @ Rdir.T

    # cross-covariances using Jacobians of augmentation (approx with Jr for pose only)
    Jr = np.array([[1.0, 0.0, -r * s],
                   [0.0, 1.0,  r * c]])
    # extend P (pose cross terms)
    bottomPart = np.hstack((Jr @ PEst[0:3, 0:3], Jr @ PEst[0:3, 3:]))
    rightPart  = bottomPart.T
    PEst = np.vstack((np.hstack((PEst, rightPart)),
                      np.hstack((bottomPart, Jr @ PEst[0:3, 0:3] @ Jr.T + Plm_world))))
    # bookkeeping
    lm_weights.append(1.0 / len(UDI_R_LIST))  # start uniform
    lm_groups.append(group_id)
    return xEst, PEst

def prune_landmark(xEst, PEst, i):
    global lm_weights, lm_groups
    id0 = STATE_SIZE + 2*i
    keep_idx = list(range(id0)) + list(range(id0+2, len(xEst)))
    xEst = xEst[keep_idx, :]

    keep_rows = keep_idx
    keep_cols = keep_idx
    PEst = PEst[np.ix_(keep_rows, keep_cols)]

    del lm_weights[i]
    del lm_groups[i]
    return xEst, PEst


def calc_n_lm(x):
    """
    Computes the number of landmarks in state vector
    """

    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def calc_landmark_position(x, y):
    """
    Computes absolute landmark position from robot pose and observation
    """

    y_abs = np.zeros((2, 1))

    y_abs[0, 0] = x[0, 0] + y[0] * math.cos(x[2, 0] + y[1])
    y_abs[1, 0] = x[1, 0] + y[0] * math.sin(x[2, 0] + y[1])

    return y_abs


def get_landmark_position_from_state(x, ind):
    """
    Extract landmark position from state vector
    """

    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def pi_2_pi(angle):
    """
    Put an angle between -pi / pi
    """

    return (angle + math.pi) % (2 * math.pi) - math.pi


def plot_covariance_ellipse(xEst, PEst, axes, lineType):
    """
    Plot one covariance ellipse from covariance matrix
    """

    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    if eigval[smallind] < 0:
        print('Pb with Pxy :\n',Pxy)
        exit()

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [3*a * math.cos(it) for it in t]
    y = [3*b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    rot = np.array([[math.cos(angle), math.sin(angle)],
                    [-math.sin(angle), math.cos(angle)]])
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    axes.plot(px, py, lineType)


# --- Motion model related functions

def calc_input():
    """
    Generate a control vector to make the robot follow a circular trajectory
    """

    v = 1  # [m/s]
    #yaw_rate = 0.1  # [rad/s]
    #yaw_rate = 0.2 # small radius loop: faster rotation
    yaw_rate = 0.08 # big radius loop: slower rotation
    u = np.array([[v, yaw_rate]]).T
    return u


def motion_model(x, u):
    """
    Compute future robot position from current position and control
    """
    
    xp = np.array([[x[0,0] + u[0,0]*DT * math.cos(x[2,0])],
                  [x[1,0] + u[0,0]*DT * math.sin(x[2,0])],
                  [x[2,0] + u[1,0]*DT]])
    xp[2] = pi_2_pi(xp[2])

    return xp.reshape((3, 1))


def jacob_motion(x, u):
    """
    Compute the jacobians of motion model wrt x and u
    """

    # Jacobian of f(X,u) wrt X
    A = np.array([[1.0, 0.0, float(-DT * u[0,0] * math.sin(x[2, 0]))],
                  [0.0, 1.0, float(DT * u[0,0] * math.cos(x[2, 0]))],
                  [0.0, 0.0, 1.0]])

    # Jacobian of f(X,u) wrt u
    B = np.array([[float(DT * math.cos(x[2, 0])), 0.0],
                  [float(DT * math.sin(x[2, 0])), 0.0],
                  [0.0, DT]])

    return A, B


# --- Observation model related functions

def observation(xTrue, xd, uTrue, Landmarks):
    """
    Bearing-only perception: y = [angle, id]
    """
    xTrue = motion_model(xTrue, uTrue)

    y = np.zeros((0, 2))  # angle, id
    for i in range(len(Landmarks[:, 0])):
        dx = Landmarks[i, 0] - xTrue[0, 0]
        dy = Landmarks[i, 1] - xTrue[1, 0]
        d  = math.hypot(dx, dy)
        if d <= MAX_RANGE:
            angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
            angle_n = angle + np.random.randn() * (Py_sim[1, 1] ** 0.5)
            yi = np.array([angle_n, i])
            y = np.vstack((y, yi))

    # noisy control for dead-reckoning
    u = np.array([[uTrue[0, 0] + np.random.randn() * Q_sim[0, 0] ** 0.5,
                   uTrue[1, 0] + np.random.randn() * Q_sim[1, 1] ** 0.5]]).T
    xd = motion_model(xd, u)
    return xTrue, y, xd, u



def search_correspond_landmark_id(xEst, PEst, yi):
    """
    Landmark association with Mahalanobis distance
    """

    nLM = calc_n_lm(xEst)

    min_dist = []

    for i in range(nLM):
        innov, S, H = calc_innovation(xEst, PEst, yi, i)
        min_dist.append(innov.T @ np.linalg.inv(S) @ innov)
        

    min_dist.append(M_DIST_TH)  # new landmark

    min_id = min_dist.index(min(min_dist))

    return min_id


def jacob_h(q, delta, x, i):
    """
    Compute the jacobian of observation model
    """

    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], -delta[0, 0], -q,  -delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * i)),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * (i + 1)))))

    F = np.vstack((F1, F2))

    H = G @ F

    return H

def jacob_h_bearing(x, i):
    """
    H for bearing-only: h = atan2(dy,dx) - theta
    Returns 1 x (3+2n) matrix
    """
    nLM = calc_n_lm(x)
    lm = get_landmark_position_from_state(x, i)
    delta = lm - x[0:2]
    q = (delta.T @ delta)[0, 0]
    dx, dy = delta[0, 0], delta[1, 0]

    # d(angle)/d(...)
    G2 = np.array([[ dy / q, -dx / q, -1.0, -dy / q, dx / q ]])  # 1x5

    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * i)),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * (i + 1)))))
    F  = np.vstack((F1, F2))
    H  = G2 @ F
    return H, q, delta



def jacob_augment(x, y):
    """
    Compute the jacobians for extending covariance matrix
    """
    
    Jr = np.array([[1.0, 0.0, -y[0] * math.sin(x[2,0] + y[1])],
                   [0.0, 1.0, y[0] * math.cos(x[2,0] + y[1])]])

    Jy = np.array([[math.cos(x[2,0] + y[1]), -y[0] * math.sin(x[2,0] + y[1])],
                   [math.sin(x[2,0] + y[1]), y[0] * math.cos(x[2,0] + y[1])]])

    return Jr, Jy


# --- Kalman filter related functions

def calc_innovation(xEst, PEst, y, LMid):
    """
    Compute innovation and Kalman gain elements
    """

    # Compute predicted observation from state
    lm = get_landmark_position_from_state(xEst, LMid)
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    y_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    yp = np.array([[math.sqrt(q), pi_2_pi(y_angle)]])

    # compute innovation, i.e. diff with real observation
    innov = (y - yp).T
    innov[1] = pi_2_pi(innov[1])

    # compute matrixes for Kalman Gain
    H = jacob_h(q, delta, xEst, LMid)
    S = H @ PEst @ H.T + Py
    
    return innov, S, H

def calc_innovation_bearing(xEst, PEst, angle_meas, LMid):
    lm = get_landmark_position_from_state(xEst, LMid)
    delta = lm - xEst[0:2]
    y_angle_pred = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    innov = np.array([[pi_2_pi(angle_meas - y_angle_pred)]])  # 1x1
    H, _, _ = jacob_h_bearing(xEst, LMid)
    S = H @ PEst @ H.T + Py  # 1x1
    return innov, S, H



def ekf_slam(xEst, PEst, u, y):
    global next_group_id

    S_ = STATE_SIZE
    # Predict
    A, B = jacob_motion(xEst[0:S_], u)
    xEst[0:S_] = motion_model(xEst[0:S_], u)
    PEst[0:S_, 0:S_] = A @ PEst[0:S_, 0:S_] @ A.T + B @ Q @ B.T
    PEst[0:S_, S_:] = A @ PEst[0:S_, S_:]
    PEst[S_:, 0:S_] = PEst[0:S_, S_:].T
    PEst = 0.5 * (PEst + PEst.T)

    # Update (bearing-only)
    for k in range(len(y[:, 0])):
        angle_meas = y[k, 0]
        # === Data association: pick the lm (if any) with smallest Mahalanobis on bearing
        nLM = calc_n_lm(xEst)
        best_i, best_d = -1, M_DIST_TH
        for i in range(nLM):
            innov, S, H = calc_innovation_bearing(xEst, PEst, angle_meas, i)
            d = float(innov @ np.linalg.inv(S) @ innov.T)  # scalar
            if d < best_d:
                best_d = d
                best_i = i

        if best_i < 0:
            # --- New perceived direction: create hypothesis bank along the ray
            gid = next_group_id; next_group_id += 1
            for r in UDI_R_LIST:
                xEst, PEst = add_landmark_hypothesis(xEst, PEst, r, angle_meas, gid)
            continue  # no correction this step (we just seeded hypotheses)
        else:
            # --- EKF correction using the MOST LIKELY hypothesis among the same group
            # Compute likelihoods for all hypotheses in this group
            gid = lm_groups[best_i]
            idxs = [i for i in range(nLM) if lm_groups[i] == gid]
            lik = []
            for i in idxs:
                innov, S, H = calc_innovation_bearing(xEst, PEst, angle_meas, i)
                L = float(np.exp(-0.5 * innov * (np.linalg.inv(S)) * innov) / np.sqrt(2*np.pi*S))
                lik.append(L)
            # normalize & update weights
            lik = np.array(lik).reshape(-1)
            lik = lik / (np.sum(lik) + 1e-12)
            for j, i in enumerate(idxs):
                lm_weights[i] *= lik[j]

            # pick the max-weight hypothesis
            i_star = idxs[int(np.argmax([lm_weights[i] for i in idxs]))]

            # EKF update only with i_star
            innov, S, H = calc_innovation_bearing(xEst, PEst, angle_meas, i_star)
            K = (PEst @ H.T) @ np.linalg.inv(S)
            xEst = xEst + K @ innov
            PEst = (np.eye(len(xEst)) - K @ H) @ PEst
            PEst = 0.5 * (PEst + PEst.T)

            # prune weak hypotheses from the same group
            # (recompute nLM each time because indices shift after pruning)
            i = 0
            while i < calc_n_lm(xEst):
                if lm_groups[i] == gid and lm_weights[i] < UDI_PRUNE_THRESH and i != i_star:
                    xEst, PEst = prune_landmark(xEst, PEst, i)
                    # do not increment i (list shrank)
                else:
                    i += 1

    xEst[2] = pi_2_pi(xEst[2])
    return xEst, PEst



def ring_landmarks(R=12.5, cx=0.0, cy=12.5, n=12):
    th = np.linspace(0, 2*np.pi, n, endpoint=False)
    xs = cx + R*np.cos(th)
    ys = cy + R*np.sin(th)
    return np.vstack([xs, ys]).T

# --- Main script

def main():
    print(__file__ + " start!!")

    time = 0.0

    """

    # Define landmark positions [x, y] original code
    Landmarks = np.array([[0.0, 5.0],
                          [11.0, 1.0],
                          [3.0, 15.0],
                          [-5.0, 20.0]])

    # short loop, dense map Q1-a
    Landmarks = np.array([[0, 5], [5, 5], [5, 0], [5, -5],
                          [0, -5], [-5, -5], [-5, 0], [-5, 5],
                          [0, 0], [0, 10], [0, -10], [-5, 10], [5, 10]], dtype=float)



    # landmarks for Q1-b
    Landmarks = np.vstack([
        ring_landmarks(R=12.5, cx=0.0, cy=12.5, n=12),   # dense ring along the path
        ring_landmarks(R=9.0,  cx=0.0, cy=12.5, n=8),    # inner ring to keep something in range
        [[0.0, 0.0]]                                     # (optional) center landmark
    ]).astype(float)

    

    # landmark positions for Q1-c
    Landmarks = np.array([[0, 0], [0, 2], [0, -2],
                          [2, 0], [-2, 0], [2,2], 
                          [-2,2], [2,-2], [-2,-2]])

"""
    # landmark positions for Q4 (sparse landmarks)
    Landmarks = np.array([[12.0, 12.5]])


    # Init state vector [x y yaw]' and covariance for Kalman
    xEst = np.zeros((STATE_SIZE, 1))
    PEst = initPEst

    # Init true state for simulator
    xTrue = np.zeros((STATE_SIZE, 1))

    # Init dead reckoning (sum of individual controls)
    xDR = np.zeros((STATE_SIZE, 1))

    # Init history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hxError = np.abs(xEst-xTrue)  # pose error
    hxVar = np.sqrt(np.diag(PEst[0:STATE_SIZE,0:STATE_SIZE]).reshape(3,1))  #state std dev


    # counter for plotting
    count = 0

    while  time <= SIM_TIME:
        count = count + 1
        time += DT

        # Simulate motion and generate u and y
        uTrue = calc_input()
        xTrue, y, xDR, u = observation(xTrue, xDR, uTrue, Landmarks)

        xEst, PEst = ekf_slam(xEst, PEst, u, y)

        # store data history
        hxEst = np.hstack((hxEst, xEst[0:STATE_SIZE]))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        err = xEst[0:STATE_SIZE]-xTrue
        err[2] = pi_2_pi(err[2])
        hxError = np.hstack((hxError,err))
        hxVar = np.hstack((hxVar,np.sqrt(np.diag(PEst[0:STATE_SIZE,0:STATE_SIZE]).reshape(3,1))))


        if show_animation and count%15==0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            
            ax1.cla()
            
            # Plot true landmark and trajectory
            ax1.plot(Landmarks[:, 0], Landmarks[:, 1], "*k")
            ax1.plot(hxTrue[0, :], hxTrue[1, :], "-k", label="True")

            # Plot odometry trajectory
            ax1.plot(hxDR[0, :], hxDR[1, :], "-g", label="Odom")

            # Plot estimated trajectory, pose and landmarks
            ax1.plot(hxEst[0, :], hxEst[1, :], "-r", label="EKF")
            ax1.plot(xEst[0], xEst[1], ".r")
            plot_covariance_ellipse(xEst[0: STATE_SIZE],
                                    PEst[0: STATE_SIZE, 0: STATE_SIZE], ax1, "--r")

            for i in range(calc_n_lm(xEst)):
                id = STATE_SIZE + i * 2
                ax1.plot(xEst[id], xEst[id + 1], "xr")
                plot_covariance_ellipse(xEst[id:id + 2],
                                        PEst[id:id + 2, id:id + 2], ax1, "--r")



            #ax1.axis([-12, 12, -2, 22])
            ax1.axis([-17, 17, -2, 27])
            ax1.grid(True)
            ax1.legend()
            
            # plot errors curves
            ax3.plot(hxError[0, :],'b')
            ax3.plot(3.0 * hxVar[0, :],'r')
            ax3.plot(-3.0 * hxVar[0, :],'r')
            ax3.set_ylabel('x')
            ax3.set_title('Real error (blue) and 3 $\sigma$ covariances (red)')
            
            ax4.plot(hxError[1, :],'b')
            ax4.plot(3.0 * hxVar[1, :],'r')
            ax4.plot(-3.0 * hxVar[1, :],'r')
            ax4.set_ylabel('y')

            ax5.plot(hxError[2, :],'b')
            ax5.plot(3.0 * hxVar[2, :],'r')
            ax5.plot(-3.0 * hxVar[2, :],'r')
            ax5.set_ylabel(r"$\theta$")

            plt.pause(0.001)


    plt.savefig('EKFSLAM_Q4.png')

    tErrors = np.sqrt(np.square(hxError[0, :]) + np.square(hxError[1, :]))
    oErrors = np.sqrt(np.square(hxError[2, :]))
    print("Mean (var) translation error : {:e} ({:e})".format(np.mean(tErrors), np.var(tErrors)))
    print("Mean (var) rotation error : {:e} ({:e})".format(np.mean(oErrors), np.var(oErrors)))    # keep window open
    print("Press Q in figure to finish...")
    plt.show()

if __name__ == '__main__':
    main()
