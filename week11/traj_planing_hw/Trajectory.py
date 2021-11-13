import numpy as np


class Trajectory:
    def __init__(self):
        pass

    # q = [q1, ..., q6]
    def Linear(self, q0, qf, v, t_step):
        num_axis = len(q0)
        q0 = np.array(q0)
        qf = np.array(qf)
        v = np.array(v)

        if np.allclose(q0, qf):
            t = [0.0]
            joint = [qf]
            return t, joint

        # Design variables
        tf = abs((qf - q0) / v)  # total time taken

        # Calculate trajectories
        t = np.arange(start=0.0, stop=tf, step=t_step)
        joint = []
        for i in range(num_axis):
            # joint traj.
            q = (qf[i] - q0[i]) / tf[i] * t + q0[i]
            joint.append(q)
        joint = np.array(joint).T
        assert ~np.isnan(t).any()
        assert ~np.isnan(joint).any()
        return t, joint

    @classmethod
    def cubic(cls, q0, qf, v0, vf, tf, t_step=0.01):
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)
        if np.allclose(q0, qf):
            t = [0.0]
            q_pos = [qf]
            return q_pos, t

        # Define coefficients
        a0 = q0
        a1 = v0
        a2 = 3*(qf-q0)/(tf**2) - 2/tf*v0 - 1/tf*vf
        a3 = -2*(qf-q0)/(tf**3) + 1/(tf**2)*(v0+vf)

        # Calculate trajectories
        t = np.arange(start=0.0, stop=tf, step=t_step).reshape(-1, 1)

        # joint traj.
        q_pos = a0 + a1*t + a2*t**2 + a3*t**3
        q_vel = a1 + 2*a2*t + 3*a3*t**2
        q_acc = 2*a2 + 6*a3*t
        assert ~np.isnan(t).any()
        assert ~np.isnan(q_pos).any()
        return q_pos, q_vel, q_acc, t

    @classmethod
    def LSPB(cls, q0, qf, tf, tb, t_step=0.01):
        q0 = np.array(q0)
        qf = np.array(qf)
        if np.allclose(q0, qf):
            t = [0.0]
            q_pos = [qf]
            return q_pos, t

        # Define coefficients
        ab = (q0-qf)/(tb-tf)/tb
        A = np.array([[tb, -tb, 0.0, 1.0, -1.0, 0.0],
                      [0.0, -(tf-tb), tf-tb, 0.0, -1.0, 1.0],
                      [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, tf, 0.0, 0.0, 1.0]])
        b = np.block([[-ab*tb**2/2], [ab*(tf-tb)**2/2], [np.zeros_like(q0)], [q0], [ab*tf], [qf+ab*tf**2/2]])
        coeff = np.linalg.inv(A).dot(b)
        C1 = coeff[0]
        C2 = coeff[1]
        C3 = coeff[2]
        C4 = coeff[3]
        C5 = coeff[4]
        C6 = coeff[5]

        # Calculate trajectories
        t = np.arange(start=0.0, stop=tf, step=t_step)
        t1 = t[t < tb].reshape(-1, 1)
        t2 = t[(tb <= t) & (t < tf - tb)].reshape(-1, 1)
        t3 = t[tf - tb <= t].reshape(-1, 1)

        # Combine joint trajectories
        traj1 = ab/2*t1**2 + C1*t1 + C4
        traj2 = C2*t2 + C5
        traj3 = -ab/2*t3**2 + C3*t3 + C6
        q_pos = np.concatenate((traj1, traj2, traj3))
        assert ~np.isnan(t).any()
        assert ~np.isnan(q_pos).any()
        return q_pos, t


if __name__ == '__main__':
    # Constraints
    q0 = [1.0, 2.0]
    qf = [0.0, -2.0]
    v0 = [1.0, -1.0]
    vf = [1.0, -1.0]
    tf = 3.0

    # Get trajectories
    q_pos, t = Trajectory.cubic(q0=q0, qf=qf, v0=v0, vf=vf, tf=tf)
    q_pos2, t2 = Trajectory.LSPB(q0=q0, qf=qf, tf=tf, tb=tf/3)

    import matplotlib.pyplot as plt
    # Plot cubic
    plt.figure(0)
    plt.plot(t, q_pos[:, 0], 'b-')
    plt.plot(t, q_pos[:, 1], 'r-')

    # Plot LSPB
    plt.figure(1)
    plt.plot(t2, q_pos2[:, 0], 'b-')
    plt.plot(t2, q_pos2[:, 1], 'r-')
    plt.show()