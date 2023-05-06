import numpy as np
import math

# D-H parameters of kinova Gen3 Lite
D_H = np.array([[     0.0,     0.0, (128.3+115.0)/1000,     0.0],
                [     0.0, np.pi/2,            30/1000, np.pi/2],
                [   0.280,   np.pi,            20/1000, np.pi/2],
                [     0.0, np.pi/2, (140.0+105.0)/1000, np.pi/2],
                [     0.0, np.pi/2,   (28.5+28.5)/1000,     0.0],
                [     0.0,-np.pi/2, (105.0+130.0)/1000,-np.pi/2]], dtype=np.float32)

def transformation_matrix(joint_angles: np.ndarray, base_idx=0, end_idx=6) -> np.ndarray:
    '''
    Compute the transformation matrix from the base frame to the end-effector frame

    Parameters
    ----------
    joint_angles : np.ndarray
        Joint angles of the kinova Gen3 Lite
        size: (6, 1)(default)
    base_idx : int
        Index of the base frame
        default: 0
    end_idx : int
        Index of the end-effector frame
        default: 6
    Note: end_idx - base_idx = len(joint_angles)        

    Returns
    -------
    T : np.ndarray
        Transformation matrix from the base frame to the end-effector frame
        size: (4, 4)
    '''
    # Compute the transformation matrix
    a_i_1 = D_H[base_idx:end_idx, 0]
    alpha_i_1 = D_H[base_idx:end_idx, 1]
    d_i = D_H[base_idx:end_idx, 2]
    theta_i = D_H[base_idx:end_idx, 3] + joint_angles

    T = np.eye(4)
    for i in range(base_idx, end_idx):
        screw_x = np.array([[1.0,                  0.0,                   0.0, a_i_1[i]],
                            [0.0, np.cos(alpha_i_1[i]), -np.sin(alpha_i_1[i]),      0.0],
                            [0.0, np.sin(alpha_i_1[i]),  np.cos(alpha_i_1[i]),      0.0],
                            [0.0,                  0.0,                   0.0,      1.0]], dtype=np.float32)
        screw_z = np.array([[np.cos(theta_i[i]), -np.sin(theta_i[i]), 0.0,    0.0],
                            [np.sin(theta_i[i]),  np.cos(theta_i[i]), 0.0,    0.0],
                            [               0.0,                 0.0, 1.0, d_i[i]],
                            [               0.0,                 0.0, 0.0,    1.0]], dtype=np.float32)

        T_i_1_i = np.dot(screw_x, screw_z)
        T = np.dot(T, T_i_1_i)

    return T

def E_to_B(P: np.ndarray, joint_angles: np.ndarray) -> np.ndarray:
    '''
    Compute the position of a point P in the base frame B 
    from the position of P in the end-effector frame E
    
    Parameters
    ----------
    P : np.ndarray
        Position of a point P in the end-effector frame E
        size: (3, 1)
    joint_angles : np.ndarray
        Joint angles of the kinova Gen3 Lite
        size: (6, 1)

    Returns
    -------
    P_B : np.ndarray
        Position of a point P in the base frame B
        size: (3, 1)
    '''
    T_0_6 = transformation_matrix(joint_angles)
    return np.dot(T_0_6, np.array([P[0], P[1], P[2], 1.0]).T)[0:3]

if __name__ == "__main__":
    P = np.array([0.0, 0.0, 0.0]).T
    theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
    print(E_to_B(P, theta))
