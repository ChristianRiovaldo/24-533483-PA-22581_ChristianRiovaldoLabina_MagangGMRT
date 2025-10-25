import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def fk_matriks(l, d, theta_derajat, alpha_derajat):
    theta = np.radians(theta_derajat)
    alpha = np.radians(alpha_derajat)
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), l*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), l*np.sin(theta)],
        [0,             np.sin(alpha),                np.cos(alpha),               d              ],
        [0,             0,                            0,                           1              ]
    ])
    
def forward_kinematics():
    banyak_dof = int(input('Banyak dof yang ingin dihitung: '))
    dimensi = None

    while True:
        dimensi = int(input('Pilih dimensi (2/3): '))
        if dimensi in [2,3] :
            break
        else:
            print('Hanya bisa 2D atau 3D')
        
    dof = []

    if dimensi == 2:
        for i in range(banyak_dof):
            print(f'\nDOF ke-{i+1}:')
            panjang_l = float(input(f'l{i+1} = '))
            sudut_theta = float(input(f'theta{i+1} = '))
            dof.append(fk_matriks(panjang_l, 0.0, sudut_theta, 0.0))
    elif dimensi == 3:
        for i in range(banyak_dof):
            print(f'\nDOF ke-{i+1}:')
            panjang_l = float(input(f'l{i+1} = '))
            sudut_theta = float(input(f'theta{i+1} = '))
            panjang_d = float(input(f'd{i+1} = '))
            sudut_alpha = float(input(f'alpha{i+1} = '))
            dof.append(fk_matriks(panjang_l, panjang_d, sudut_theta, sudut_alpha))    

    fk = np.eye(4)
    koordinat = [[0,0,0]]
    
    for i in range(banyak_dof):
        fk @= dof[i]
        koordinat.append(fk[:3,3].copy())
        
    print('\nForward Kinematics =\n', fk)
    koordinat = np.array(koordinat)
        
    if dimensi == 2:
        plt.figure(figsize=(6,6))
        plt.plot(koordinat[:,0],koordinat[:,1],'-o',color='r')
        plt.title('Visualisasi Forward Kinematics 2D')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    elif dimensi == 3:
        fig = plt.figure(figsize=(6,6))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(koordinat[:,0],koordinat[:,1],koordinat[:,2],'-o',color='r')
        ax.set_title('Visualisasi Forward Kinematics 3D')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.grid(True)
        plt.show()
    
def inverse_kinematics():
    print('End-effector planar:')
    x = float(input('x = '))
    y = float(input('y = '))
    l1 = float(input('l1 = '))
    l2 = float(input('l2 = '))

    theta2 = np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
    theta1 = np.arctan(y/x)-np.arctan((l2*np.sin(theta2))/(l1+l2*np.cos(theta2)))

    theta2 = np.degrees(theta2)
    theta1 = np.degrees(theta1)

    print('\nInverse Kinematics:')
    print(f'theta1 = {theta1}')
    print(f'theta2 = {theta2}')
    
pilih = input('Menghitung forward kinematics atau inverse kinematics (fk/ik)? ')

if pilih == 'fk':
    forward_kinematics()
elif pilih == 'ik':
    inverse_kinematics()
else:
    print('Pilihan tidak valid')