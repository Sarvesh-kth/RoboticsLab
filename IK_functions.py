#! /usr/bin/env python3
import numpy
from numpy import sin,cos,arctan2, sqrt
"""
    # Sarvesh Rajkumar
    # sarvesh@kth.se
"""
L0 = 0.07
L1 = 0.3
L2 = 0.35

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    
    #reducing distance from base to first joint
    x = x - L0

    
    cosq2 = (x**2 + y**2 -L1**2-L2**2)/(2*L1*L2)
    
    # consider sinq2 always positive as the second joint angle considered always between 0 - 180
    sinq2 = sqrt(1-cosq2**2)
    Q2 = arctan2(sinq2,cosq2)
    
    sinq1 = (y*(L1 +L2*cosq2)- L2*sinq2*x)/((L2*sinq2)**2 + (L1 + L2*cosq2)**2)
    
    cosq1= sqrt(1-sinq1**2)
    
    #test for quadrant 3 for link 1 using circle from joint 2. Check if point within circle or not    
    if x**2 + (y+L1)**2 <= L2**2 and y < 0:
        cosq1= - cosq1   

    Q1 = arctan2(sinq1,cosq1)

    q = [Q1,Q2,z]
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements
    # q = [0,0,0,0,0,0,0]
    target = numpy.matrix([[x],[y],[z]])
    # print("Target ",target)
    """
    Fill in your IK solution here and return the seven joint values in q
    Matrix of Ai_i+1
    """
    ali = [numpy.pi/2,-numpy.pi/2,-numpy.pi/2,numpy.pi/2,numpy.pi/2,-numpy.pi/2,0]
    Di = [0,0,0.4,0,0.39,0,0]
    Ai = [0,0,0,0,0,0,0]
    thresh = 1e-2
    Ti = numpy.array(
        [[1,0,0,0],
         [0,1,0,0],
         [0,0,1,0.311],
         [0,0,0,1]]
    )
    Te = numpy.array(
        [[1,0,0,0],
         [0,1,0,0],
         [0,0,1,0.078],
         [0,0,0,1]]
    )
    ixx = 0
    while True:
        ixx+=1
        Tmat =[]
        
        for i in range(0,7):
            matrix =  numpy.array(
                    [[cos(q[i]),-1*sin(q[i])*cos(ali[i]),sin(q[i])*sin(ali[i]),0],
                    [sin(q[i]),cos(q[i])*cos(ali[i]),-1*cos(q[i])*sin(ali[i]),0],
                    [ 0, sin(ali[i]),  cos(ali[i]), Di[i]],
                    [0,0,0,1]])
            
            Tmat.append(matrix)
        
        Tmul = []
        for i in range(0,7):
            if i == 0:
                Tmul.append(Tmat[i])
            else:
                Tmul.append(numpy.matmul(Tmul[i-1],Tmat[i]))
        
        jacobian = numpy.zeros((6,7))
        
        F_Tmul1 = numpy.matmul(Ti,Tmul[-1])
        
        F_Tmul = numpy.matmul(F_Tmul1,Te)

        curpos = F_Tmul[:3,[3]]

        for i in range(0,7):
            
            
            
            if i == 0:
                zi = numpy.array([[0],[0],[1]])
                pi = numpy.array([[0],[0],[0]])
            else:
                pi = Tmul[i-1][:3,[3]]
                zi = Tmul[i-1][:3,[2]]
            pdiff = curpos - pi
            zi = zi.reshape(3,1)
            
            cross = numpy.cross(zi.T,pdiff.T).T
            cross = cross.reshape(3,1)
            
            jacobian[:3,[i]] = cross
            jacobian[3:,[i]] = zi
        
        J_psuedo = numpy.linalg.pinv(jacobian)
        E_pose = curpos - target
        re = F_Tmul[:3,:3]        
        
        re = numpy.array(re)
        R = numpy.array(R)
        ned = numpy.cross(re[:3,[0]].T,R[:3,[0]].T).T
        sed = numpy.cross(re[:3,[1]].T,R[:3,[1]].T).T
        aed = numpy.cross(re[:3,[2]].T,R[:3,[2]].T).T
        E_or = 1/2*(ned + sed + aed)
        
        Ex = numpy.vstack((E_pose,E_or))
        Exnorm = numpy.linalg.norm(Ex)
        
        Eq = numpy.matmul(J_psuedo,Ex)        
        for i in range(7):            
            q[i] = q[i] - Eq[i,[0]]
            q[i] = q[i].item()
        
        if Exnorm <= thresh:
            break
    
    return q
