import vrep
import time
import numpy as np
from scipy.linalg import expm, logm
from numpy.linalg import inv, norm
import random 

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def invskew(v):
    return np.array([[v[2][1]],[v[0][2]],[v[1][0]]])

def bracket4(v):
    return np.block([ [skew(v[0:3, :]), v[3:6, :]],[ 0,0,0,0] ])   


def adt(T):
    """
    Returns the adjoint transformation matrix of T
    :param T: the pose whose 6x6 adjoint matrix to return
    """
    
    return np.block([[ T[:3,:3],                   np.zeros((3,3)) ],
                           [ skew(T[:3,3:]).dot(T[:3,:3]), T[:3,:3]             ]])    

# Getthing the goalpose from user 
n=4
numberjoints=6    #int(input('How many joints does the robot have'))

goalpose=np.identity(4)
for i in range (0,n-1):
    for j in range (0,n):
        goalpose[i][j]=float(input('Please enter the goal pose row by row from left to right'))
        
print('This is the goal pose of the robot: \n', goalpose)

goal_position=goalpose[0:3,3]

# Homogenous transformation matrix

M = np.array([ [0, 0, -1, -172.3/1000], [0 ,1, 0, 0], [1 ,0 ,0 ,986.76/1000] ,[0 ,0 ,0 ,1]])

# Matrix of spatial twist axes
a1 = np.array([[0],[0],[1]])
a2 = np.array([[-1],[0],[0]])
a3 = np.array([[-1],[0],[0]])
a4 = np.array([[-1],[0],[0]])
a5 = np.array([[0],[0],[1]])
a6 = np.array([[-1],[0],[0]])




q1 = np.array([[0],[0],[0]])
q2 = np.array([[0],[0],[0.0746]])
q3 = np.array([[0],[0],[0.4997]])
q4 = np.array([[0],[0],[0.892]])
q5 = np.array([[-0.110],[0],[0]]) 
q6 = np.array([[0],[0],[0.9866]])


S1 = np.block([[a1],[-skew(a1).dot(q1)]])
S2 = np.block([[a2],[-skew(a2).dot(q2)]])
S3 = np.block([[a3],[-skew(a3).dot(q3)]])
S4 = np.block([[a4],[-skew(a4).dot(q4)]])
S5 = np.block([[a5],[-skew(a5).dot(q5)]])
S6 = np.block([[a6],[-skew(a6).dot(q6)]])
S=np.block([S1,S2,S3,S4,S5,S6])

# Generate random joint angles to start the algorithm 
theta1= random.random()
theta2= random.random()
theta3= random.random()
theta4= random.random()
theta5= random.random()
theta6= random.random()


joint_theta = np.block([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])



#M = np.array([[-1,  0,  0, -8], [ 0, -1,  0, -8], [ 0,  0,  1,  0], [ 0,  0,  0,  1]])
#S = np.array([[ 1,  0, -1,  0,  0,  0], [ 0,  0,  0, -1, -1,  1], [ 0,  1,  0,  0,  0,  0], [ 0, -4,  0,  0,  0,  0], [ 0, -2,  0,  0,  0,  0], [ 2,  0, -4,  0,  4, -6]])
#print('S/n',S)
# Inverse kinematics algorithm 
error=0.01
Vs=np.array([1,2,3,4,5,6])

while norm(Vs) > 0.001 :

    e1=expm(bracket4(S[:6 ,:1])*joint_theta[0])
    e2=expm(bracket4(S[:6 ,1:2])*joint_theta[1])
    e3=expm(bracket4(S[:6 ,2:3])*joint_theta[2])
    e4=expm(bracket4(S[:6 ,3:4])*joint_theta[3])
    e5=expm(bracket4(S[:6 ,4:5])*joint_theta[4])
    e6=expm(bracket4(S[:6 ,5:6])*joint_theta[5])

    j1=S[:6 ,:1]
    j2=adt(e1).dot(S[:6 ,1:2])
    j3=adt(e1.dot(e2)).dot(S[:6 ,2:3])
    j4=adt(e1.dot(e2).dot(e3)).dot(S[:6 ,3:4])
    j5=adt(e1.dot(e2).dot(e3).dot(e4)).dot(S[:6 ,4:5])
    j6=adt(e1.dot(e2).dot(e3).dot(e4).dot(e5)).dot(S[:6 ,5:6])
    J=np.block([j1,j2,j3,j4,j5,j6])

    #print('J\n',J)


    T10=e1.dot(e2).dot(e3).dot(e4).dot(e5).dot(e6).dot(M)
    Vs_bracket=logm(goalpose.dot(inv(T10)))
    #print('Vs_brackst\n', Vs_bracket)
    #print('goalpoose\n', goalpose)
    #print('invT10\n',inv(T10))
    #print('T_1in0\n', T10)

    Vs=np.block([[invskew(Vs_bracket[:3,:3])],[Vs_bracket[:3,3:]]])

    #print('Vs\n',Vs)
    w=0.1

    thetadot=inv(np.transpose(J).dot(J)+w*np.identity(numberjoints)).dot(np.transpose(J)).dot(Vs)
    #print('thetadot',thetadot)
    #print('norm(Vs)\n',norm(Vs))
    #print('theta\n',theta)
    joint_theta=joint_theta+thetadot

print('theta= ', joint_theta)
print('this is the error ', norm(Vs) )


#Connecting to Vrep and Start simulation

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# Get "handle" to the Dummy Object
result, dummy_handle = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for dummy object')

# Get "handle" to the first joint of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

    # Get "handle" to the second joint of robot
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for second joint')

 # Get "handle" to the third joint of robot
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for third joint')


# Get "handle" to the fourth joint of robot
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')

 # Get "handle" to the fifth joint of robot
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')   

 # Get "handle" to the sixth joint of robot
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')       

# Get "handle" to the tool 

result, tool_handle = vrep.simxGetObjectHandle(clientID, 'UR5_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for tool')

#Get "handle" to the robot 

result, assembly_handle = vrep.simxGetObjectHandle(clientID, 'UR5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for the assembly')

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)


#Place the dummy object in the goal pose
vrep.simxSetObjectPosition (clientID, dummy_handle,assembly_handle, goal_position, vrep.simx_opmode_oneshot)

time.sleep(2)


# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, float(joint_theta[0]), vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

time.sleep(2)



# Set the desired value of the second joint variable
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, float(joint_theta[1]), vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# Get the current value of the second joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get second joint variable')
print('current value of second joint variable: theta = {:f}'.format(theta))

time.sleep(2)



# Set the desired value of the third joint variable
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, float(joint_theta[2]), vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)


# Set the desired value of the fourth joint variable
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, float(joint_theta[3]), vrep.simx_opmode_oneshot)



#Wait two seconds
time.sleep(2)


# Set the desired value of the fifth joint variable
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, float(joint_theta[4]), vrep.simx_opmode_oneshot)



#Wait two seconds
time.sleep(2)

# Set the desired value of the sixth joint variable
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, float(joint_theta[5]), vrep.simx_opmode_oneshot)

#Wait two seconds
time.sleep(2)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)



