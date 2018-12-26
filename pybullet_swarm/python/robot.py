import numpy as np
import pybullet as p
import itertools
import math

class Robot():
    cnt=0
    time=0
    """ 
    The class is the interface to a single robot
    """
    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.reset()

        # No friction between bbody and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=5., rollingFriction=0.)

        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=5., rollingFriction=0.)
            
        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []
        

    def reset(self):
        """
        Moves the robot back to its initial position 
        """
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))
            
    def set_wheel_velocity(self, vel):
        """ 
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel) 
        """
        assert len(vel) == 2, "Expect velocity to be array of size two"
        p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
            targetVelocities=vel)

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]
    
    def get_messages(self):
        """
        returns a list of received messages, each element of the list is a tuple (a,b)
        where a= id of the sending robot and b= message (can be any object, list, etc chosen by user)
        Note that the message will only be received if the robot is a neighbor (i.e. is close enough)
        """
        return self.messages_received
        
    def send_message(self, robot_id, message):
        """
        sends a message to robot with id number robot_id, the message can be any object, list, etc
        """
        self.messages_to_send.append([robot_id, message])
        
    def get_neighbors(self):
        """
        returns a list of neighbors (i.e. robots within 2m distance) to which messages can be sent
        """
        return self.neighbors
    
    def compute_controller(self):
        """ 
        function that will be called each control cycle which implements the control law
        TO BE MODIFIED
        
        we expect this function to read sensors (built-in functions from the class)
        and at the end to call set_wheel_velocity to set the appropriate velocity of the robots
        """
        
        # here we implement an example for a consensus algorithm
        neig = self.get_neighbors()
        messages = self.get_messages()
        pos, rot = self.get_pos_and_orientation()
        
        #send message of positions to all neighbors indicating our position
        for n in neig:
            self.send_message(n, pos)
        
        # check if we received the position of our neighbors and compute desired change in position
        # as a function of the neighbors (message is composed of [neighbors id, position])
        p_des=np.zeros((100,6,2))

        p_des[0,:]=[[0.5,-0.5],[0.5,0.5],[1.5,0.5],[0.5,1.5],[2.5,-0.5],[2.5,1.5]] # sqaure formation
        p_des[1,:]=[[0.75,-0.5],[1.5,-0.5],[2.5,-0.5],[1.5,0.5],[2.5,0.5],[2.5,1.5]] #2nd formation
        p_des[2,:]=[[0.75,-0.5],[1.5,-0.5],[2.5,-0.5],[2.5,0.5],[2.5,1.5],[2.5,2.5]] #L formation
        p_des[3,:]=[[1.5,-0.5],[2.5,-0.5],[2.5,0.5],[2.5,1.5],[2.5,2.5],[2.5,3.5]] #L formation
        p_des[4,:]=[[2.5,-0.5],[2.5,0.5],[2.5,1.5],[2.5,2.5],[2.5,3.5],[2.5,4.5]] #Line formation
        p_des[5,:]=[[2.5,1],[2.5,2],[2.5,3],[2.5,4],[2.5,5],[2.5,6]] #Line formation
        p_des[6,:]=[[2.5,3],[2.5,4],[2.5,5],[2.5,6],[1.5,6],[0.5,6]] #L formation
        p_des[7,:]=[[2.5,6],[1.5,6],[0.5,6],[-0.5,6],[-1.5,6],[-2.5,6]] #Line formation
        p_des[8,:]=[[1.5,5.5],[0.5,5.5],[-0.5,5.5],[-1.5,5.5],[-2.5,5.5],[-3.5,5.5]] #Line formation
        p_des[9,:]=[[0.5,5.5],[-0.5,5.5],[-1.5,5.5],[-2.5,5.5],[-3.5,5.5],[-4.5,5.5]] #Line formation
        p_des[10,:]=[[-1.5,5.5],[-2.5,5.5],[-3.5,5.5],[-4.5,5.5],[-4.5,6.5],[-4.5,7.5]] #L formation
        p_des[11,:]=[[-2.5,5.5],[-3.5,5.5],[-4.5,5.5],[-4.5,6.5],[-4.5,7.5],[-4.5,8.5]] #L formation
        p_des[12,:]=[[-3.5,5.5],[-4.5,5.5],[-4.5,6.5],[-4.5,7.5],[-4.5,8.5],[-4.5,9.5]] #Line formation
        p_des[13,:]=[[-4.5,5.5],[-4.5,6.5],[-4.5,7.5],[-4.5,8.5],[-4.5,9.5],[-4.5,10.5]] #Line formation
        p_des[14,:]=[[-3.5,8.5],[-3.5,9.5],[-3.5,10.5],[-3.5,11.5],[-3.5,12.5],[-3.5,13.5]] #Line down formation
        p_des[15:100,:]=[[-3.5,8.5],[-3.5,9.8],[-5.4,11],[-3.5,11],[-3.5,12.2],[-3.5,13.5]]#tri
        #p_des[16,:]=[[-3.5,8.5],[-3.5,9.8],[-5.4,11],[-3.5,11],[-3.5,12.2],[-3.5,13.5]]#tri
        
        #p_des[16,:]=[[-4.5,10.7],[-3.5,9.7],[-3.5,11.5],[-3.5,12.8],[-4.5,12.8],[-5.5,11.5]]
        #p_des[17,:]=[[-4.5,10.7],[-3.5,9.7],[-3.5,11.5],[-3.5,12.8],[-4.5,12.8],[-5.5,11.5]]





        kf=10
        df=2*math.sqrt(kf)
        kt=5
        dt=2*math.sqrt(kt)
        L=np.zeros((1,6))
        if messages:
            for m in messages:
                L[0][m[0]]=-1

            L[0][self.id]=len(messages)
         
        #print(L)        
        # dx = kf*L*(p_des-pos[0])
        # dy = kf*L*(p_des-pos[1])
        d_c_x=np.zeros((6,1))
        d_c_y=np.zeros((6,1))

        if messages:
            for m in messages:
                d_c_x[m[0]]=p_des[Robot.cnt][m[0]][0]-m[1][0]
                d_c_y[m[0]]=p_des[Robot.cnt][m[0]][1]-m[1][1]

        d_c_x[self.id]=p_des[Robot.cnt][self.id][0]-pos[0]
        d_c_y[self.id]=p_des[Robot.cnt][self.id][1]-pos[1]  

        dx=kf*np.matmul(L,d_c_x)+kt*(p_des[Robot.cnt][self.id][0]-pos[0])
        dy=kf*np.matmul(L,d_c_y)+kt*(p_des[Robot.cnt][self.id][1]-pos[1])    
    
            #     dx += m[1][0] - pos[0]
            #     dy += m[1][1] - pos[1]

                   
            # integrate
        des_pos_x = pos[0] + self.dt * dx
        des_pos_y = pos[1] + self.dt * dy
        
        #compute velocity change for the wheels
        vel_norm = np.linalg.norm([dx, dy]) #norm of desired velocity
        if vel_norm < 0.01:
            vel_norm = 0.01
        des_theta = np.arctan2(dy/vel_norm, dx/vel_norm)
        right_wheel = np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
        left_wheel = -np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
        self.set_wheel_velocity([left_wheel, right_wheel])

        Robot.time+=1
        print(Robot.time)
        if Robot.time==12000:
            Robot.cnt+=1
            Robot.time=0

    

    
       
