def sysCall_init():
    sim = require('sim')
    
    ### ----  Variable Initialization -- #############
    self.infiniteStrength = True
    self.maxPullForce = 3
    self.maxShearForce = 1
    self.maxPeelTorque = 0.1
    self.enabled = True
    ###############################################
    
    ####### ---- Static Dummy ------ ##############
    self.s_static=sim.getObject('/Sensor')
    self.l_static=sim.getObject('/LoopClosureDummy1')
    self.l2_static=sim.getObject('/LoopClosureDummy2')
    self.b_static=sim.getObject('/static_dummy')
    self.suctionPadLink_static=sim.getObject('/moving_Link2')
    
    sim.setLinkDummy(self.l_static, -1)
    sim.setObjectParent(self.l_static,self.b_static, True)
    self.m_static=sim.getObjectMatrix(self.l2_static)
    sim.setObjectMatrix(self.l_static, self.m_static)
    ################################################

    ####### ---- Moving Dummy ------ ##############
    self.s_moving=sim.getObject('/moving_Sensor')
    self.l_moving=sim.getObject('/movingLoopClosureDummy1')
    self.l2_moving=sim.getObject('/movingLoopClosureDummy2')
    self.b_moving=sim.getObject('/moving_dummy')
    self.suctionPadLink_moving=sim.getObject('/moving_Link1')
    
    sim.setLinkDummy(self.l_moving,-1)
    sim.setObjectParent(self.l_moving, self.b_moving, True)
    self.m_moving=sim.getObjectMatrix(self.l2_moving)
    sim.setObjectMatrix(self.l_moving,self.m_moving)
    ################################################


    # do some initialization here
    #
    # Instead of using globals, you can do e.g.:
    # self.myVariable = 21000000

def sysCall_actuation():
    # put your actuation code here
    self.parent_static = sim.getObjectParent(self.l_static)
    self.parent_moving = sim.getObjectParent(self.l_moving)
    
    if not self.enabled :
        if self.parent_static != self.b_static:
            sim.setLinkDummy(self.l_static, -1)
            sim.setObjectParent(self.l_static, self.b_static, True)
            self.m_static=sim.getObjectMatrix(self.l2_static)
            sim.setObjectMatrix(self.l_static, self.m_static)         
            
    else :
        if self.parent_static == self.b_static:
            self.index = 0
            #print('@@@@@@@@@@@')
            while True:
                
                self.shape = sim.getObjects(self.index, sim.object_shape_type)
                
                if self.shape == -1:
                    break
                if self.shape != -1:
                    #print(sim.checkProximitySensor(self.s_static,self.shape))
                    self.res,_,_,_,_ = sim.checkProximitySensor(self.s_static,self.shape)
                    if self.res == 1:
                        self.resp_flag = 1
                    else:
                        self.resp_flag = 0
                
                if (self.shape != self.b_static) and (sim.getObjectInt32Param(self.shape,sim.shapeintparam_respondable) != 0) and (self.resp_flag == 1):
                    #print('$$$$$$$')
                    message,data,data2 = sim.getSimulatorMessage()
                    if (message == sim.message_keypress):
                        if  data[0] == 97:
                            sim.setObjectParent(self.l_static, self.b_static,True)
                            self.m_static=sim.getObjectMatrix(self.l2_static)
                            sim.setObjectMatrix(self.l_static, self.m_static)
                            #### Do the connection:
                            sim.setObjectParent(self.l_static, self.shape,True)
                            sim.setLinkDummy(self.l_static, self.l2_static)
                            self.resp_flag = 0
    
                    else:
                        pass
                    
                    break
                self.index = self.index + 1
                
        else:
            message,data,data2 = sim.getSimulatorMessage()
            if data[0] == 100:
                #print('#######')
                sim.setLinkDummy(self.l_static,-1)
                sim.setObjectParent(self.l_static, self.b_static,True)
                self.m_static=sim.getObjectMatrix(self.l2_static)
                sim.setObjectMatrix(self.l_static,self.m_static)
                self.resp_flag = 0
            
            if (self.infiniteStrength == False):
                result,force,torque=sim.readForceSensor(self.suctionPadLink_static) # Here we read the median value out of 5 values (check the force sensor prop. dialog)
                if (result>0):
                    breakIt=False
                    if (force[3] > maxPullForce): breakIt = True 
                    sf=math.sqrt(force[1]*force[1]+force[2]*force[2])
                    if (sf > maxShearForce): breakIt=True
                    if (torque[1] > maxPeelTorque): breakIt = True
                    if (torque[2] > maxPeelTorque): breakIt = True 
                    if (breakIt):
                        # We break the link:
                        sim.setLinkDummy(self.l_static,-1)
                        sim.setObjectParent(self.l_static, self.b_static,True)
                        self.m_static=sim.getObjectMatrix(self.l2_static)
                        sim.setObjectMatrix(self.l_static,self.m_static)
    
    pass

def sysCall_sensing():
    message, data, data2 = sim.getSimulatorMessage()
    
    if message == sim.message_keypress:
        # Handle Prismatic Joint velocity based on key 'q' and 'e' (closing and opening)
        if data[0] == 113:  # 'q' key for closing the gripper
            sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), -0.050)  # Set a negative velocity to close
        elif data[0] == 101:  # 'e' key for opening the gripper
            sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), 0.050)   # Set a positive velocity to open
        
        # Handle Arm Joint velocity based on key 'w' and 's' (raising and lowering)
        if data[0] == 119:  # 'w' key for raising the gripper
            sim.setJointTargetVelocity(sim.getObject('/arm_joint'), 1)         # Positive velocity to raise
        elif data[0] == 115:  # 's' key for lowering the gripper
            sim.setJointTargetVelocity(sim.getObject('/arm_joint'), -1)        # Negative velocity to lower

    else:
        # If no key is pressed, stop the joints by setting velocity to 0
        sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), 0)
        sim.setJointTargetVelocity(sim.getObject('/arm_joint'), 0)
        
    pass


def sysCall_cleanup():
    # do some clean-up here
    ##### ---------------------- -#######################
    sim.setLinkDummy(self.l_static,-1)
    sim.setObjectParent(self.l_static,self.b_static,True)
    self.m_static=sim.getObjectMatrix(self.l2_static)
    sim.setObjectMatrix(self.l_static,self.m_static)
    #####################################################
    
    ######## ----------------------- #######################
    sim.setLinkDummy(self.l_moving,-1)
    sim.setObjectParent(self.l_moving, self.b_moving, True)
    self.m_moving=sim.getObjectMatrix(self.l2_moving)
    sim.setObjectMatrix(self.l_moving,self.m_moving)
    ########################################################
    
    pass

# See the user manual or the available code snippets for additional callback functions and details