#-------------------------------------------------------------------------------
#
# Load libraries and setup agents
#
#-------------------------------------------------------------------------------

##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()

import numpy as np
import lgsm


##### Create physic and graphic agents
import agents.physic.core
import agents.graphic.simple
import agents.graphic.proto

time_step=.01

# Physic
phy = agents.physic.core.createAgent("physic", 0)                        # physic agent
ms = agents.physic.core.createGVMScene(phy, "main", time_step=time_step) # physic scene
lmd = agents.physic.core.createXCDScene(phy, "xcd", "LMD", lmd_max=.01)  # collision scene

ms.setGeometricalScene(lmd) # link physic scene and collision scene
phy.s.setPeriod(time_step)


#-------------------------------------------------------------------------------
#
# Create a pendulum with GVM
#
#-------------------------------------------------------------------------------

##### Create all rigid bodies and configure
bodies_name = ["b_root", "b_1", "b_2", "b_3"]

bodies = [] # lists of body and joint instances

for b_name in bodies_name:
    b = phy.s.GVM.RigidBody.new(b_name)
    b.setMass(1) # 1kg
    b.setCenterOfGravity(lgsm.zero())               # in body frame
    b.setPrincipalMomentsOfInertia(lgsm.one())      # inertia moments
    b.setPrincipalInertiaFrame(lgsm.Displacement()) # inertia frame
    bodies.append(b)


##### Create all joints and configure
joints_name = ["j_root", "j_1", "j_2", "j_3"]

joints = [] # ists of joint instances

joints.append(phy.s.GVM.FixedJoint.new(joints_name[0]))
joints[0].configure(lgsm.Displacement())     # Set position of fixed joint

for j_name in joints_name[1:]:
    j = phy.s.GVM.HingeJoint.new(j_name)
    j.configure(lgsm.Displacement(lgsm.vector(.5,0,0)), # Displacement from parent body to child body
                lgsm.vector(0,0,0),                     # Hinge center position in parent frame
                lgsm.vector(0,1,0),                     # Hinge axis in parent frame
                0.2)                                    # Hinge angular offset
    j.setJointDampingCoeff(2) # add some damping
    j.enableJointDamping()
    joints.append(j)


##### Create connection between bodies with joints
ms.addRigidBodyToGround(bodies[0], joints[0])
for i in range(1, len(bodies)):
    ms.addRigidBody(bodies[i-1], bodies[i], joints[i])




#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

#phy.s.start()
#graph.s.start()

cc = phy.s.Connectors

#cc.IConnectorCompositePos               
#cc.IConnectorExtWrench                     
#cc.IConnectorPDCoupling                  
#cc.IConnectorPDCouplingList         
#cc.IConnectorRobotJointPDCoupling 
#cc.IConnectorRobotJointTorque      
#cc.IConnectorRobotState          
#cc.IConnectorSynchro              
#cc.IOConnectorInternalPDCoupling  

#conn = cc.OConnectorBodyState            
#conn = cc.OConnectorBodyStateList       
conn = cc.OConnectorCable                
#cc.OConnectorComposite            
#cc.OConnectorContactBody          
#cc.OConnectorContactComposite    
#cc.OConnectorDgmJointStateList 
#cc.OConnectorManikin           
#cc.OConnectorPDCoupling
cc.OConnectorRobotState
#cc.OConnectorSound 
#cc.OConnectorTimer

print ""
print ""

print conn.new.__doc__
conn.new("connector_name", "port_name", "cable_name")

getPort = phy.getPortNames

print getPort()



##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


