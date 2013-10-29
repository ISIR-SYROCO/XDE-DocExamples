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

# Graphic
graph = agents.graphic.simple.createAgent("graphic", 0)                          # graphic scne
gInterface, Nscene, Nwin, Nview = agents.graphic.simple.setupSingleGLView(graph) # graphic interface
agents.graphic.proto.configureBasicLights(gInterface)
agents.graphic.proto.configureBasicCamera(gInterface)
graph.s.Viewer.enableNavigation(True)
gInterface.SceneryInterface.showGround(True)



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
    T_pc = lgsm.Displacement()
    T_pc.setTranslation(lgsm.vector(.5,0,0))
    j.configure(T_pc,                # Displacement from parent body to child body
                lgsm.vector(0,0,0),  # Hinge center position in parent frame
                lgsm.vector(0,1,0),  # Hinge axis in parent frame
                0.2)                 # Hinge angular offset
    j.setJointDampingCoeff(2) # add some damping
    j.enableJointDamping()
    joints.append(j)


##### Create connection between bodies with joints
ms.addRigidBodyToGround(bodies[0], joints[0])
for i in range(1, len(bodies)):
    ms.addRigidBody(bodies[i-1], bodies[i], joints[i])



phy.s.GVM.Robot.new("my_robot")




##### Connect physic and graphic agents to see bodies with markers

# Create output connector from physical scene
ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "bodyPosition")

# Create input connector from graphical scene
graph.s.Connectors.IConnectorFrame.new("icf", "framePosition", "mainScene")

# Connect the two
graph.getPort("framePosition").connectTo(phy.getPort("bodyPosition_H"))

# add markers on bodies
for n in phy.s.GVM.Scene("main").getBodyNames():
    ocb.addBody(n)
    gInterface.MarkersInterface.addMarker(n, False)



#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()


##### Interactive shell
import xdefw.interactive
shell = xdefw.interactive.shell_console()
shell()


