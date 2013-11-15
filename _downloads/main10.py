#-------------------------------------------------------------------------------
#
# Load libraries and setup agents
#
#-------------------------------------------------------------------------------
import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

import common

import lgsm

import time

time_step = 0.01

phy, ms, lmd = common.get_physic_agent(time_step)

graph, gInterface = common.get_graphic_agent()

world = common.create_world_and_deserialized(phy, ms, lmd, graph, gInterface)


#-------------------------------------------------------------------------------
#
# Create Connections between phy and graph
#
#-------------------------------------------------------------------------------
##### Connect physic and graphic agents to see bodies with markers
ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "bodyPosition")
graph.s.Connectors.IConnectorFrame.new("icf", "framePosition", "mainScene")
graph.getPort("framePosition").connectTo(phy.getPort("bodyPosition_H"))

# add markers on bodies
for n in phy.s.GVM.Scene("main").getBodyNames():
    ocb.addBody(n)
    gInterface.MarkersInterface.addMarker(n, False)



#-------------------------------------------------------------------------------
#
# Create a new Task. This will be the controller.
#
#-------------------------------------------------------------------------------

import rtt_interface
import xdefw.rtt
import physicshelper
import xde.desc.physic.physic_pb2

##### Cartesian controller
class CartesianController(xdefw.rtt.Task):
  
    def __init__(self, taskName, world, robotName):
        task = rtt_interface.PyTaskFactory.CreateTask(taskName)
        xdefw.rtt.Task.__init__(self, task)

        multiBodyModel = xde.desc.physic.physic_pb2.MultiBodyModel()
        mechanism_index = 0
        for m in world.scene.physical_scene.mechanisms:
            if robotName == m.name:
                break
            else:
                mechanism_index = mechanism_index + 1

        multiBodyModel.kinematic_tree.CopyFrom(world.scene.physical_scene.nodes[ mechanism_index ])
        multiBodyModel.meshes.extend(world.library.meshes)
        multiBodyModel.mechanism.CopyFrom(world.scene.physical_scene.mechanisms[ mechanism_index ])
        multiBodyModel.composites.extend(world.scene.physical_scene.collision_scene.meshes)
        self.model = physicshelper.createDynamicModel(multiBodyModel)

        # Input port to read the current state of the robot
        self.q_in = self.addCreateInputPort("q", "VectorXd", True)
        self.q_ok = False
        
        # Input port to read the current state of the robot
        self.qdot_in = self.addCreateInputPort("qdot", "VectorXd", True)
        self.qdot_ok = False
    
        # Output port to write torque commands
        self.tau_out = self.addCreateOutputPort("tau", "VectorXd")
        
        self.kp = 100
        self.kd = 20
  
    def startHook(self):
        pass
  
    def stopHook(self):
        pass
  
    def updateHook(self):
        # Try to read new value of robot state
        if not self.q_ok:
            self.q,self.q_ok = self.q_in.read()
        if not self.qdot_ok:
            self.qdot,self.qdot_ok = self.qdot_in.read()
        
        if self.q_ok and self.qdot_ok:
            self.q_ok = False
            self.qdot_ok = False
            self.doUpdate(self.q, self.qdot)

  
    def doUpdate(self, q, qdot):
        model = self.model
        model.setJointPositions(q)
        model.setJointVelocities(qdot)
        
        H6 = model.getSegmentPosition(model.getSegmentIndex("p1_b_3"))
        H = lgsm.Displacement()
        H.setTranslation(lgsm.vectord(0,0,0))
        H.setRotation(H6.getRotation())
        J66 = model.getSegmentJacobian(model.getSegmentIndex("p1_b_3"))
        J60 = H.adjoint() * J66
        T60 = H.adjoint() * model.getSegmentVelocity(model.getSegmentIndex("p1_b_3"))
    
        Xdes = lgsm.Displacement(.3,.3,.3,1,0,0,0) 
         
        xd = Xdes.getTranslation() # desired position of the effector
        x  = H6.getTranslation()
        v = T60.getLinearVelocity()
        fc = self.kp * (xd - x) - self.kd * v
    
        #tau = lgsm.vector([0] * model.nbInternalDofs())
        tau = J60[3:6,:].transpose() * fc
        
        tau += model.getGravityTerms()
        
        self.tau_out.write(tau)



##### Create Controller, Clock

# Create controller
controller = CartesianController("MyController", world, "p1") # ControllerName, the world instance, RobotName
controller.s.setPeriod(0.001)

# Creation of robot state connectors
phy.s.Connectors.OConnectorRobotState.new("ocpos", "p1_", "p1")         # ConnectorName, PortName, RobotName
                                                                        # It generates two ports named "PortName"+"q" & "PortName"+"qdpt"

# Creation of robot torque input
phy.s.Connectors.IConnectorRobotJointTorque.new("icjt", "p1_", "p1")    # ConnectorName, PortName, RobotName
                                                                        # It generates a port named "PortName"+"tau"


##### Create clock
import deploy.deployer as ddeployer
clock = xdefw.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", ""))
clock.s.setPeriod(time_step) #clock period == phy period


#-------------------------------------------------------------------------------
#
# Create input ports in physic agent for synchronization
#
#-------------------------------------------------------------------------------

phy.addCreateInputPort("clock_trigger", "double")

icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("p1_tau")
icps.addEvent("clock_trigger")

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))




# Create connection between phy and controller for Cartesian control
phy.getPort('p1_q').connectTo(controller.getPort('q'))
phy.getPort('p1_qdot').connectTo(controller.getPort('qdot'))

controller.getPort("tau").connectTo(phy.getPort("p1_tau"))


# Run agents
phy.s.start()
graph.s.start()
controller.s.start()
clock.s.start()

# Bootstrap the update of the physic agent so that the controller can
# do a first iteration
phy.s.agent.triggerUpdate()

##### Interactive shell
import xdefw.interactive
shell = xdefw.interactive.shell_console()
shell()


