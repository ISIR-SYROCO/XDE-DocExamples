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
import dsimi.rtt
import physicshelper

class CartesianController(dsimi.rtt.Task):
  
    def __init__(self, taskName, world, robotName):
        task = rtt_interface.PyTaskFactory.CreateTask(taskName)
        dsimi.rtt.Task.__init__(self, task)

        self.model = physicshelper.createDynamicModel(world, robotName)

        self.q_in = self.addCreateInputPort("q", "VectorXd", True)
        self.q_ok = False
        
        self.qdot_in = self.addCreateInputPort("qdot", "VectorXd", True)
        self.qdot_ok = False
    
        self.tau_out = self.addCreateOutputPort("tau", "VectorXd")
        
        self.kp = 100
        self.kd = 20
  
    def startHook(self):
        pass
  
    def stopHook(self):
        pass
  
    def updateHook(self):
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
        H = lgsm.Displacement(lgsm.vectord(0,0,0), H6.getRotation())
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



#-------------------------------------------------------------------------------
#
# Create Controller, Clock
#
#-------------------------------------------------------------------------------

# Create controller
controller = CartesianController("MyController", world, "p1") # ControllerName, the world instance, RobotName
controller.s.setPeriod(0.001)

phy.s.Connectors.OConnectorRobotState.new("ocpos", "p1_", "p1")         # ConnectorName, PortName, RobotName
                                                                        # It generates two ports named "PortName"+"q" & "PortName"+"qdpt"
phy.s.Connectors.IConnectorRobotJointTorque.new("icjt", "p1_", "p1")    # ConnectorName, PortName, RobotName
                                                                        # It generates a port named "PortName"+"tau"


##### Create clock
import deploy.deployer as ddeployer
clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
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




#-------------------------------------------------------------------------------
#
# Create connection between phy and controller for Cartesian control
#
#-------------------------------------------------------------------------------

phy.getPort('p1_q').connectTo(controller.getPort('q'))
phy.getPort('p1_qdot').connectTo(controller.getPort('qdot'))

controller.getPort("tau").connectTo(phy.getPort("p1_tau"))


#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()
controller.s.start()
clock.s.start()

phy.s.agent.triggerUpdate()

##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


