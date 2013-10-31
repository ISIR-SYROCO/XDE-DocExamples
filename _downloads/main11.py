#-------------------------------------------------------------------------------
#
# Load libraries
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


    
#-------------------------------------------------------------------------------
#
# Create a new Task. This will be the Kinematic controller.
#
#-------------------------------------------------------------------------------
import rtt_interface
import xdefw.rtt
import physicshelper
import xde.desc.physic.physic_pb2

class KinematicController(xdefw.rtt.Task):
  
    def __init__(self, taskName, world, robotName):
        task = rtt_interface.PyTaskFactory.CreateTask(taskName)
        xdefw.rtt.Task.__init__(self, task)

        # model instance
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

        # create input ports
        self.q_in = self.addCreateInputPort("q", "VectorXd", True)
        self.q_ok = False
        self.qdot_in = self.addCreateInputPort("qdot", "VectorXd", True)
        self.qdot_ok = False
    
        # create output ports
        self.q_des_out = self.addCreateOutputPort("q_des", "VectorXd")
        self.qdot_des_out = self.addCreateOutputPort("qdot_des", "VectorXd")
        self.kp_des_out = self.addCreateOutputPort("kp_des", "VectorXd")
        self.kd_des_out = self.addCreateOutputPort("kd_des", "VectorXd")
        self.kp_des = 100
        self.kd_des = 20
  
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
        
        # define output vectors
        q_des_out = q
        qdot_des_out = lgsm.vector([.1] * (model.nbInternalDofs()))
        kp_des_out = lgsm.vector([self.kp_des] * model.nbInternalDofs())
        kd_des_out = lgsm.vector([self.kd_des] * model.nbInternalDofs())
        
        
        
        # send output vectors
        self.kp_des_out.write(kp_des_out)
        self.kd_des_out.write(kd_des_out)
        self.q_des_out.write(q_des_out)
        self.qdot_des_out.write(qdot_des_out)
        # print "-------------------"
        # print q
        # print qdot
        # print q_des_out
        # print qdot_des_out



#-------------------------------------------------------------------------------
#
# Setup agents & Create Connections between phy and graph
#
#-------------------------------------------------------------------------------
# create agents
phy, ms, lmd = common.get_physic_agent(time_step)                               # create physic agent
graph, gInterface = common.get_graphic_agent()                                  # create graphic agent
world = common.create_world_and_deserialized(phy, ms, lmd, graph, gInterface)   # create world and deserialize

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
# Create Kinematic controller & clock
#
#-------------------------------------------------------------------------------

# Create controller
controller = KinematicController("MyController", world, "p1") # ControllerName, the world instance, RobotName
controller.s.setPeriod(0.001)

phy.s.Connectors.OConnectorRobotState.new("ocpos", "p1_", "p1")         # ConnectorName, PortName, RobotName
                                                                        # It generates two ports named "PortName"+"q"
                                                                        #                              "PortName"+"qdot"
irj = phy.s.Connectors.IConnectorRobotJointPDCoupling.new("icrjPDc", "p1_", "p1")    # ConnectorName, PortName, RobotName
                                                                        # It generates 4 ports named "PortName"+"q_des"
                                                                        #                                      +"qdot_des"
                                                                        #                                      +"kp_des"
                                                                        #                                      +"kd_des"


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
icps.addEvent("p1_qdot_des")
icps.addEvent("clock_trigger")

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))



#-------------------------------------------------------------------------------
#
# Create connection between phy and controller for Kinematic control
#
#-------------------------------------------------------------------------------

phy.getPort('p1_q').connectTo(controller.getPort('q'))
phy.getPort('p1_qdot').connectTo(controller.getPort('qdot'))

controller.getPort("q_des").connectTo(phy.getPort("p1_q_des"))
controller.getPort("qdot_des").connectTo(phy.getPort("p1_qdot_des"))
controller.getPort("kp_des").connectTo(phy.getPort("p1_kp_des"))
controller.getPort("kd_des").connectTo(phy.getPort("p1_kd_des"))


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

p1 = phy.s.GVM.Robot("p1")
p1.enableAllJointPDCouplings(True)

##### Interactive shell
import xdefw.interactive
shell = xdefw.interactive.shell_console()
shell()


