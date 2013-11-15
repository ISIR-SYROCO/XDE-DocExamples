
import rtt_interface
import xdefw.rtt
import physicshelper

import lgsm

import time

#-------------------------------------------------------------------------------
#
# Create a new Task. This will be the Kinematic controller.
#
#-------------------------------------------------------------------------------
class ContactController(xdefw.rtt.Task):
  
    def __init__(self, taskName, world, robotName):
        task = rtt_interface.PyTaskFactory.CreateTask(taskName)
        xdefw.rtt.Task.__init__(self, task)

        # model instance
        self.model = physicshelper.createDynamicModel(world, robotName)

        # create input ports
        self.c_in = self.addCreateInputPort("cont", "SMsg", True)
        self.c_ok = False
        self.c = None
        
        self.tau_out = self.addCreateOutputPort("tau", "VectorXd")


    def startHook(self):
        pass
  
    def stopHook(self):
        pass
  
    def updateHook(self):
        if not self.c_ok:
            self.c,self.c_ok = self.c_in.read()
        
        if self.c_ok:
            self.c_ok = False
            self.doUpdate(self.c)

  
    def doUpdate(self, c):
        model = self.model
        
        if len(c.cpt) == 0:
            print "no contact yet"
        else:
            # That what we get 
            print "Nb of contact: ", len(c.cpt)
            print "for contact 0: "
            cc = c.cpt[0]
            print "    ai :", cc.ai[:]
            print "    aj :", cc.aj[:]
            print "    gap:", cc.gap
            print "    ni :", cc.ni[:]
            print "    nj :", cc.nj[:]
            print "    normalForce:", cc.normalForce
            time.sleep(.1)

        tau = lgsm.zeros(model.nbInternalDofs())
        self.tau_out.write(tau)
