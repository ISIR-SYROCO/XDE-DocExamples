##### Import modules
import rtt_interface
import xdefw.rtt

##### Orocos Task
class MyComponent(xdefw.rtt.Task):
    def __init__(self, name, time_step):
        super(MyComponent, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

        self.s.setPeriod(time_step)

##### Create ports
        # addCreateInputPort(name, type, eventPort)
        # eventPort is a boolean, if True, the input port becomes an event port
        # event port triggers updateHook() when data is received
        self.i_port = self.addCreateInputPort("i", "VectorXd", True)
        self.o_port = self.addCreateOutputPort("o", "VectorXd")

##### Implement hooks
    def startHook(self):
        print "Start!"

    def stopHook(self):
        print "Stop!"

    def configureHook(self):
        print "Configure!"

    def updateHook(self):
        print "Update!"

        # Read a port.
        # The function return a couple: (data, flag)
        # the flag is True if data has been properly read
        i, iok = self.i_port.read()
        if iok:
            print i
            # Write something
            self.o_port.write(i)

