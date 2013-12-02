
Kinematic control
=================

.. important::
   This example show how to build a kinematic controller for a pendulum

.. note::
   Get source of this example:
   
   * :download:`common.py <06_kinematic_control/common.py>`.
   * :download:`main.py <06_kinematic_control/main.py>`.

The kinematic controller
------------------------

The controller is an Orocos component. In this example we use python to implement it.
In the initialisation, we create a multibody_model, that is updated to be representative 
of state of the simulated robot and used to compute when needed
jacobians for instance.
Then in the updateHook, the current joint positions/velocities are read from the robot state,
and those are used as desired positions/velocities.

.. literalinclude:: 06_kinematic_control/main.py
   :start-after: ##### The Controller
   :end-before:         # print "-------------------"

Association with the robot
--------------------------

The controller is then instanciated and associated to the robot p1.

.. literalinclude:: 06_kinematic_control/main.py
   :start-after: # Create controller associated with the robot p1
   :end-before: # In order to get the state of the robot, we need to create a ConnectorRobotState

In order to read the state of the robot, we need to create a special connector:
the OConnectorRobotState which will add ports to the physic agent.
The argument are ``Connectors.OConnectorRobotState.new(connector_name, prefix_port_name, robot_name)``.
The ``prefix_port_name`` will be use to prefix the name of the new ports.
The state of the associated robot be written in those ports. Connecting
input ports to those generated ports will allow us to read the state of the robot.

.. literalinclude:: 06_kinematic_control/main.py
   :start-after: controller.s.setPeriod(0.001)
   :end-before: # In order to send command to the robot, we need to create a RobotJointPDCoupling

A similar connector has to be created to send command to the robot:
the creation of a IConnectorRobotJointPDCoupling will add ports to the physic agent.
Those ports have to be plugged to some output ports where a command is written.
Typically, it is plugged to the output of a controller.

.. literalinclude:: 06_kinematic_control/main.py
   :start-after:        #                              "PortName"+"qdot"
   :end-before: ##### Create clock

Then we have to connect the controller and the ports added by the creation of the connectors:

.. literalinclude:: 06_kinematic_control/main.py
   :start-after: clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))
   :end-before: # Run agents

It has to be noted that the joint control have to be enable for the robot:

.. literalinclude:: 06_kinematic_control/main.py
   :start-after: phy.s.agent.triggerUpdate()
   :end-before: ##### Interactive shell
