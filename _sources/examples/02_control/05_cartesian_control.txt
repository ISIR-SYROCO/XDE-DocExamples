
Cartesian control
=================

.. important::
   This example show how to build a simple cartesian controller for a pendulum

.. note::
   Get source of this example:
   
   * :download:`common.py <05_cartesian_control/common.py>`.
   * :download:`main.py <05_cartesian_control/main.py>`.

The cartesian controller
------------------------

The controller is an Orocos component. In this example we use python to implement it.
In the initialisation, we create a multibody_model, that is updated to be representative 
of state of the simulated robot and used to compute when needed
jacobians for instance.
Then in the updateHook, the current joint positions/velocities are read from the robot state,
and those are used as desired positions/velocities.

.. literalinclude:: 05_cartesian_control/main.py
   :start-after: ##### Cartesian controller
   :end-before: ##### Create Controller, Clock

Association with the robot
--------------------------

The controller is then instanciated and associated to the robot p1.

.. literalinclude:: 05_cartesian_control/main.py
   :start-after: ##### Create Controller, Clock
   :end-before: # Creation of robot state connectors

In order to read the state of the robot, we need to create a special connector:
the OConnectorRobotState which will add ports to the physic agent.
The argument are ``Connectors.OConnectorRobotState.new(connector_name, prefix_port_name, robot_name)``.
The ``prefix_port_name`` will be use to prefix the name of the new ports.
The state of the associated robot be written in those ports. Connecting
input ports to those generated ports will allow us to read the state of the robot.

.. literalinclude:: 05_cartesian_control/main.py
   :start-after: # Creation of robot state connectors
   :end-before: # Creation of robot torque input

A similar connector has to be created to send command to the robot:
the creation of a IConnectorRobotJointTorque will add a port to the physic agent.
This port has to be plugged to an output port where a torque command is written.
Typically, it is plugged to the output of a controller.

.. literalinclude:: 05_cartesian_control/main.py
   :start-after: # Creation of robot torque input
   :end-before: ##### Create clock

Then we have to connect the controller and the ports added by the creation of the connectors:

.. literalinclude:: 05_cartesian_control/main.py
   :start-after: # Create connection between phy and controller for Cartesian control
   :end-before: # Run agents
