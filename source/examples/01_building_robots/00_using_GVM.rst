
Building a robot from GVM module
================================

.. note::
   Get source of this example:
   
   * :download:`main.py <00_using_GVM/main.py>`.

**Goal**: This example shows a *low-level* method to create a pendulum using the GVM module.
It also shows how to connect physic and graphic agents with markers on bodies.
For now, we do not talk about *robot* or *mechanism*, we simple create joints and bodies.


Create rigid bodies and configure
---------------------------------

Roughly, a rigid body is defined by a name and a mass matrix.
The following snippet of code present how to create many bodies with a for loop.


.. literalinclude:: 00_using_GVM/main.py
   :start-after: ##### Create all rigid bodies and configure
   :end-before: ##### Create all joints and configure


All body instances are saved in the ``bodies`` list.
We recall that an other way to get back any body instance by its name is to use the follwoing method::
  
  the_body = phy.s.GVM.RigidBody.new("the name of the body")



Create joints and configure
---------------------------

A joint links two bodies in a kinematic tree.
There are many different joints in the GVM module, here it shows how to create a configure hinge joints.

.. literalinclude:: 00_using_GVM/main.py
   :start-after: ##### Create all joints and configure
   :end-before: ##### Create connection between bodies with joints

.. The arguments of the ``configure`` method can be explained in the following picture.
   There are 4 arguments which are respectively, a ``Dispacement`` instance  `H_{pc}`, two ``vector`` instances `(V_{pj}, V_{axis})`, and a float `\theta`.



Connect bodies with joints
--------------------------

The GVM module creates the physical elements, but they have to be linked in the **main scene** to create the kinematic tree.
The connection are made with the methods ``addRigidBodyToGround`` and ``addRigidBody``.

.. literalinclude:: 00_using_GVM/main.py
   :start-after: ##### Create connection between bodies with joints
   :end-before: ##### Connect physic and graphic agents to see bodies with markers



Add markers to bodies in graphic interface
------------------------------------------

The previous steps has shown the building of the pendulum in the physic scene, and this can be sufficient in some cases.
However, the graphical scene has no information about evolutions in the physic scene, so we cannot display the body position during time.

To do this, we use **connectors** to create a pipe between the scenes.
The physic scene has an outupt connector to send data about bodies positions, and the graphic scene has an input connector to receive data about bodies positions.

.. literalinclude:: 00_using_GVM/main.py
   :start-after: ##### Connect physic and graphic agents to see bodies with markers
   :end-before: #-------------------------------------------------------------------------------



