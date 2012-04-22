
Create robot with desc module
=============================

.. important::
   This example shows how to create a **robot** with the module :py:mod:`desc`.

   Source files:
   
   * :download:`main.py <02_robot_with_desc/main.py>`.
   * :download:`common.py <02_robot_with_desc/common.py>`.



Kinematic tree
--------------

The description of the robot is done through a tree-structure made of python tuples (or lists).
Each tuple represents a node which defines each segment of the robot.


.. literalinclude:: 02_robot_with_desc/main.py
   :start-after: ##### Create the kinematic tree
   :end-before: ##### Create a world with description of the kinematic tree



Robot description
-----------------

When the kinematic tree is set, it has to be *registered* in a **world**.
This world is composed of a physical scene (where the kinematic tree is *registered*),
but it also contains a graphical scene and a collision scene.

The following methods create a named mechanism
whose information about the kinematic and dynamic models can be obtain later,
for instance Jacobian matrices or generalized coordinates and velocity.
 

.. literalinclude:: 02_robot_with_desc/main.py
   :start-after: ##### Create a world with description of the kinematic tree
   :end-before: ##### Deserialize world





Deserialization
---------------

when the world building is complete, it has to be converted into elements in the GVM scene.
This is done with the :py:meth:`agents.physic.builder.deserializeWorld` method.

Note that the same goes for the graphical agent,
to convert elements that are registered in the graphical scene of the world.


.. literalinclude:: 02_robot_with_desc/main.py
   :start-after: ##### Deserialize world
   :end-before: ##### Connect physic and graphic agents




