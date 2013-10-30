
Export a robot with desc module
===============================

.. important::
   This example shows how to export/import a **robot** with the module :py:mod:`desc`.

   Source files:
   
   * :download:`main.py <03_export_robot/main.py>`.
   * :download:`common.py <03_export_robot/common.py>`.



Kinematic tree
--------------

The description of the robot is done through a tree-structure made of python tuples (or lists).
Each tuple represents a node which defines each segment of the robot.


.. literalinclude:: 03_export_robot/main.py
   :start-after: ##### Create the kinematic tree
   :end-before: ##### Create a description of the robot



Export robot
------------

Once the description of the robot is done, we can export it into a binary file.
This can be useful if you always work with the same robot.


.. literalinclude:: 03_export_robot/main.py
   :start-after: ##### We write the description of the robot in a binary file
   :end-before: ##### Import and deserialize world: register world description in phy & graph agents


Import and deserialization
--------------------------

The binary file can then be imported:

.. literalinclude:: 03_export_robot/main.py
   :start-after: ##### Import and deserialize world: register world description in phy & graph agents
   :end-before: ##### Connect physic and graphic agents




