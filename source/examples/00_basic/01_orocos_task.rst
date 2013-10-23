Create an Orocos component
==========================

.. important::
   This example shows how to **write an orocos component in Python**.

   Source files:
   
   * :download:`orocos_task.py <01_orocos_task/orocos_task.py>`




The orocos task
---------------

First you have to import the modules of the Python interface for RTT:

.. literalinclude:: 01_orocos_task/orocos_task.py
   :start-after: ##### Import modules
   :end-before: ##### Orocos Task

Then write a class that derives from ``xdefw.rtt.Task``. In the constructor,
you can set the period of the component, add data ports...

.. literalinclude:: 01_orocos_task/orocos_task.py
   :start-after: ##### Orocos Task
   :end-before: ##### Implement hooks

Then, you have to implement the different hooks that will define the behavior of the component:

.. literalinclude:: 01_orocos_task/orocos_task.py
   :start-after: ##### Implement hooks

The operations of the orocos task can be accessed in Python in the field ``s``.
For example ``task.s.getPeriod()``
