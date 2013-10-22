
Create a graphic agent
======================

.. important::
   This example shows how to **instantiate a graphic agent**, and what are these parameters.
   There are many ways to create and configure this agent, as explain in the following sections.

   source files:
   
   * :download:`main_orocos.py <03_graphic_agent/main_orocos.py>`
   * :download:`main_simple.py <03_graphic_agent/main_simple.py>`




The orocos task
---------------

Create
~~~~~~

The *low-level* technique is to call the method :py:meth:`deploy.deployer.rtt.Task`

.. literalinclude:: 03_graphic_agent/main_orocos.py
   :start-after: ##### Create graphic agent: orocos task
   :end-before: ##### Configure graphic agent




Configure
~~~~~~~~~

The viewer has to load plugins and Ogre resources.

.. literalinclude:: 03_graphic_agent/main_orocos.py
   :start-after: ##### Configure graphic agent
   :end-before: ##### Open viewer


Open viewer
~~~~~~~~~~~

The viewer is initialize as follows

.. literalinclude:: 03_graphic_agent/main_orocos.py
   :start-after: ##### Open viewer
   :end-before: ##### Configure viewer



Configure viewer
~~~~~~~~~~~~~~~~

The viewer is configured as follows

.. literalinclude:: 03_graphic_agent/main_orocos.py
   :start-after: ##### Configure viewer
   :end-before: ##### Run agent



Start & Stop
~~~~~~~~~~~~

The agent starts and stops when one calls these functions::
   
   graph.s.start()
   
   graph.s.stop()


.. warning::
   The call :py:meth:`graph.s.stop` does not stop the viewer window.
   For now, I don't know how to close it and how to have many viewers at the same time.



The agents module
-----------------

The previous method is a little tricky.
The other way is to use the module :py:mod:`agents.graphic.simple`.
This creates a graphical agent and a viewer by setting few parameters.

Create
~~~~~~

The creation of the graphic agent is done as follows

.. literalinclude:: 03_graphic_agent/main_simple.py
   :start-after: ##### Create graphic agent: agent module
   :end-before: ##### Open viewer


Open viewer
~~~~~~~~~~~

The viewer is initialize as follows

.. literalinclude:: 03_graphic_agent/main_simple.py
   :start-after: ##### Open viewer
   :end-before: ##### Configure viewer


Configure viewer
~~~~~~~~~~~~~~~~

The viewer is configured as follows

.. literalinclude:: 03_graphic_agent/main_simple.py
   :start-after: ##### Configure viewer
   :end-before: ##### Run agent


