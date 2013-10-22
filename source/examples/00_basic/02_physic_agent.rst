
Create a physical agent
=======================

.. important::
   This example shows how to **instantiate a physic agent**, and what are these parameters.
   There are many ways to create and configure this agent, as explain in the following sections.

   Source files:
   
   * :download:`main_orocos.py <02_physic_agent/main_orocos.py>`
   * :download:`main_simple.py <02_physic_agent/main_simple.py>`.




The orocos task
---------------

Create
~~~~~~

The *low-level* technique is to call the method :py:meth:`deploy.deployer.rtt.Task`

.. literalinclude:: 02_physic_agent/main_orocos.py
   :start-after: ##### Create physic agent: orocos task
   :end-before: ##### Configure main scene

The physic agent gives access to many services, for example it can create
the **physical scene** by calling :py:meth:`phy.s.GVM.Scene.new`.
This scene roughly contains the links and bodies.


It also gives access to the creation of a **collision scene**, by calling
:py:meth:`phy.s.XCD.Scene.new_LMD`.



Configure
~~~~~~~~~

The scenes (physical and collision) have to be configured and linked together to work properly.

.. literalinclude:: 02_physic_agent/main_orocos.py
   :start-after: ##### Configure main scene
   :end-before: ##### Run agent


Start & Stop
~~~~~~~~~~~~

The physical agent starts and stops when these functions are called::
   
   phy.s.start()
   
   phy.s.stop()




The agents module
-----------------

The other way is to use the module :py:mod:`agents.physic.simple`

.. literalinclude:: 02_physic_agent/main_simple.py
   :start-after: ##### Create physic agent: agent module
   :end-before: ##### Run agent





