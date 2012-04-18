
Create a physical agent
=======================

.. note::
   Get sources of this example  :download:`here <01_physic_agent/main_orocos.py>` & :download:`here <01_physic_agent/main_simple.py>`.

**Goal**: This example shows how to instantiate a physic agent, and what
are these parameters. There are many ways to create and configure this agent,
as explain in the following sections.


The orocos task
---------------

Create
~~~~~~

The *low-level* technique is to call
the method :py:meth:`deploy.deployer.rtt.Task`

.. literalinclude:: 01_physic_agent/main_orocos.py
   :start-after: ##### Create physic agent: orocos task
   :end-before: ##### Configure main scene

The physic agent gives access to many services, for example it can create
the **physical scene** by calling :py:meth:`phy.s.GVM.Scene.new`. This scene
roughly contains the links and bodies.


It also gives access to the creation of a **collision scene**, by calling
:py:meth:`phy.s.XCD.Scene.new_LMD`.



Configure
~~~~~~~~~

The scenes have to be configured and linked together to work properly.

.. literalinclude:: 01_physic_agent/main_orocos.py
   :start-after: ##### Configure main scene
   :end-before: ##### Run agent


Start & Stop
~~~~~~~~~~~~

The agent starts and stops when one calls thes functions::
   
   phy.s.start()
   
   phy.s.stop()




The agents module
-----------------

The other way is to use the :py:mod:`agents.physic.simple`

.. literalinclude:: 01_physic_agent/main_simple.py
   :start-after: ##### Create physic agent: agent module
   :end-before: ##### Run agent





