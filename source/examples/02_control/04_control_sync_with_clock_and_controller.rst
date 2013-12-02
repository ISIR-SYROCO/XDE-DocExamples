
Waiting for the controller for non-real time simulations
========================================================

.. note::
   Get source of this example:
   
   * :download:`common.py <04_control_sync_with_clock_and_controller/common.py>`.
   * :download:`main.py <04_control_sync_with_clock_and_controller/main.py>`.


.. important::
   This example shows how to write non-real time simulations. The computation
   of the physics is triggered by events, as opposed to the real time simulations
   where the computation of the physics is triggered by a clock.

   It is important to note that the integration step is not affected, but the rate of integration is.
   It means that if the events occur faster than the integration time, the simulation
   will look accelerated. If the events occur slower than the integration time,
   the simulation will appear to be in slow motion.

Synchronization of the physic agent
-----------------------------------

This example shows how to synchronize the physics with a controller:
as soon as the command is computed, the computation of the physic is triggered.

.. literalinclude:: 04_control_sync_with_clock_and_controller/main.py
   :start-after: ##### Create clock, to synchronize phy and controller
   :end-before: #-------------------------------------------------------------------------------

