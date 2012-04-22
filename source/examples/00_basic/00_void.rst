
Void script
===========

.. important::
   This example shows how to **load XDE libraries** and
   how to open an **interactive shell** at the end of the script.
   This give the possibility to edit the scene properties 
   (e.g. joints position & velocity) while simulation is running.

   source files:
   
   * :download:`main.py <00_void/main.py>`





Load XDE libraries
------------------

At the very beginning of any script, we need to import the XDE ``loader``
in order to be able to load XDE libraries. This is done as follows

.. literalinclude:: 00_void/main.py
   :start-after: ##### Preambule
   :end-before: ##### Simulation


Get an interactive shell
------------------------

At any moment of the script, an interactive shell can be launched to access or modify simulation data.
This is done as follows

.. literalinclude:: 00_void/main.py
   :start-after: ##### Interactive shell


Usually, this method is called at this end of the script, after all agents are running.


.. warning::
   The call of ``shell()`` is a blocking procedure, meaning that python
   interpreter waits for the interactive shell to terminate. The remaining
   code is executed after that termination, for instance when user
   calls ``exit()``.



