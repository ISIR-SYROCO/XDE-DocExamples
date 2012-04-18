
Void script
===========

.. note::
   Get source of this example  :download:`here <00_void/main.py>`.

**Goal**: The following code block shows how to load XDE libraries and
how to open an interactive shell at the end of the script, to edit scene
properties while simulation is running.


Load XDE libraries
------------------

At the very beginning of any script, we have to load XDE library, as follows

.. literalinclude:: 00_void/main.py
   :start-after: ##### Preambule
   :end-before: ##### Simulation


Get an interactive shell
------------------------

At the very end of the script, when all elements of the simulation have
been configured and are running (`start` services have been called),
an interactive shell can be launched to access or modify simulation data.
This is done as follows

.. literalinclude:: 00_void/main.py
   :start-after: ##### Interactive shell


.. warning::
   The call of ``shell()`` is a blocking procedure, meaning that python
   interpreter waits for the interactive shell to terminate. The remaining
   code is executed after that termination, for instance when user
   call ``exit()``.



