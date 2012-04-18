
Get physic agent connectors
===========================

.. note::
   Get source of this example  :download:`here <01_create_robot/main.py>`.

**Goal**: This example shows ...

Basic port
----------

* ``tick``: get tick of physical agent.


BodyState
---------

send position of Body with name ``body_name``.

:py:meth:`phy.s.OConnectorBodyState.new("connector_name", "port_name", "body_name")`

Ports:

* ``port_name``: Body position.


BodyStateList
-------------

send position of Body with name ``body_name``.

:py:meth:`phy.s.OConnectorBodyState.new("connector_name", "port_name", "body_name")`

Ports:

* ``port_name**_H**``: Bodies displacement.
* ``port_name**_T**``: Bodies twist.
