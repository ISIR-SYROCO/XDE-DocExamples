
Introduction
============

In this section we will briefly describe what is the Orocos Toolchain and its relation with the XDE framework.
Indeed, XDE framework is a set of Orocos components developped by the french CEA dedicated to interactive dynamic simulation.
We will then introduce how the XDE framework is organised.

Orocos Toolchain
----------------

Orocos Toolchain is a framework to build real time applications using a components approach. The toolchain contains:
  * RTT: Real Time Toolkit, which provides a framework to write C++ Orocos components
  * OCL: Orocos Component Library, which is a set of Orocos components to manage setup and interactions between components
  * Orogen: A code generation tool to quickly generate skeleton of component including compilation scripts

Orocos Component
~~~~~~~~~~~~~~~~

Basically, an Orocos component_ is a state machine with three states:
  * Pre-Operational
  * Stopped
  * Running

The transition are triggered by hook function. The component can work in a synchronous mode, where
the ``updateHook()`` is called periodically, and in an asynchronous mode where the ``updateHook()``
is triggered by events (data on ports for example).

.. _component:
.. figure:: introduction/component.png
   :align:  center

   An Orocos component.

We can interact with a component through:
  * ``Services`` or ``Operations`` which typically are functions like ``setPeriod(T)``
  * ``Data ports`` where the data are processed in the ``updateHook()``
  * ``Properties`` which are special parameters that can be set from a configuration file

OCL Deployer
~~~~~~~~~~~~

The Deployer is a special component that provides an interactive shell that enables us to manage other components. Basically, the Deployer allows us to:
  * Import a library containing a component
  * Load the component in the application environement
  * Connect ports of components
  * Start and stop components
  * Run scripts to perform those tasks

XDE and Orocos relationship
---------------------------

XDE framework is a set of Orocos components also called, in XDE applications, agents. The main ones are:
  * Physics
  * Graphics
  * Virtual human simulation
  * IO Devices

Python Interface
~~~~~~~~~~~~~~~~

XDE framework contains Python interface for RTT. As a consequence,
it is possible to write Orocos components in Python.

There is also a python interface for OCL Deployer. It allows us to import library (compiled from C++) that contains
Orocos components, to load components and to connect them in Python. Basically, in order to build a simulation,
we have to write a Python script that will be an equivalent of an OCL Deployer script, loading, configuring and connecting different components.

.. _xde_deploy:
.. figure:: introduction/xde_deploy.png
   :align:  center

   An example of a deployment in XDE.
