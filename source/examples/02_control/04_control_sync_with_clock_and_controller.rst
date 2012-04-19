
Waiting for the controller for non-real time simulations
========================================================

.. note::
   Get source of this example:
   
   * :download:`main.py <04_control_sync_with_clock_and_controller/main.py>`.



.. note::

   En fait, dès qu'il y a un input port dans l'agent physic, alors celui ci n'est plus temps réel mais est calé sur l'input (voire aussi avec un l'input synchro).

   S'il n'y a rien l'input est le temps d'intégration.

   ATTENTION: ce temps d'intégration reste le même dans les deux cas.

   donc si les input de l'agent physic sont plus rapide que le temps d'intégration la simulation a l'air accélérée.

   si les inputs sont plus lents, alors la simu a l'air ralenti.
