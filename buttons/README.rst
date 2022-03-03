.. _buttons-example:

Buttons
#######

Overview
********

A simple buttons demo showcasing the use of GPIO input with interrupts.
The example prints a message to the console each time a button is pressed.

Building and Running
********************

This application can be built and executed on the Summit SOM 8M Plus DVK as
follows:


.. zephyr-app-commands::
   :zephyr-app: samples/buttons
   :host-os: unix
   :board: summitsom8mplus_dvk_itcm
   :goals: run
   :compact:

After startup, the program looks up the predefined GPIO devices and configures
the pins in input mode, enabling interrupt generation on falling edge. During
each iteration of the main loop, the state of GPIO line is monitored and printed
to the serial console. When an input button gets pressed, an interrupt handler
will print information about this event along with its timestamp.

Flashing and Booting the M7 Core
********************************
Below are the set of U-Boot commands to load and boot the M7 core. Currently,
two run-modes are supported: ITCM and DDR. These steps assume the name of the
binary to be ``zephyr.bin``.

Flash and Boot M7 Core in ITCM Run-mode
=======================================
.. code-block:: console

    fatload mmc 1:1 0x48000000 zephyr.bin
    cp.b 0x48000000 0x7e0000 20000
    bootaux 0x7e0000

Flash and Boot M7 Core in DDR Run-mode
======================================
.. code-block:: console

    fatload mmc 1:1 0x80000000 zephyr.bin
    dcache flush
    bootaux 0x80000000
