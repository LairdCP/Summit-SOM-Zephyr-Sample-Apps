.. _rgb-blinky-example:

Blinky
######

Overview
********

RGB Blinky is a simple application which cycles the RGB LED onboard the Summit
SOM 8M Plus DVK through a set of colors forever using the :ref:`GPIO API
<gpio_api>`. The source code shows how to configure GPIO pins as outputs, then
turn them on and off.

This application also demonstrates the use of the :ref:`I2C API <i2c_api>` as
the RGB is connected to a GPIO expander on the I2C bus (I2C3).

Building and Running
********************

This application can be built and executed on the Summit SOM 8M Plus DVK as
follows:


.. zephyr-app-commands::
   :zephyr-app: samples/rgb_blinky
   :host-os: unix
   :board: summitsom8mplus_dvk_itcm
   :goals: run
   :compact:

After flashing, the RGB LED starts to blink through a set of 7 colors.

NOTE
====

Care must be taken to ensure the I2C bus used in this demo (I2C3) is not
actively being controlled by the A cores of the system. This can be achieved by
specifying a proper devicetree file in U-Boot before booting into Linux or
simply by not booting the system past the U-Boot console.

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
