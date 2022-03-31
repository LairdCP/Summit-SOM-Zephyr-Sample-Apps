.. _rpmsg_lite_str_echo:

RPMsg-Lite String Echo Example
##############################

Overview
********

The Multicore RPMsg-Lite string echo sample is a simple demonstration program
that uses the Zephyr IPM driver and the RPMsg-Lite library and shows how to
implement inter-core communicaton between cores of the multicore system.

It works with Linux RPMsg master peer to transfer string content back and forth.
The name service handshake is performed first to create the communication
channels. Next, the Linux OS waits for user input to the RPMsg virtual tty.
Anything which is received is sent back to the M7. The M7 displays what is
received, and echoes back the same message as an acknowledgement. The tty reader
on the Linux side displays the message and can start another transaction. The
sample demonstrates RPMsg's ability to send arbitrary content back and forth.
Note: The maximum message length supported by RPMsg is currently 256 bytes.
Strings longer than 256 bytes will be divided by virtual tty into several
messages.

Building and Running
********************

This application can be built and executed on the Summit SOM 8M Plus
DVK as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/rpmsg_lite_str_echo_zephyr
   :host-os: unix
   :board: summitsom8mplus_dvk_itcm
   :goals: run
   :compact:

Prepare the Demo
****************
1.  Copy the sample binary to the ``boot`` directory of the microSD card and
    insert it into the target board.
2.  Connect a USB cable between the host PC and the J2 USB port on the target
    board.
3.  Connect 12V power supply and switch S9 to power on the board.
4.  Open two serial terminals for A53 core and M7 core with the following
    settings:

    * 115200 baud rate
    * 8 data bits
    * No parity
    * One stop bit
    * No flow control
5.  From the U-Boot prompt, load the sample binary into the M7 core and boot it
    by following the steps in `Flashing and Booting the M7 Core`_.
6.  After booting the M7 core, using the ``boot`` command to boot the kernel on
    the A core terminal
7.  After the kernel has booted, login.
8.  After login, make sure the ``imx_rpmsg_tty`` kernel module is inserted
    (``lsmod``), and if not, insert it (``modprobe imx_rpmsg_tty``).

Running the demo
****************
After the boot process succeeds, the ARM Cortex-M7 terminal displays the
following information:

.. code-block:: console

    RPMSG String Echo Zerphyr RTOS API Demo...

    Nameservice sent, ready for incoming messages...

After the Linux RPMsg tty module was installed, the ARM Cortex-M7 terminal
displays the following information:

.. code-block:: console

    Get Messgae From Master Side : "hello world!" [len : 12]

The user can then input an arbitrary string to the virtual RPMsg tty using the
following echo command on Cortex-A terminal:

.. code-block:: bash

    echo test > /dev/ttyRPMSG30

On the M7 terminal, the received string content and its length is output, as
shown in the log.

.. code-block:: console

    Get Message From Master Side : "test" [len : 4]
    Get New Line From Master side

Flashing and Booting the M7 Core
********************************
Below are the set of U-Boot commands to load and boot the M7 core. Currently,
three run-modes are supported: ITCM, DDR, and QSPI. These steps assume the name
of the binary to be ``zephyr.bin``.

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

Flash and Boot M7 Core in QSPI Run-mode
=======================================
To initially load the binary into the QSPI NOR flash:
.. code-block:: console

    fatload mmc 1:1 0x48000000 zephyr.bin
    dcache flush
    sf probe
    sf erase 0 0x100000
    sf write 0x48000000 0 0x100000

To boot the M7 core from the QSPI NOR flash:
.. code-block:: console

    dcache flush
    sf probe
    sf read 0x48000000 0 0x100000
    bootaux 0x8000000
