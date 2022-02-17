.. _low_power_audio:

Low Power Audio Example
##############################

Overview
********

In this demo, the A core(s) decodes music data, puts it into DDR buffer, and
informs the M7 core with the related information. Then, the M7 core will take
ownership of consuming the buffer. It will copy buffer data from DDR to TCM,
manipulating SDMA to transfer the data to SAI and codec for playback. It gives
DDR and the A core(s) opportunity to enter power saving mode for a rather long
time frame. The M7 core will also take ownership of codec initialization.

SRTM (Simplified Real Time Messaging) protocol is used to communicate between
the A core(s) and the M7 core. The protocol provides various commands for the A
core(s) and the M7 core to communicate with each other. If there is no audio
playback, the M7 core will enter the STOP mode, and the whole SOC system enters
deep sleep mode (DSM) once the A core(s) enter low power status.

NOTE: This example does not support running the M7 core from DDR.

Building and Running
********************

This application can be built and executed on the Summit SOM 8M Plus
DVK as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/low_power_audio
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

NOTES
=====
1.  16/24/32 bits for PCM audio streams are supported.
2.  Since audio files are typically large, it can be useful to create a new,
    large-size partition on the SD card to store the audio files.
3.  After the M7 core is running, please boot the Linux kernel to create the
    rpmsg channel between the A cores and the M7 core. Make sure the FDT file is
    correctly set before booting the Linux kernel. The following commands can be
    used to set the FDT file in the U-Boot console:

    .. code-block:: console

        u-boot=>setenv fdtfile <target-fdtfile.dtb>
        u-boot=>saveenv

4.  Please make sure that a target .wav file exists on the SD card. If the audio
    file is stored on the Windows FAT32 paritition, after the Linux kernel
    boots, and you've logged in as ``root``, mount the FAT32 partition using the
    "``mount /dev/mmcblk1p1 /mnt``" command and then go to the ``/mnt`` folder to
    playback the audio file using the playback command. If the audio file is
    stored on the Linux partition (e.g., ``/home``), you can playback the audio
    file directly using the playback command.

Playback Command
****************
1.  Use the command "``cat /proc/asound/cards``" to check the wm8960 sound card
    number. For example:

    .. code-block:: console

        # cat /proc/asound/cards
        0 [imxaudioxcvr   ]: imx-audio-xcvr - imx-audio-xcvr
                        imx-audio-xcvr
        1 [wm8960audio    ]: wm8960-audio - wm8960-audio
                        wm8960-audio

    Here, the wm8960 sound card number is 1.

Running the Demo
****************
After the boot process succeeds, the ARM Cortex-M7 terminal displays a message
similar to the following:

.. code-block:: console

    *** Booting Zephyr OS build v3.0.0-rc1-239-g11cbaf46deae  ***

    ####################  LOW POWER AUDIO TASK ####################

        Build Time: Feb 15 2022--16:50:38 
    ********************************
    Wait for Linux kernel to boot up and create the link between M core and A core.

    ********************************
    The rpmsg channel created between M core and A core!
    ********************************


    Main thread is now running.

Flashing and Booting the M7 Core
********************************
Below are the set of U-Boot commands to load and boot the M7 core. Currently,
one run-mode is supported: ITCM. These steps assume the name of the binary to be
``zephyr.bin``.

Flash and Boot M7 Core
=======================================
.. code-block:: console

    fatload mmc 1:1 0x48000000 zephyr.bin
    cp.b 0x48000000 0x7e0000 20000
    bootaux 0x7e0000
