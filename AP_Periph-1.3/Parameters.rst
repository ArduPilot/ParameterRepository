.. Dynamically generated list of documented parameters
.. This page was generated using Tools\/autotest\/param\_metadata\/param\_parse\.py

.. DO NOT EDIT


.. _parameters:

Complete Parameter List
=======================

This is a complete list of the parameters which can be set \(e\.g\. via the MAVLink protocol\) to control vehicle behaviour\. They are stored in persistent storage on the vehicle\.

This list is automatically generated from the latest ardupilot source code\, and so may contain parameters which are not yet in the stable released versions of the code\.




.. _parameters_AP_Periph:

AP\_Periph Parameters
---------------------


.. _FORMAT_VERSION:

FORMAT\_VERSION: Eeprom format version number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This value is incremented when changes are made to the eeprom format


.. _CAN_NODE:

CAN\_NODE: UAVCAN node that is used for this network
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

UAVCAN node should be set implicitly or 0 for dynamic node allocation


+---------+
| Range   |
+=========+
| 0 - 250 |
+---------+




.. _CAN_BAUDRATE:

CAN\_BAUDRATE: Bitrate of CAN interface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Bit rate can be set up to from 10000 to 1000000


+-----------------+
| Range           |
+=================+
| 10000 - 1000000 |
+-----------------+




.. _CAN_PROTOCOL:

CAN\_PROTOCOL: Enable use of specific protocol to be used on this port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enabling this option starts selected protocol that will use this virtual driver\. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Disabled   | |
| +-------+------------+ |
| | 1     | UAVCAN     | |
| +-------+------------+ |
| | 3     | ToshibaCAN | |
| +-------+------------+ |
| | 4     | PiccoloCAN | |
| +-------+------------+ |
| | 5     | CANTester  | |
| +-------+------------+ |
| | 6     | EFI_NWPMU  | |
| +-------+------------+ |
| | 7     | USD1       | |
| +-------+------------+ |
| | 8     | KDECAN     | |
| +-------+------------+ |
|                        |
+------------------------+




.. _CAN2_BAUDRATE:

CAN2\_BAUDRATE: Bitrate of CAN2 interface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Bit rate can be set up to from 10000 to 1000000


+-----------------+
| Range           |
+=================+
| 10000 - 1000000 |
+-----------------+




.. _CAN2_PROTOCOL:

CAN2\_PROTOCOL: Enable use of specific protocol to be used on this port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enabling this option starts selected protocol that will use this virtual driver\. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Disabled   | |
| +-------+------------+ |
| | 1     | UAVCAN     | |
| +-------+------------+ |
| | 3     | ToshibaCAN | |
| +-------+------------+ |
| | 4     | PiccoloCAN | |
| +-------+------------+ |
| | 5     | CANTester  | |
| +-------+------------+ |
| | 6     | EFI_NWPMU  | |
| +-------+------------+ |
| | 7     | USD1       | |
| +-------+------------+ |
| | 8     | KDECAN     | |
| +-------+------------+ |
|                        |
+------------------------+




.. _CAN3_BAUDRATE:

CAN3\_BAUDRATE: Bitrate of CAN3 interface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Bit rate can be set up to from 10000 to 1000000


+-----------------+
| Range           |
+=================+
| 10000 - 1000000 |
+-----------------+




.. _CAN3_PROTOCOL:

CAN3\_PROTOCOL: Enable use of specific protocol to be used on this port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enabling this option starts selected protocol that will use this virtual driver\. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Disabled   | |
| +-------+------------+ |
| | 1     | UAVCAN     | |
| +-------+------------+ |
| | 3     | ToshibaCAN | |
| +-------+------------+ |
| | 4     | PiccoloCAN | |
| +-------+------------+ |
| | 5     | CANTester  | |
| +-------+------------+ |
| | 6     | EFI_NWPMU  | |
| +-------+------------+ |
| | 7     | USD1       | |
| +-------+------------+ |
| | 8     | KDECAN     | |
| +-------+------------+ |
|                        |
+------------------------+




.. _FLASH_BOOTLOADER:

FLASH\_BOOTLOADER: Trigger bootloader update
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DANGER\! When enabled\, the App will perform a bootloader update by copying the embedded bootloader over the existing bootloader\. This may take a few seconds to perform and should only be done if you know what you\'re doing\.


+-------+
| Range |
+=======+
| 0 - 1 |
+-------+




.. _DEBUG:

DEBUG: Debug
~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Debug


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | 0     | Disabled              | |
| +-------+-----------------------+ |
| | 1     | Show free stack space | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _BRD_SERIAL_NUM:

BRD\_SERIAL\_NUM: Serial number of device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Non\-zero positive values will be shown on the CAN App Name string


+----------------+
| Range          |
+================+
| 0 - 2147483648 |
+----------------+




.. _BUZZER_VOLUME:

BUZZER\_VOLUME: Buzzer volume
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Control the volume of the buzzer


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 100 | percent |
+-----------+---------+---------+




.. _GPS_PORT:

GPS\_PORT: GPS Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This is the serial port number where SERIALx\_PROTOCOL will be set to GPS\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 1         | 0 - 10 |
+-----------+--------+




.. _MB_CAN_PORT:

MB\_CAN\_PORT: Moving Baseline CAN Port option
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Autoselect dedicated CAN port on which moving baseline data will be transmitted\.


+------------------------------------------------------------------------------+
| Values                                                                       |
+==============================================================================+
| +-------+------------------------------------------------------------------+ |
| | Value | Meaning                                                          | |
| +=======+==================================================================+ |
| | 0     | Sends moving baseline data on all ports                          | |
| +-------+------------------------------------------------------------------+ |
| | 1     | auto select remaining port for transmitting Moving baseline Data | |
| +-------+------------------------------------------------------------------+ |
|                                                                              |
+------------------------------------------------------------------------------+




.. _BARO_ENABLE:

BARO\_ENABLE: Barometer Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Barometer Enable


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _LED_BRIGHTNESS:

LED\_BRIGHTNESS: LED Brightness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Select the RGB LED brightness level\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 100 | percent |
+-----------+---------+---------+




.. _RNGFND_BAUDRATE:

RNGFND\_BAUDRATE: Rangefinder serial baudrate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Rangefinder serial baudrate\.


+-----------+---------------------+
| Increment | Values              |
+===========+=====================+
| 1         | +-------+---------+ |
|           | | Value | Meaning | |
|           | +=======+=========+ |
|           | | 1     | 1200    | |
|           | +-------+---------+ |
|           | | 2     | 2400    | |
|           | +-------+---------+ |
|           | | 4     | 4800    | |
|           | +-------+---------+ |
|           | | 9     | 9600    | |
|           | +-------+---------+ |
|           | | 19    | 19200   | |
|           | +-------+---------+ |
|           | | 38    | 38400   | |
|           | +-------+---------+ |
|           | | 57    | 57600   | |
|           | +-------+---------+ |
|           | | 111   | 111100  | |
|           | +-------+---------+ |
|           | | 115   | 115200  | |
|           | +-------+---------+ |
|           | | 230   | 230400  | |
|           | +-------+---------+ |
|           | | 256   | 256000  | |
|           | +-------+---------+ |
|           | | 460   | 460800  | |
|           | +-------+---------+ |
|           | | 500   | 500000  | |
|           | +-------+---------+ |
|           | | 921   | 921600  | |
|           | +-------+---------+ |
|           | | 1500  | 1500000 | |
|           | +-------+---------+ |
|           |                     |
+-----------+---------------------+




.. _RNGFND_PORT:

RNGFND\_PORT: Rangefinder Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This is the serial port number where SERIALx\_PROTOCOL will be set to Rangefinder\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 1         | 0 - 10 |
+-----------+--------+




.. _ADSB_BAUDRATE:

ADSB\_BAUDRATE: ADSB serial baudrate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

ADSB serial baudrate\.


+-----------+---------------------+
| Increment | Values              |
+===========+=====================+
| 1         | +-------+---------+ |
|           | | Value | Meaning | |
|           | +=======+=========+ |
|           | | 1     | 1200    | |
|           | +-------+---------+ |
|           | | 2     | 2400    | |
|           | +-------+---------+ |
|           | | 4     | 4800    | |
|           | +-------+---------+ |
|           | | 9     | 9600    | |
|           | +-------+---------+ |
|           | | 19    | 19200   | |
|           | +-------+---------+ |
|           | | 38    | 38400   | |
|           | +-------+---------+ |
|           | | 57    | 57600   | |
|           | +-------+---------+ |
|           | | 111   | 111100  | |
|           | +-------+---------+ |
|           | | 115   | 115200  | |
|           | +-------+---------+ |
|           | | 230   | 230400  | |
|           | +-------+---------+ |
|           | | 256   | 256000  | |
|           | +-------+---------+ |
|           | | 460   | 460800  | |
|           | +-------+---------+ |
|           | | 500   | 500000  | |
|           | +-------+---------+ |
|           | | 921   | 921600  | |
|           | +-------+---------+ |
|           | | 1500  | 1500000 | |
|           | +-------+---------+ |
|           |                     |
+-----------+---------------------+




.. _ADSB_PORT:

ADSB\_PORT: ADSB Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This is the serial port number where SERIALx\_PROTOCOL will be set to ADSB\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 1         | 0 - 10 |
+-----------+--------+




.. _HARDPOINT_ID:

HARDPOINT\_ID: Hardpoint ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Hardpoint ID


.. _HARDPOINT_RATE:

HARDPOINT\_RATE: Hardpoint PWM rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Hardpoint PWM rate


+-----------+----------+-------+
| Increment | Range    | Units |
+===========+==========+=======+
| 1         | 10 - 100 | hertz |
+-----------+----------+-------+




.. _ESC_NUMBER:

ESC\_NUMBER: ESC number
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ESC number to report as in UAVCAN ESC telemetry feedback packets\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _ESC_PWM_TYPE:

ESC\_PWM\_TYPE: Output PWM type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This selects the output PWM type\, allowing for normal PWM continuous output\, OneShot\, brushed or DShot motor output


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 1     | Normal     | |
| +-------+------------+ |
| | 2     | OneShot    | |
| +-------+------------+ |
| | 3     | OneShot125 | |
| +-------+------------+ |
| | 4     | Brushed    | |
| +-------+------------+ |
| | 5     | DShot150   | |
| +-------+------------+ |
| | 6     | DShot300   | |
| +-------+------------+ |
| | 7     | DShot600   | |
| +-------+------------+ |
| | 8     | DShot1200  | |
| +-------+------------+ |
|                        |
+------------------------+




.. _ESC_TELEM_PORT:

ESC\_TELEM\_PORT: ESC Telemetry Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This is the serial port number where SERIALx\_PROTOCOL will be set to ESC Telemetry


+-----------+--------+
| Increment | Range  |
+===========+========+
| 1         | 0 - 10 |
+-----------+--------+




.. _MSP_PORT:

MSP\_PORT: MSP Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This is the serial port number where SERIALx\_PROTOCOL will be set to MSP


+-----------+--------+
| Increment | Range  |
+===========+========+
| 1         | 0 - 10 |
+-----------+--------+




.. _LOG_BITMASK:

LOG\_BITMASK: Log bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~


4 byte bitmap of log types to enable


+-------------------+
| Bitmask           |
+===================+
| +-----+---------+ |
| | Bit | Meaning | |
| +=====+=========+ |
| | 2   | GPS     | |
| +-----+---------+ |
|                   |
+-------------------+




.. _SYSID_THISMAV:

SYSID\_THISMAV: MAVLink system ID of this vehicle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Allows setting an individual system id for this vehicle to distinguish it from others on the same network


+---------+
| Range   |
+=========+
| 1 - 255 |
+---------+





.. _parameters_ARSP:

ARSP Parameters
---------------


.. _ARSP_TYPE:

ARSP\_TYPE: Airspeed type
~~~~~~~~~~~~~~~~~~~~~~~~~


Type of airspeed sensor


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | None              | |
| +-------+-------------------+ |
| | 1     | I2C-MS4525D0      | |
| +-------+-------------------+ |
| | 2     | Analog            | |
| +-------+-------------------+ |
| | 3     | I2C-MS5525        | |
| +-------+-------------------+ |
| | 4     | I2C-MS5525 (0x76) | |
| +-------+-------------------+ |
| | 5     | I2C-MS5525 (0x77) | |
| +-------+-------------------+ |
| | 6     | I2C-SDP3X         | |
| +-------+-------------------+ |
| | 7     | I2C-DLVR-5in      | |
| +-------+-------------------+ |
| | 8     | DroneCAN          | |
| +-------+-------------------+ |
| | 9     | I2C-DLVR-10in     | |
| +-------+-------------------+ |
| | 10    | I2C-DLVR-20in     | |
| +-------+-------------------+ |
| | 11    | I2C-DLVR-30in     | |
| +-------+-------------------+ |
| | 12    | I2C-DLVR-60in     | |
| +-------+-------------------+ |
| | 13    | NMEA water speed  | |
| +-------+-------------------+ |
| | 14    | MSP               | |
| +-------+-------------------+ |
| | 15    | ASP5033           | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _ARSP_DEVID:

ARSP\_DEVID: Airspeed ID
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _ARSP_USE:

ARSP\_USE: Airspeed use
~~~~~~~~~~~~~~~~~~~~~~~


Enables airspeed use for automatic throttle modes and replaces control from THR\_TRIM\. Continues to display and log airspeed if set to 0\. Uses airspeed for control if set to 1\. Only uses airspeed when throttle \= 0 if set to 2 \(useful for gliders with airspeed sensors behind propellers\)\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | DoNotUse            | |
| +-------+---------------------+ |
| | 1     | Use                 | |
| +-------+---------------------+ |
| | 2     | UseWhenZeroThrottle | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _ARSP_OFFSET:

ARSP\_OFFSET: Airspeed offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration offset


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSP_RATIO:

ARSP\_RATIO: Airspeed ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity\. Increasing this value will indicate a higher airspeed at any given dynamic pressure\.


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSP_PIN:

ARSP\_PIN: Airspeed pin
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors\. Set to 15 on the Pixhawk for the analog airspeed port\. 


.. _ARSP_AUTOCAL:

ARSP\_AUTOCAL: Automatic airspeed ratio calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enables automatic adjustment of ARSPD\_RATIO during a calibration flight based on estimation of ground speed and true airspeed\. New ratio saved every 2 minutes if change is \> 5\%\. Should not be left enabled\.


.. _ARSP_TUBE_ORDER:

ARSP\_TUBE\_ORDER: Control pitot tube order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the first \(often the top\) connector on the sensor needs to be the stagnation pressure \(the pressure at the tip of the pitot tube\)\. If set to 1 then the second \(often the bottom\) connector needs to be the stagnation pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall\, which would otherwise be seen as a positive airspeed\.


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | Normal      | |
| +-------+-------------+ |
| | 1     | Swapped     | |
| +-------+-------------+ |
| | 2     | Auto Detect | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _ARSP_SKIP_CAL:

ARSP\_SKIP\_CAL: Skip airspeed calibration on startup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup\, instead using the offset from the last calibration\. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _ARSP_PSI_RANGE:

ARSP\_PSI\_RANGE: The PSI range of the device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to to set the PSI \(pounds per square inch\) range for your sensor\. You should not change this unless you examine the datasheet for your device


.. _ARSP_BUS:

ARSP\_BUS: Airspeed I2C bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | 0     | Bus0(internal)  | |
| +-------+-----------------+ |
| | 1     | Bus1(external)  | |
| +-------+-----------------+ |
| | 2     | Bus2(auxillary) | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _ARSP_PRIMARY:

ARSP\_PRIMARY: Primary airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This selects which airspeed sensor will be the primary if multiple sensors are found


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | FirstSensor | |
| +-------+-------------+ |
| | 1     | 2ndSensor   | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _ARSP_OPTIONS:

ARSP\_OPTIONS: Airspeed options bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask of options to use with airspeed\. 0\:Disable use based on airspeed\/groundspeed mismatch \(see ARSPD\_WIND\_MAX\)\, 1\:Automatically reenable use based on airspeed\/groundspeed mismatch recovery \(see ARSPD\_WIND\_MAX\) 2\:Disable voltage correction


+--------------------------------------+
| Bitmask                              |
+======================================+
| +-----+----------------------------+ |
| | Bit | Meaning                    | |
| +=====+============================+ |
| | 0   | SpeedMismatchDisable       | |
| +-----+----------------------------+ |
| | 1   | AllowSpeedMismatchRecovery | |
| +-----+----------------------------+ |
| | 2   | DisableVoltageCorrection   | |
| +-----+----------------------------+ |
|                                      |
+--------------------------------------+




.. _ARSP_WIND_MAX:

ARSP\_WIND\_MAX: Maximum airspeed and ground speed difference
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy\. Using ARSPD\_OPTION this health value can be used to disable the sensor\.


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _ARSP_WIND_WARN:

ARSP\_WIND\_WARN: Airspeed and ground speed difference that gives a warning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will issue a warning\. If 0 ARSPD\_WIND\_MAX is used\.


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _ARSP2_TYPE:

ARSP2\_TYPE: Second Airspeed type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of 2nd airspeed sensor


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | None              | |
| +-------+-------------------+ |
| | 1     | I2C-MS4525D0      | |
| +-------+-------------------+ |
| | 2     | Analog            | |
| +-------+-------------------+ |
| | 3     | I2C-MS5525        | |
| +-------+-------------------+ |
| | 4     | I2C-MS5525 (0x76) | |
| +-------+-------------------+ |
| | 5     | I2C-MS5525 (0x77) | |
| +-------+-------------------+ |
| | 6     | I2C-SDP3X         | |
| +-------+-------------------+ |
| | 7     | I2C-DLVR-5in      | |
| +-------+-------------------+ |
| | 8     | DroneCAN          | |
| +-------+-------------------+ |
| | 9     | I2C-DLVR-10in     | |
| +-------+-------------------+ |
| | 10    | I2C-DLVR-20in     | |
| +-------+-------------------+ |
| | 11    | I2C-DLVR-30in     | |
| +-------+-------------------+ |
| | 12    | I2C-DLVR-60in     | |
| +-------+-------------------+ |
| | 13    | NMEA water speed  | |
| +-------+-------------------+ |
| | 14    | MSP               | |
| +-------+-------------------+ |
| | 15    | ASP5033           | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _ARSP2_USE:

ARSP2\_USE: Enable use of 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


use airspeed for flight control\. When set to 0 airspeed sensor can be logged and displayed on a GCS but won\'t be used for flight\. When set to 1 it will be logged and used\. When set to 2 it will be only used when the throttle is zero\, which can be useful in gliders with airspeed sensors behind a propeller


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | Don't Use           | |
| +-------+---------------------+ |
| | 1     | use                 | |
| +-------+---------------------+ |
| | 2     | UseWhenZeroThrottle | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _ARSP2_OFFSET:

ARSP2\_OFFSET: Airspeed offset for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration offset


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSP2_RATIO:

ARSP2\_RATIO: Airspeed ratio for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration ratio


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSP2_PIN:

ARSP2\_PIN: Airspeed pin for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Pin number indicating location of analog airspeed sensors\. Pixhawk\/Cube if set to 15\. 


.. _ARSP2_AUTOCAL:

ARSP2\_AUTOCAL: Automatic airspeed ratio calibration for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If this is enabled then the autopilot will automatically adjust the ARSPD\_RATIO during flight\, based upon an estimation filter using ground speed and true airspeed\. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5\%\. This option should be enabled for a calibration flight then disabled again when calibration is complete\. Leaving it enabled all the time is not recommended\.


.. _ARSP2_TUBE_ORDR:

ARSP2\_TUBE\_ORDR: Control pitot tube order of 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the first \(often the top\) connector on the sensor needs to be the stagnation pressure \(the pressure at the tip of the pitot tube\)\. If set to 1 then the second \(often the bottom\) connector needs to be the stagnation pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall\, which would otherwise be seen as a positive airspeed\.


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | Normal      | |
| +-------+-------------+ |
| | 1     | Swapped     | |
| +-------+-------------+ |
| | 2     | Auto Detect | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _ARSP2_SKIP_CAL:

ARSP2\_SKIP\_CAL: Skip airspeed calibration on startup for 2nd sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup\, instead using the offset from the last calibration\. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _ARSP2_PSI_RANGE:

ARSP2\_PSI\_RANGE: The PSI range of the device for 2nd sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to to set the PSI \(pounds per square inch\) range for your sensor\. You should not change this unless you examine the datasheet for your device


.. _ARSP2_BUS:

ARSP2\_BUS: Airspeed I2C bus for 2nd sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The bus number of the I2C bus to look for the sensor on


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | 0     | Bus0(internal)  | |
| +-------+-----------------+ |
| | 1     | Bus1(external)  | |
| +-------+-----------------+ |
| | 2     | Bus2(auxillary) | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _ARSP2_DEVID:

ARSP2\_DEVID: Airspeed2 ID
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed2 sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+





.. _parameters_ARSPD:

ARSPD Parameters
----------------


.. _ARSPD_TYPE:

ARSPD\_TYPE: Airspeed type
~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of airspeed sensor


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | None              | |
| +-------+-------------------+ |
| | 1     | I2C-MS4525D0      | |
| +-------+-------------------+ |
| | 2     | Analog            | |
| +-------+-------------------+ |
| | 3     | I2C-MS5525        | |
| +-------+-------------------+ |
| | 4     | I2C-MS5525 (0x76) | |
| +-------+-------------------+ |
| | 5     | I2C-MS5525 (0x77) | |
| +-------+-------------------+ |
| | 6     | I2C-SDP3X         | |
| +-------+-------------------+ |
| | 7     | I2C-DLVR-5in      | |
| +-------+-------------------+ |
| | 8     | DroneCAN          | |
| +-------+-------------------+ |
| | 9     | I2C-DLVR-10in     | |
| +-------+-------------------+ |
| | 10    | I2C-DLVR-20in     | |
| +-------+-------------------+ |
| | 11    | I2C-DLVR-30in     | |
| +-------+-------------------+ |
| | 12    | I2C-DLVR-60in     | |
| +-------+-------------------+ |
| | 13    | NMEA water speed  | |
| +-------+-------------------+ |
| | 14    | MSP               | |
| +-------+-------------------+ |
| | 15    | ASP5033           | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _ARSPD_DEVID:

ARSPD\_DEVID: Airspeed ID
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _ARSPD_USE:

ARSPD\_USE: Airspeed use
~~~~~~~~~~~~~~~~~~~~~~~~


Enables airspeed use for automatic throttle modes and replaces control from THR\_TRIM\. Continues to display and log airspeed if set to 0\. Uses airspeed for control if set to 1\. Only uses airspeed when throttle \= 0 if set to 2 \(useful for gliders with airspeed sensors behind propellers\)\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | DoNotUse            | |
| +-------+---------------------+ |
| | 1     | Use                 | |
| +-------+---------------------+ |
| | 2     | UseWhenZeroThrottle | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _ARSPD_OFFSET:

ARSPD\_OFFSET: Airspeed offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration offset


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD_RATIO:

ARSPD\_RATIO: Airspeed ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity\. Increasing this value will indicate a higher airspeed at any given dynamic pressure\.


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD_PIN:

ARSPD\_PIN: Airspeed pin
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors\. Set to 15 on the Pixhawk for the analog airspeed port\. 


.. _ARSPD_AUTOCAL:

ARSPD\_AUTOCAL: Automatic airspeed ratio calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enables automatic adjustment of ARSPD\_RATIO during a calibration flight based on estimation of ground speed and true airspeed\. New ratio saved every 2 minutes if change is \> 5\%\. Should not be left enabled\.


.. _ARSPD_TUBE_ORDER:

ARSPD\_TUBE\_ORDER: Control pitot tube order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the first \(often the top\) connector on the sensor needs to be the stagnation pressure \(the pressure at the tip of the pitot tube\)\. If set to 1 then the second \(often the bottom\) connector needs to be the stagnation pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall\, which would otherwise be seen as a positive airspeed\.


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | Normal      | |
| +-------+-------------+ |
| | 1     | Swapped     | |
| +-------+-------------+ |
| | 2     | Auto Detect | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _ARSPD_SKIP_CAL:

ARSPD\_SKIP\_CAL: Skip airspeed calibration on startup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup\, instead using the offset from the last calibration\. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _ARSPD_PSI_RANGE:

ARSPD\_PSI\_RANGE: The PSI range of the device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to to set the PSI \(pounds per square inch\) range for your sensor\. You should not change this unless you examine the datasheet for your device


.. _ARSPD_BUS:

ARSPD\_BUS: Airspeed I2C bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | 0     | Bus0(internal)  | |
| +-------+-----------------+ |
| | 1     | Bus1(external)  | |
| +-------+-----------------+ |
| | 2     | Bus2(auxillary) | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _ARSPD_PRIMARY:

ARSPD\_PRIMARY: Primary airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This selects which airspeed sensor will be the primary if multiple sensors are found


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | FirstSensor | |
| +-------+-------------+ |
| | 1     | 2ndSensor   | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _ARSPD_OPTIONS:

ARSPD\_OPTIONS: Airspeed options bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask of options to use with airspeed\. 0\:Disable use based on airspeed\/groundspeed mismatch \(see ARSPD\_WIND\_MAX\)\, 1\:Automatically reenable use based on airspeed\/groundspeed mismatch recovery \(see ARSPD\_WIND\_MAX\) 2\:Disable voltage correction


+--------------------------------------+
| Bitmask                              |
+======================================+
| +-----+----------------------------+ |
| | Bit | Meaning                    | |
| +=====+============================+ |
| | 0   | SpeedMismatchDisable       | |
| +-----+----------------------------+ |
| | 1   | AllowSpeedMismatchRecovery | |
| +-----+----------------------------+ |
| | 2   | DisableVoltageCorrection   | |
| +-----+----------------------------+ |
|                                      |
+--------------------------------------+




.. _ARSPD_WIND_MAX:

ARSPD\_WIND\_MAX: Maximum airspeed and ground speed difference
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy\. Using ARSPD\_OPTION this health value can be used to disable the sensor\.


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _ARSPD_WIND_WARN:

ARSPD\_WIND\_WARN: Airspeed and ground speed difference that gives a warning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will issue a warning\. If 0 ARSPD\_WIND\_MAX is used\.


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _ARSPD2_TYPE:

ARSPD2\_TYPE: Second Airspeed type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of 2nd airspeed sensor


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | None              | |
| +-------+-------------------+ |
| | 1     | I2C-MS4525D0      | |
| +-------+-------------------+ |
| | 2     | Analog            | |
| +-------+-------------------+ |
| | 3     | I2C-MS5525        | |
| +-------+-------------------+ |
| | 4     | I2C-MS5525 (0x76) | |
| +-------+-------------------+ |
| | 5     | I2C-MS5525 (0x77) | |
| +-------+-------------------+ |
| | 6     | I2C-SDP3X         | |
| +-------+-------------------+ |
| | 7     | I2C-DLVR-5in      | |
| +-------+-------------------+ |
| | 8     | DroneCAN          | |
| +-------+-------------------+ |
| | 9     | I2C-DLVR-10in     | |
| +-------+-------------------+ |
| | 10    | I2C-DLVR-20in     | |
| +-------+-------------------+ |
| | 11    | I2C-DLVR-30in     | |
| +-------+-------------------+ |
| | 12    | I2C-DLVR-60in     | |
| +-------+-------------------+ |
| | 13    | NMEA water speed  | |
| +-------+-------------------+ |
| | 14    | MSP               | |
| +-------+-------------------+ |
| | 15    | ASP5033           | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _ARSPD2_USE:

ARSPD2\_USE: Enable use of 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


use airspeed for flight control\. When set to 0 airspeed sensor can be logged and displayed on a GCS but won\'t be used for flight\. When set to 1 it will be logged and used\. When set to 2 it will be only used when the throttle is zero\, which can be useful in gliders with airspeed sensors behind a propeller


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | Don't Use           | |
| +-------+---------------------+ |
| | 1     | use                 | |
| +-------+---------------------+ |
| | 2     | UseWhenZeroThrottle | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _ARSPD2_OFFSET:

ARSPD2\_OFFSET: Airspeed offset for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration offset


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD2_RATIO:

ARSPD2\_RATIO: Airspeed ratio for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration ratio


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD2_PIN:

ARSPD2\_PIN: Airspeed pin for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Pin number indicating location of analog airspeed sensors\. Pixhawk\/Cube if set to 15\. 


.. _ARSPD2_AUTOCAL:

ARSPD2\_AUTOCAL: Automatic airspeed ratio calibration for 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If this is enabled then the autopilot will automatically adjust the ARSPD\_RATIO during flight\, based upon an estimation filter using ground speed and true airspeed\. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5\%\. This option should be enabled for a calibration flight then disabled again when calibration is complete\. Leaving it enabled all the time is not recommended\.


.. _ARSPD2_TUBE_ORDR:

ARSPD2\_TUBE\_ORDR: Control pitot tube order of 2nd airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the first \(often the top\) connector on the sensor needs to be the stagnation pressure \(the pressure at the tip of the pitot tube\)\. If set to 1 then the second \(often the bottom\) connector needs to be the stagnation pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall\, which would otherwise be seen as a positive airspeed\.


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | Normal      | |
| +-------+-------------+ |
| | 1     | Swapped     | |
| +-------+-------------+ |
| | 2     | Auto Detect | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _ARSPD2_SKIP_CAL:

ARSPD2\_SKIP\_CAL: Skip airspeed calibration on startup for 2nd sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup\, instead using the offset from the last calibration\. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _ARSPD2_PSI_RANGE:

ARSPD2\_PSI\_RANGE: The PSI range of the device for 2nd sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to to set the PSI \(pounds per square inch\) range for your sensor\. You should not change this unless you examine the datasheet for your device


.. _ARSPD2_BUS:

ARSPD2\_BUS: Airspeed I2C bus for 2nd sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The bus number of the I2C bus to look for the sensor on


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | 0     | Bus0(internal)  | |
| +-------+-----------------+ |
| | 1     | Bus1(external)  | |
| +-------+-----------------+ |
| | 2     | Bus2(auxillary) | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _ARSPD2_DEVID:

ARSPD2\_DEVID: Airspeed2 ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed2 sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+





.. _parameters_BARO:

BARO Parameters
---------------


.. _BARO1_GND_PRESS:

BARO1\_GND\_PRESS: Ground Pressure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

calibrated ground pressure in Pascals


+-----------+----------+--------+----------+
| Increment | ReadOnly | Units  | Volatile |
+===========+==========+========+==========+
| 1         | True     | pascal | True     |
+-----------+----------+--------+----------+




.. _BARO_GND_TEMP:

BARO\_GND\_TEMP: ground temperature
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

User provided ambient ground temperature in degrees Celsius\. This is used to improve the calculation of the altitude the vehicle is at\. This parameter is not persistent and will be reset to 0 every time the vehicle is rebooted\. A value of 0 means use the internal measurement ambient temperature\.


+-----------+-----------------+----------+
| Increment | Units           | Volatile |
+===========+=================+==========+
| 1         | degrees Celsius | True     |
+-----------+-----------------+----------+




.. _BARO_ALT_OFFSET:

BARO\_ALT\_OFFSET: altitude offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

altitude offset in meters added to barometric altitude\. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer\. The value is added to the barometric altitude read by the aircraft\. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed\.


+-----------+--------+
| Increment | Units  |
+===========+========+
| 0.1       | meters |
+-----------+--------+




.. _BARO_PRIMARY:

BARO\_PRIMARY: Primary barometer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This selects which barometer will be the primary if multiple barometers are found


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | FirstBaro | |
| +-------+-----------+ |
| | 1     | 2ndBaro   | |
| +-------+-----------+ |
| | 2     | 3rdBaro   | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _BARO_EXT_BUS:

BARO\_EXT\_BUS: External baro bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This selects the bus number for looking for an I2C barometer\. When set to \-1 it will probe all external i2c buses based on the GND\_PROBE\_EXT parameter\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 0     | Bus0     | |
| +-------+----------+ |
| | 1     | Bus1     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BARO_SPEC_GRAV:

BARO\_SPEC\_GRAV: Specific Gravity \(For water depth measurement\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the specific gravity of the fluid when flying an underwater ROV\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 1.0   | Freshwater | |
| +-------+------------+ |
| | 1.024 | Saltwater  | |
| +-------+------------+ |
|                        |
+------------------------+




.. _BARO2_GND_PRESS:

BARO2\_GND\_PRESS: Ground Pressure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

calibrated ground pressure in Pascals


+-----------+----------+--------+----------+
| Increment | ReadOnly | Units  | Volatile |
+===========+==========+========+==========+
| 1         | True     | pascal | True     |
+-----------+----------+--------+----------+




.. _BARO3_GND_PRESS:

BARO3\_GND\_PRESS: Absolute Pressure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

calibrated ground pressure in Pascals


+-----------+----------+--------+----------+
| Increment | ReadOnly | Units  | Volatile |
+===========+==========+========+==========+
| 1         | True     | pascal | True     |
+-----------+----------+--------+----------+




.. _BARO_FLTR_RNG:

BARO\_FLTR\_RNG: Range in which sample is accepted
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the range around the average value that new samples must be within to be accepted\. This can help reduce the impact of noise on sensors that are on long I2C cables\. The value is a percentage from the average value\. A value of zero disables this filter\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 100 | percent |
+-----------+---------+---------+




.. _BARO_PROBE_EXT:

BARO\_PROBE\_EXT: External barometers to probe
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets which types of external i2c barometer to look for\. It is a bitmask of barometer types\. The I2C buses to probe is based on GND\_EXT\_BUS\. If BARO\_EXT\_BUS is \-1 then it will probe all external buses\, otherwise it will probe just the bus number given in GND\_EXT\_BUS\.


+-------------------+
| Bitmask           |
+===================+
| +-----+---------+ |
| | Bit | Meaning | |
| +=====+=========+ |
| | 0   | BMP085  | |
| +-----+---------+ |
| | 1   | BMP280  | |
| +-----+---------+ |
| | 2   | MS5611  | |
| +-----+---------+ |
| | 3   | MS5607  | |
| +-----+---------+ |
| | 4   | MS5637  | |
| +-----+---------+ |
| | 5   | FBM320  | |
| +-----+---------+ |
| | 6   | DPS280  | |
| +-----+---------+ |
| | 7   | LPS25H  | |
| +-----+---------+ |
| | 8   | Keller  | |
| +-----+---------+ |
| | 9   | MS5837  | |
| +-----+---------+ |
| | 10  | BMP388  | |
| +-----+---------+ |
| | 11  | SPL06   | |
| +-----+---------+ |
| | 12  | MSP     | |
| +-----+---------+ |
|                   |
+-------------------+




.. _BARO1_DEVID:

BARO1\_DEVID: Baro ID
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Barometer sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _BARO2_DEVID:

BARO2\_DEVID: Baro ID2
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Barometer2 sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _BARO3_DEVID:

BARO3\_DEVID: Baro ID3
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Barometer3 sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+





.. _parameters_BARO1_WCF_:

BARO1\_WCF\_ Parameters
-----------------------


.. _BARO1_WCF_ENABLE:

BARO1\_WCF\_ENABLE: Wind coefficient enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables the use of wind coefficients for barometer compensation


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BARO1_WCF_FWD:

BARO1\_WCF\_FWD: Pressure error coefficient in positive X direction \(forward\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis\. If the baro height estimate rises during forwards flight\, then this will be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO1_WCF_BCK:

BARO1\_WCF\_BCK: Pressure error coefficient in negative X direction \(backwards\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis\. If the baro height estimate rises during backwards flight\, then this will be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO1_WCF_RGT:

BARO1\_WCF\_RGT: Pressure error coefficient in positive Y direction \(right\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Y body axis\. If the baro height estimate rises during sideways flight to the right\, then this should be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO1_WCF_LFT:

BARO1\_WCF\_LFT: Pressure error coefficient in negative Y direction \(left\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Y body axis\. If the baro height estimate rises during sideways flight to the left\, then this should be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+





.. _parameters_BARO2_WCF_:

BARO2\_WCF\_ Parameters
-----------------------


.. _BARO2_WCF_ENABLE:

BARO2\_WCF\_ENABLE: Wind coefficient enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables the use of wind coefficients for barometer compensation


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BARO2_WCF_FWD:

BARO2\_WCF\_FWD: Pressure error coefficient in positive X direction \(forward\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis\. If the baro height estimate rises during forwards flight\, then this will be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO2_WCF_BCK:

BARO2\_WCF\_BCK: Pressure error coefficient in negative X direction \(backwards\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis\. If the baro height estimate rises during backwards flight\, then this will be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO2_WCF_RGT:

BARO2\_WCF\_RGT: Pressure error coefficient in positive Y direction \(right\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Y body axis\. If the baro height estimate rises during sideways flight to the right\, then this should be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO2_WCF_LFT:

BARO2\_WCF\_LFT: Pressure error coefficient in negative Y direction \(left\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Y body axis\. If the baro height estimate rises during sideways flight to the left\, then this should be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+





.. _parameters_BARO3_WCF_:

BARO3\_WCF\_ Parameters
-----------------------


.. _BARO3_WCF_ENABLE:

BARO3\_WCF\_ENABLE: Wind coefficient enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables the use of wind coefficients for barometer compensation


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BARO3_WCF_FWD:

BARO3\_WCF\_FWD: Pressure error coefficient in positive X direction \(forward\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis\. If the baro height estimate rises during forwards flight\, then this will be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO3_WCF_BCK:

BARO3\_WCF\_BCK: Pressure error coefficient in negative X direction \(backwards\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis\. If the baro height estimate rises during backwards flight\, then this will be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO3_WCF_RGT:

BARO3\_WCF\_RGT: Pressure error coefficient in positive Y direction \(right\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Y body axis\. If the baro height estimate rises during sideways flight to the right\, then this should be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+




.. _BARO3_WCF_LFT:

BARO3\_WCF\_LFT: Pressure error coefficient in negative Y direction \(left\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Y body axis\. If the baro height estimate rises during sideways flight to the left\, then this should be a negative number\. Multirotors can use this feature only if using EKF3 and if the EK3\_BCOEF\_X and EK3\_BCOEF\_Y parameters have been tuned\.


+-----------+------------+
| Increment | Range      |
+===========+============+
| 0.05      | -1.0 - 1.0 |
+-----------+------------+





.. _parameters_BATT2_:

BATT2\_ Parameters
------------------


.. _BATT2_MONITOR:

BATT2\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT2_CAPACITY:

BATT2\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT2_SERIAL_NUM:

BATT2\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT2_LOW_TIMER:

BATT2\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT2_FS_VOLTSRC:

BATT2\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT2_LOW_VOLT:

BATT2\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT2\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT2\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT2_LOW_MAH:

BATT2\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT2\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT2_CRT_VOLT:

BATT2\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT2\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT2\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT2_CRT_MAH:

BATT2\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT2\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT2_ARM_VOLT:

BATT2\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT2_ARM_MAH:

BATT2\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT2\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT2_OPTIONS:

BATT2\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT2_VOLT_PIN:

BATT2\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT2_CURR_PIN:

BATT2\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT2_VOLT_MULT:

BATT2\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT2\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT2_AMP_PERVLT:

BATT2\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT2_AMP_OFFSET:

BATT2\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT2_VLT_OFFSET:

BATT2\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT2_I2C_BUS:

BATT2\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT2_I2C_ADDR:

BATT2\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT2_SUM_MASK:

BATT2\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT2_CURR_MULT:

BATT2\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT3_:

BATT3\_ Parameters
------------------


.. _BATT3_MONITOR:

BATT3\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT3_CAPACITY:

BATT3\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT3_SERIAL_NUM:

BATT3\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT3_LOW_TIMER:

BATT3\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT3_FS_VOLTSRC:

BATT3\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT3_LOW_VOLT:

BATT3\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT3\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT3\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT3_LOW_MAH:

BATT3\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT3\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT3_CRT_VOLT:

BATT3\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT3\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT3\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT3_CRT_MAH:

BATT3\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT3\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT3_ARM_VOLT:

BATT3\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT3_ARM_MAH:

BATT3\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT3\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT3_OPTIONS:

BATT3\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT3_VOLT_PIN:

BATT3\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT3_CURR_PIN:

BATT3\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT3_VOLT_MULT:

BATT3\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT3\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT3_AMP_PERVLT:

BATT3\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT3_AMP_OFFSET:

BATT3\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT3_VLT_OFFSET:

BATT3\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT3_I2C_BUS:

BATT3\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT3_I2C_ADDR:

BATT3\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT3_SUM_MASK:

BATT3\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT3_CURR_MULT:

BATT3\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT4_:

BATT4\_ Parameters
------------------


.. _BATT4_MONITOR:

BATT4\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT4_CAPACITY:

BATT4\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT4_SERIAL_NUM:

BATT4\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT4_LOW_TIMER:

BATT4\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT4_FS_VOLTSRC:

BATT4\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT4_LOW_VOLT:

BATT4\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT4\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT4\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT4_LOW_MAH:

BATT4\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT4\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT4_CRT_VOLT:

BATT4\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT4\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT4\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT4_CRT_MAH:

BATT4\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT4\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT4_ARM_VOLT:

BATT4\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT4_ARM_MAH:

BATT4\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT4\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT4_OPTIONS:

BATT4\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT4_VOLT_PIN:

BATT4\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT4_CURR_PIN:

BATT4\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT4_VOLT_MULT:

BATT4\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT4\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT4_AMP_PERVLT:

BATT4\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT4_AMP_OFFSET:

BATT4\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT4_VLT_OFFSET:

BATT4\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT4_I2C_BUS:

BATT4\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT4_I2C_ADDR:

BATT4\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT4_SUM_MASK:

BATT4\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT4_CURR_MULT:

BATT4\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT5_:

BATT5\_ Parameters
------------------


.. _BATT5_MONITOR:

BATT5\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT5_CAPACITY:

BATT5\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT5_SERIAL_NUM:

BATT5\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT5_LOW_TIMER:

BATT5\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT5_FS_VOLTSRC:

BATT5\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT5_LOW_VOLT:

BATT5\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT5\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT5\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT5_LOW_MAH:

BATT5\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT5\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT5_CRT_VOLT:

BATT5\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT5\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT5\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT5_CRT_MAH:

BATT5\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT5\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT5_ARM_VOLT:

BATT5\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT5_ARM_MAH:

BATT5\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT5\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT5_OPTIONS:

BATT5\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT5_VOLT_PIN:

BATT5\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT5_CURR_PIN:

BATT5\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT5_VOLT_MULT:

BATT5\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT5\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT5_AMP_PERVLT:

BATT5\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT5_AMP_OFFSET:

BATT5\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT5_VLT_OFFSET:

BATT5\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT5_I2C_BUS:

BATT5\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT5_I2C_ADDR:

BATT5\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT5_SUM_MASK:

BATT5\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT5_CURR_MULT:

BATT5\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT6_:

BATT6\_ Parameters
------------------


.. _BATT6_MONITOR:

BATT6\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT6_CAPACITY:

BATT6\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT6_SERIAL_NUM:

BATT6\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT6_LOW_TIMER:

BATT6\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT6_FS_VOLTSRC:

BATT6\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT6_LOW_VOLT:

BATT6\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT6\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT6\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT6_LOW_MAH:

BATT6\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT6\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT6_CRT_VOLT:

BATT6\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT6\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT6\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT6_CRT_MAH:

BATT6\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT6\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT6_ARM_VOLT:

BATT6\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT6_ARM_MAH:

BATT6\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT6\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT6_OPTIONS:

BATT6\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT6_VOLT_PIN:

BATT6\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT6_CURR_PIN:

BATT6\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT6_VOLT_MULT:

BATT6\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT6\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT6_AMP_PERVLT:

BATT6\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT6_AMP_OFFSET:

BATT6\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT6_VLT_OFFSET:

BATT6\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT6_I2C_BUS:

BATT6\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT6_I2C_ADDR:

BATT6\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT6_SUM_MASK:

BATT6\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT6_CURR_MULT:

BATT6\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT7_:

BATT7\_ Parameters
------------------


.. _BATT7_MONITOR:

BATT7\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT7_CAPACITY:

BATT7\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT7_SERIAL_NUM:

BATT7\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT7_LOW_TIMER:

BATT7\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT7_FS_VOLTSRC:

BATT7\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT7_LOW_VOLT:

BATT7\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT7\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT7\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT7_LOW_MAH:

BATT7\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT7\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT7_CRT_VOLT:

BATT7\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT7\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT7\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT7_CRT_MAH:

BATT7\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT7\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT7_ARM_VOLT:

BATT7\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT7_ARM_MAH:

BATT7\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT7\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT7_OPTIONS:

BATT7\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT7_VOLT_PIN:

BATT7\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT7_CURR_PIN:

BATT7\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT7_VOLT_MULT:

BATT7\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT7\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT7_AMP_PERVLT:

BATT7\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT7_AMP_OFFSET:

BATT7\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT7_VLT_OFFSET:

BATT7\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT7_I2C_BUS:

BATT7\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT7_I2C_ADDR:

BATT7\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT7_SUM_MASK:

BATT7\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT7_CURR_MULT:

BATT7\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT8_:

BATT8\_ Parameters
------------------


.. _BATT8_MONITOR:

BATT8\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT8_CAPACITY:

BATT8\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT8_SERIAL_NUM:

BATT8\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT8_LOW_TIMER:

BATT8\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT8_FS_VOLTSRC:

BATT8\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT8_LOW_VOLT:

BATT8\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT8\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT8\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT8_LOW_MAH:

BATT8\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT8\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT8_CRT_VOLT:

BATT8\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT8\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT8\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT8_CRT_MAH:

BATT8\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT8\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT8_ARM_VOLT:

BATT8\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT8_ARM_MAH:

BATT8\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT8\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT8_OPTIONS:

BATT8\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT8_VOLT_PIN:

BATT8\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT8_CURR_PIN:

BATT8\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT8_VOLT_MULT:

BATT8\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT8\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT8_AMP_PERVLT:

BATT8\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT8_AMP_OFFSET:

BATT8\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT8_VLT_OFFSET:

BATT8\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT8_I2C_BUS:

BATT8\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT8_I2C_ADDR:

BATT8\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT8_SUM_MASK:

BATT8\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT8_CURR_MULT:

BATT8\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT9_:

BATT9\_ Parameters
------------------


.. _BATT9_MONITOR:

BATT9\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT9_CAPACITY:

BATT9\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT9_SERIAL_NUM:

BATT9\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT9_LOW_TIMER:

BATT9\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT9_FS_VOLTSRC:

BATT9\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT9_LOW_VOLT:

BATT9\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT9\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT9\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT9_LOW_MAH:

BATT9\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT9\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT9_CRT_VOLT:

BATT9\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT9\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT9\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT9_CRT_MAH:

BATT9\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT9\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT9_ARM_VOLT:

BATT9\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT9_ARM_MAH:

BATT9\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT9\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT9_OPTIONS:

BATT9\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT9_VOLT_PIN:

BATT9\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT9_CURR_PIN:

BATT9\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT9_VOLT_MULT:

BATT9\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT9\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT9_AMP_PERVLT:

BATT9\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT9_AMP_OFFSET:

BATT9\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT9_VLT_OFFSET:

BATT9\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT9_I2C_BUS:

BATT9\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT9_I2C_ADDR:

BATT9\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT9_SUM_MASK:

BATT9\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT9_CURR_MULT:

BATT9\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_BATT_:

BATT\_ Parameters
-----------------


.. _BATT_MONITOR:

BATT\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | Solo                       | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
| | 7     | SMBus-Generic              | |
| +-------+----------------------------+ |
| | 8     | DroneCAN-BatteryInfo       | |
| +-------+----------------------------+ |
| | 9     | ESC                        | |
| +-------+----------------------------+ |
| | 10    | Sum Of Selected Monitors   | |
| +-------+----------------------------+ |
| | 11    | FuelFlow                   | |
| +-------+----------------------------+ |
| | 12    | FuelLevelPWM               | |
| +-------+----------------------------+ |
| | 13    | SMBUS-SUI3                 | |
| +-------+----------------------------+ |
| | 14    | SMBUS-SUI6                 | |
| +-------+----------------------------+ |
| | 15    | NeoDesign                  | |
| +-------+----------------------------+ |
| | 16    | SMBus-Maxell               | |
| +-------+----------------------------+ |
| | 17    | Generator-Elec             | |
| +-------+----------------------------+ |
| | 18    | Generator-Fuel             | |
| +-------+----------------------------+ |
| | 19    | Rotoye                     | |
| +-------+----------------------------+ |
| | 20    | MPPT                       | |
| +-------+----------------------------+ |
| | 21    | INA2XX                     | |
| +-------+----------------------------+ |
| | 22    | LTC2946                    | |
| +-------+----------------------------+ |
| | 23    | Torqeedo                   | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT_CAPACITY:

BATT\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT_SERIAL_NUM:

BATT\_SERIAL\_NUM: Battery serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery serial number\, automatically filled in for SMBus batteries\, otherwise will be \-1\. With DroneCan it is the battery\_id\.


.. _BATT_LOW_TIMER:

BATT\_LOW\_TIMER: Low voltage timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered\. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs\. A value of zero disables low voltage errors\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 120 | seconds |
+-----------+---------+---------+




.. _BATT_FS_VOLTSRC:

BATT\_FS\_VOLTSRC: Failsafe voltage source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Raw Voltage             | |
| +-------+-------------------------+ |
| | 1     | Sag Compensated Voltage | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _BATT_LOW_VOLT:

BATT\_LOW\_VOLT: Low battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a low battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT\_FS\_LOW\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT_LOW_MAH:

BATT\_LOW\_MAH: Low battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the low battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT\_FS\_LOW\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT_CRT_VOLT:

BATT\_CRT\_VOLT: Critical battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage that triggers a critical battery failsafe\. Set to 0 to disable\. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT\_LOW\_TIMER parameter then the vehicle will perform the failsafe specified by the BATT\_FS\_CRT\_ACT parameter\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT_CRT_MAH:

BATT\_CRT\_MAH: Battery critical capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity at which the critical battery failsafe is triggered\. Set to 0 to disable battery remaining failsafe\. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT\_\_FS\_CRT\_ACT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT_ARM_VOLT:

BATT\_ARM\_VOLT: Required arming voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft\. Set to 0 to allow arming at any voltage\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | volt  |
+-----------+-------+




.. _BATT_ARM_MAH:

BATT\_ARM\_MAH: Required arming remaining capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft\. Set to 0 to allow arming at any capacity\. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate\, which can lead to this check not providing sufficent protection\, it is recommended to always use this in conjunction with the BATT\_\_ARM\_VOLT parameter\.


+-----------+------------------+
| Increment | Units            |
+===========+==================+
| 50        | milliampere hour |
+-----------+------------------+




.. _BATT_OPTIONS:

BATT\_OPTIONS: Battery monitor options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor


+--------------------------------------------------+
| Bitmask                                          |
+==================================================+
| +-----+----------------------------------------+ |
| | Bit | Meaning                                | |
| +=====+========================================+ |
| | 0   | Ignore DroneCAN SoC                    | |
| +-----+----------------------------------------+ |
| | 1   | MPPT reports input voltage and current | |
| +-----+----------------------------------------+ |
| | 2   | MPPT Powered off when disarmed         | |
| +-----+----------------------------------------+ |
| | 3   | MPPT Powered on when armed             | |
| +-----+----------------------------------------+ |
| | 4   | MPPT Powered off at boot               | |
| +-----+----------------------------------------+ |
| | 5   | MPPT Powered on at boot                | |
| +-----+----------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT_VOLT_PIN:

BATT\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for voltage monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 5     | Navigator                            | |
| +-------+--------------------------------------+ |
| | 13    | Pixhawk2_PM2/CubeOrange_PM2          | |
| +-------+--------------------------------------+ |
| | 14    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 16    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 100   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT_CURR_PIN:

BATT\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Sets the analog input pin that should be used for current monitoring\.


+--------------------------------------------------+
| Values                                           |
+==================================================+
| +-------+--------------------------------------+ |
| | Value | Meaning                              | |
| +=======+======================================+ |
| | -1    | Disabled                             | |
| +-------+--------------------------------------+ |
| | 3     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 | |
| +-------+--------------------------------------+ |
| | 4     | CubeOrange_PM2/Navigator             | |
| +-------+--------------------------------------+ |
| | 14    | Pixhawk2_PM2                         | |
| +-------+--------------------------------------+ |
| | 15    | CubeOrange                           | |
| +-------+--------------------------------------+ |
| | 17    | Durandal                             | |
| +-------+--------------------------------------+ |
| | 101   | PX4-v1                               | |
| +-------+--------------------------------------+ |
|                                                  |
+--------------------------------------------------+




.. _BATT_VOLT_MULT:

BATT\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick with a Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX using the PX4IO power supply this should be set to 1\.


.. _BATT_AMP_PERVLT:

BATT\_AMP\_PERVLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. With a Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------------+
| Units           |
+=================+
| ampere per volt |
+-----------------+




.. _BATT_AMP_OFFSET:

BATT\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT_VLT_OFFSET:

BATT\_VLT\_OFFSET: Volage offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Voltage offset on voltage pin\. This allows for an offset due to a diode\. This voltage is subtracted before the scaling is applied


+-------+
| Units |
+=======+
| volt  |
+-------+




.. _BATT_I2C_BUS:

BATT\_I2C\_BUS: Battery monitor I2C bus number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C bus number


+-------+
| Range |
+=======+
| 0 - 3 |
+-------+




.. _BATT_I2C_ADDR:

BATT\_I2C\_ADDR: Battery monitor I2C address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Battery monitor I2C address


+---------+
| Range   |
+=========+
| 0 - 127 |
+---------+




.. _BATT_SUM_MASK:

BATT\_SUM\_MASK: Battery Sum mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0\: sum of remaining battery monitors\, If none 0 sum of specified monitors\. Current will be summed and voltages averaged\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | monitor 1 | |
| +-----+-----------+ |
| | 1   | monitor 2 | |
| +-----+-----------+ |
| | 2   | monitor 3 | |
| +-----+-----------+ |
| | 3   | monitor 4 | |
| +-----+-----------+ |
| | 4   | monitor 5 | |
| +-----+-----------+ |
| | 5   | monitor 6 | |
| +-----+-----------+ |
| | 6   | monitor 7 | |
| +-----+-----------+ |
| | 7   | monitor 8 | |
| +-----+-----------+ |
| | 8   | monitor 9 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _BATT_CURR_MULT:

BATT\_CURR\_MULT: Scales reported power monitor current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications


+---------+
| Range   |
+=========+
| .1 - 10 |
+---------+





.. _parameters_CAM_RC_:

CAM\_RC\_ Parameters
--------------------


.. _CAM_RC_TYPE:

CAM\_RC\_TYPE: RunCam device type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RunCam deviee type used to determine OSD menu structure and shutter options\.


+-------------------------------------------------+
| Values                                          |
+=================================================+
| +-------+-------------------------------------+ |
| | Value | Meaning                             | |
| +=======+=====================================+ |
| | 0     | Disabled                            | |
| +-------+-------------------------------------+ |
| | 1     | RunCam Split Micro/RunCam with UART | |
| +-------+-------------------------------------+ |
| | 2     | RunCam Split                        | |
| +-------+-------------------------------------+ |
| | 3     | RunCam Split4 4k                    | |
| +-------+-------------------------------------+ |
| | 4     | RunCam Hybrid                       | |
| +-------+-------------------------------------+ |
|                                                 |
+-------------------------------------------------+




.. _CAM_RC_FEATURES:

CAM\_RC\_FEATURES: RunCam features available
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The available features of the attached RunCam device\. If 0 then the RunCam device will be queried for the features it supports\, otherwise this setting is used\.


+---------------------------+
| Bitmask                   |
+===========================+
| +-----+-----------------+ |
| | Bit | Meaning         | |
| +=====+=================+ |
| | 0   | Power Button    | |
| +-----+-----------------+ |
| | 1   | WiFi Button     | |
| +-----+-----------------+ |
| | 2   | Change Mode     | |
| +-----+-----------------+ |
| | 3   | 5-Key OSD       | |
| +-----+-----------------+ |
| | 4   | Settings Access | |
| +-----+-----------------+ |
| | 5   | DisplayPort     | |
| +-----+-----------------+ |
| | 6   | Start Recording | |
| +-----+-----------------+ |
| | 7   | Stop Recording  | |
| +-----+-----------------+ |
|                           |
+---------------------------+




.. _CAM_RC_BT_DELAY:

CAM\_RC\_BT\_DELAY: RunCam boot delay before allowing updates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Time it takes for the RunCam to become fully ready in ms\. If this is too short then commands can get out of sync\.


.. _CAM_RC_BTN_DELAY:

CAM\_RC\_BTN\_DELAY: RunCam button delay before allowing further button presses
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Time it takes for the a RunCam button press to be actived in ms\. If this is too short then commands can get out of sync\.


.. _CAM_RC_MDE_DELAY:

CAM\_RC\_MDE\_DELAY: RunCam mode delay before allowing further button presses
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Time it takes for the a RunCam mode button press to be actived in ms\. If a mode change first requires a video recording change then double this value is used\. If this is too short then commands can get out of sync\.


.. _CAM_RC_CONTROL:

CAM\_RC\_CONTROL: RunCam control option
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Specifies the allowed actions required to enter the OSD menu


+---------------------------------+
| Bitmask                         |
+=================================+
| +-----+-----------------------+ |
| | Bit | Meaning               | |
| +=====+=======================+ |
| | 0   | Stick yaw right       | |
| +-----+-----------------------+ |
| | 1   | Stick roll right      | |
| +-----+-----------------------+ |
| | 2   | 3-position switch     | |
| +-----+-----------------------+ |
| | 3   | 2-position switch     | |
| +-----+-----------------------+ |
| | 4   | Autorecording enabled | |
| +-----+-----------------------+ |
|                                 |
+---------------------------------+





.. _parameters_COMPASS_:

COMPASS\_ Parameters
--------------------


.. _COMPASS_OFS_X:

COMPASS\_OFS\_X: Compass offsets in milligauss on the X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to the compass x\-axis values to compensate for metal in the frame


+-------------+-----------+------------+------------+
| Calibration | Increment | Range      | Units      |
+=============+===========+============+============+
| 1           | 1         | -400 - 400 | milligauss |
+-------------+-----------+------------+------------+




.. _COMPASS_OFS_Y:

COMPASS\_OFS\_Y: Compass offsets in milligauss on the Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to the compass y\-axis values to compensate for metal in the frame


+-------------+-----------+------------+------------+
| Calibration | Increment | Range      | Units      |
+=============+===========+============+============+
| 1           | 1         | -400 - 400 | milligauss |
+-------------+-----------+------------+------------+




.. _COMPASS_OFS_Z:

COMPASS\_OFS\_Z: Compass offsets in milligauss on the Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to the compass z\-axis values to compensate for metal in the frame


+-----------+------------+------------+
| Increment | Range      | Units      |
+===========+============+============+
| 1         | -400 - 400 | milligauss |
+-----------+------------+------------+




.. _COMPASS_DEC:

COMPASS\_DEC: Compass declination
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


An angle to compensate between the true north and magnetic north


+-----------+----------------+---------+
| Increment | Range          | Units   |
+===========+================+=========+
| 0.01      | -3.142 - 3.142 | radians |
+-----------+----------------+---------+




.. _COMPASS_LEARN:

COMPASS\_LEARN: Learn compass offsets automatically
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the automatic learning of compass offsets\. You can enable learning either using a compass\-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator\. If this option is enabled then the learnt offsets are saved when you disarm the vehicle\. If InFlight learning is enabled then the compass with automatically start learning once a flight starts \(must be armed\)\. While InFlight learning is running you cannot use position control modes\.


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | Disabled          | |
| +-------+-------------------+ |
| | 1     | Internal-Learning | |
| +-------+-------------------+ |
| | 2     | EKF-Learning      | |
| +-------+-------------------+ |
| | 3     | InFlight-Learning | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _COMPASS_USE:

COMPASS\_USE: Use compass for yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the use of the compass \(instead of the GPS\) for determining heading


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_AUTODEC:

COMPASS\_AUTODEC: Auto Declination
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the automatic calculation of the declination based on gps location


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_MOTCT:

COMPASS\_MOTCT: Motor interference compensation type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Set motor interference compensation type to disabled\, throttle or current\.  Do not change manually\.


+-------------+--------------------------+
| Calibration | Values                   |
+=============+==========================+
| 1           | +-------+--------------+ |
|             | | Value | Meaning      | |
|             | +=======+==============+ |
|             | | 0     | Disabled     | |
|             | +-------+--------------+ |
|             | | 1     | Use Throttle | |
|             | +-------+--------------+ |
|             | | 2     | Use Current  | |
|             | +-------+--------------+ |
|             |                          |
+-------------+--------------------------+




.. _COMPASS_MOT_X:

COMPASS\_MOT\_X: Motor interference compensation for body frame X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to the compass\'s x\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-------------+-----------+--------------+-----------------------+
| Calibration | Increment | Range        | Units                 |
+=============+===========+==============+=======================+
| 1           | 1         | -1000 - 1000 | milligauss per ampere |
+-------------+-----------+--------------+-----------------------+




.. _COMPASS_MOT_Y:

COMPASS\_MOT\_Y: Motor interference compensation for body frame Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to the compass\'s y\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-------------+-----------+--------------+-----------------------+
| Calibration | Increment | Range        | Units                 |
+=============+===========+==============+=======================+
| 1           | 1         | -1000 - 1000 | milligauss per ampere |
+-------------+-----------+--------------+-----------------------+




.. _COMPASS_MOT_Z:

COMPASS\_MOT\_Z: Motor interference compensation for body frame Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to the compass\'s z\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-----------+--------------+-----------------------+
| Increment | Range        | Units                 |
+===========+==============+=======================+
| 1         | -1000 - 1000 | milligauss per ampere |
+-----------+--------------+-----------------------+




.. _COMPASS_ORIENT:

COMPASS\_ORIENT: Compass orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The orientation of the first external compass relative to the vehicle frame\. This value will be ignored unless this compass is set as an external compass\. When set correctly in the northern hemisphere\, pointing the nose and right side down should increase the MagX and MagY values respectively\. Rolling the vehicle upside down should decrease the MagZ value\. For southern hemisphere\, switch increase and decrease\. NOTE\: For internal compasses\, AHRS\_ORIENT is used\. The label for each option is specified in the order of rotations for that orientation\.


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | None                 | |
| +-------+----------------------+ |
| | 1     | Yaw45                | |
| +-------+----------------------+ |
| | 2     | Yaw90                | |
| +-------+----------------------+ |
| | 3     | Yaw135               | |
| +-------+----------------------+ |
| | 4     | Yaw180               | |
| +-------+----------------------+ |
| | 5     | Yaw225               | |
| +-------+----------------------+ |
| | 6     | Yaw270               | |
| +-------+----------------------+ |
| | 7     | Yaw315               | |
| +-------+----------------------+ |
| | 8     | Roll180              | |
| +-------+----------------------+ |
| | 9     | Yaw45Roll180         | |
| +-------+----------------------+ |
| | 10    | Yaw90Roll180         | |
| +-------+----------------------+ |
| | 11    | Yaw135Roll180        | |
| +-------+----------------------+ |
| | 12    | Pitch180             | |
| +-------+----------------------+ |
| | 13    | Yaw225Roll180        | |
| +-------+----------------------+ |
| | 14    | Yaw270Roll180        | |
| +-------+----------------------+ |
| | 15    | Yaw315Roll180        | |
| +-------+----------------------+ |
| | 16    | Roll90               | |
| +-------+----------------------+ |
| | 17    | Yaw45Roll90          | |
| +-------+----------------------+ |
| | 18    | Yaw90Roll90          | |
| +-------+----------------------+ |
| | 19    | Yaw135Roll90         | |
| +-------+----------------------+ |
| | 20    | Roll270              | |
| +-------+----------------------+ |
| | 21    | Yaw45Roll270         | |
| +-------+----------------------+ |
| | 22    | Yaw90Roll270         | |
| +-------+----------------------+ |
| | 23    | Yaw135Roll270        | |
| +-------+----------------------+ |
| | 24    | Pitch90              | |
| +-------+----------------------+ |
| | 25    | Pitch270             | |
| +-------+----------------------+ |
| | 26    | Yaw90Pitch180        | |
| +-------+----------------------+ |
| | 27    | Yaw270Pitch180       | |
| +-------+----------------------+ |
| | 28    | Pitch90Roll90        | |
| +-------+----------------------+ |
| | 29    | Pitch90Roll180       | |
| +-------+----------------------+ |
| | 30    | Pitch90Roll270       | |
| +-------+----------------------+ |
| | 31    | Pitch180Roll90       | |
| +-------+----------------------+ |
| | 32    | Pitch180Roll270      | |
| +-------+----------------------+ |
| | 33    | Pitch270Roll90       | |
| +-------+----------------------+ |
| | 34    | Pitch270Roll180      | |
| +-------+----------------------+ |
| | 35    | Pitch270Roll270      | |
| +-------+----------------------+ |
| | 36    | Yaw90Pitch180Roll90  | |
| +-------+----------------------+ |
| | 37    | Yaw270Roll90         | |
| +-------+----------------------+ |
| | 38    | Yaw293Pitch68Roll180 | |
| +-------+----------------------+ |
| | 39    | Pitch315             | |
| +-------+----------------------+ |
| | 40    | Pitch315Roll90       | |
| +-------+----------------------+ |
| | 42    | Roll45               | |
| +-------+----------------------+ |
| | 43    | Roll315              | |
| +-------+----------------------+ |
| | 100   | Custom               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _COMPASS_EXTERNAL:

COMPASS\_EXTERNAL: Compass is attached via an external cable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Configure compass so it is attached externally\. This is auto\-detected on most boards\. Set to 1 if the compass is externally connected\. When externally connected the COMPASS\_ORIENT option operates independently of the AHRS\_ORIENTATION board orientation option\. If set to 0 or 1 then auto\-detection by bus connection can override the value\. If set to 2 then auto\-detection will be disabled\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | 0     | Internal       | |
| +-------+----------------+ |
| | 1     | External       | |
| +-------+----------------+ |
| | 2     | ForcedExternal | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _COMPASS_OFS2_X:

COMPASS\_OFS2\_X: Compass2 offsets in milligauss on the X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to compass2\'s x\-axis values to compensate for metal in the frame


+-------------+-----------+------------+------------+
| Calibration | Increment | Range      | Units      |
+=============+===========+============+============+
| 1           | 1         | -400 - 400 | milligauss |
+-------------+-----------+------------+------------+




.. _COMPASS_OFS2_Y:

COMPASS\_OFS2\_Y: Compass2 offsets in milligauss on the Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to compass2\'s y\-axis values to compensate for metal in the frame


+-------------+-----------+------------+------------+
| Calibration | Increment | Range      | Units      |
+=============+===========+============+============+
| 1           | 1         | -400 - 400 | milligauss |
+-------------+-----------+------------+------------+




.. _COMPASS_OFS2_Z:

COMPASS\_OFS2\_Z: Compass2 offsets in milligauss on the Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to compass2\'s z\-axis values to compensate for metal in the frame


+-----------+------------+------------+
| Increment | Range      | Units      |
+===========+============+============+
| 1         | -400 - 400 | milligauss |
+-----------+------------+------------+




.. _COMPASS_MOT2_X:

COMPASS\_MOT2\_X: Motor interference compensation to compass2 for body frame X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass2\'s x\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-------------+-----------+--------------+-----------------------+
| Calibration | Increment | Range        | Units                 |
+=============+===========+==============+=======================+
| 1           | 1         | -1000 - 1000 | milligauss per ampere |
+-------------+-----------+--------------+-----------------------+




.. _COMPASS_MOT2_Y:

COMPASS\_MOT2\_Y: Motor interference compensation to compass2 for body frame Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass2\'s y\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-------------+-----------+--------------+-----------------------+
| Calibration | Increment | Range        | Units                 |
+=============+===========+==============+=======================+
| 1           | 1         | -1000 - 1000 | milligauss per ampere |
+-------------+-----------+--------------+-----------------------+




.. _COMPASS_MOT2_Z:

COMPASS\_MOT2\_Z: Motor interference compensation to compass2 for body frame Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass2\'s z\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-----------+--------------+-----------------------+
| Increment | Range        | Units                 |
+===========+==============+=======================+
| 1         | -1000 - 1000 | milligauss per ampere |
+-----------+--------------+-----------------------+




.. _COMPASS_OFS3_X:

COMPASS\_OFS3\_X: Compass3 offsets in milligauss on the X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to compass3\'s x\-axis values to compensate for metal in the frame


+-------------+-----------+------------+------------+
| Calibration | Increment | Range      | Units      |
+=============+===========+============+============+
| 1           | 1         | -400 - 400 | milligauss |
+-------------+-----------+------------+------------+




.. _COMPASS_OFS3_Y:

COMPASS\_OFS3\_Y: Compass3 offsets in milligauss on the Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to compass3\'s y\-axis values to compensate for metal in the frame


+-------------+-----------+------------+------------+
| Calibration | Increment | Range      | Units      |
+=============+===========+============+============+
| 1           | 1         | -400 - 400 | milligauss |
+-------------+-----------+------------+------------+




.. _COMPASS_OFS3_Z:

COMPASS\_OFS3\_Z: Compass3 offsets in milligauss on the Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset to be added to compass3\'s z\-axis values to compensate for metal in the frame


+-----------+------------+------------+
| Increment | Range      | Units      |
+===========+============+============+
| 1         | -400 - 400 | milligauss |
+-----------+------------+------------+




.. _COMPASS_MOT3_X:

COMPASS\_MOT3\_X: Motor interference compensation to compass3 for body frame X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass3\'s x\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-------------+-----------+--------------+-----------------------+
| Calibration | Increment | Range        | Units                 |
+=============+===========+==============+=======================+
| 1           | 1         | -1000 - 1000 | milligauss per ampere |
+-------------+-----------+--------------+-----------------------+




.. _COMPASS_MOT3_Y:

COMPASS\_MOT3\_Y: Motor interference compensation to compass3 for body frame Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass3\'s y\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-------------+-----------+--------------+-----------------------+
| Calibration | Increment | Range        | Units                 |
+=============+===========+==============+=======================+
| 1           | 1         | -1000 - 1000 | milligauss per ampere |
+-------------+-----------+--------------+-----------------------+




.. _COMPASS_MOT3_Z:

COMPASS\_MOT3\_Z: Motor interference compensation to compass3 for body frame Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass3\'s z\-axis values to compensate for motor interference \(Offset per Amp or at Full Throttle\)


+-----------+--------------+-----------------------+
| Increment | Range        | Units                 |
+===========+==============+=======================+
| 1         | -1000 - 1000 | milligauss per ampere |
+-----------+--------------+-----------------------+




.. _COMPASS_DEV_ID:

COMPASS\_DEV\_ID: Compass device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compass device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_DEV_ID2:

COMPASS\_DEV\_ID2: Compass2 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Second compass\'s device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_DEV_ID3:

COMPASS\_DEV\_ID3: Compass3 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Third compass\'s device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_USE2:

COMPASS\_USE2: Compass2 used for yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the secondary compass for determining heading\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_ORIENT2:

COMPASS\_ORIENT2: Compass2 orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The orientation of a second external compass relative to the vehicle frame\. This value will be ignored unless this compass is set as an external compass\. When set correctly in the northern hemisphere\, pointing the nose and right side down should increase the MagX and MagY values respectively\. Rolling the vehicle upside down should decrease the MagZ value\. For southern hemisphere\, switch increase and decrease\. NOTE\: For internal compasses\, AHRS\_ORIENT is used\.


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | None                 | |
| +-------+----------------------+ |
| | 1     | Yaw45                | |
| +-------+----------------------+ |
| | 2     | Yaw90                | |
| +-------+----------------------+ |
| | 3     | Yaw135               | |
| +-------+----------------------+ |
| | 4     | Yaw180               | |
| +-------+----------------------+ |
| | 5     | Yaw225               | |
| +-------+----------------------+ |
| | 6     | Yaw270               | |
| +-------+----------------------+ |
| | 7     | Yaw315               | |
| +-------+----------------------+ |
| | 8     | Roll180              | |
| +-------+----------------------+ |
| | 9     | Roll180Yaw45         | |
| +-------+----------------------+ |
| | 10    | Roll180Yaw90         | |
| +-------+----------------------+ |
| | 11    | Roll180Yaw135        | |
| +-------+----------------------+ |
| | 12    | Pitch180             | |
| +-------+----------------------+ |
| | 13    | Roll180Yaw225        | |
| +-------+----------------------+ |
| | 14    | Roll180Yaw270        | |
| +-------+----------------------+ |
| | 15    | Roll180Yaw315        | |
| +-------+----------------------+ |
| | 16    | Roll90               | |
| +-------+----------------------+ |
| | 17    | Roll90Yaw45          | |
| +-------+----------------------+ |
| | 18    | Roll90Yaw90          | |
| +-------+----------------------+ |
| | 19    | Roll90Yaw135         | |
| +-------+----------------------+ |
| | 20    | Roll270              | |
| +-------+----------------------+ |
| | 21    | Roll270Yaw45         | |
| +-------+----------------------+ |
| | 22    | Roll270Yaw90         | |
| +-------+----------------------+ |
| | 23    | Roll270Yaw135        | |
| +-------+----------------------+ |
| | 24    | Pitch90              | |
| +-------+----------------------+ |
| | 25    | Pitch270             | |
| +-------+----------------------+ |
| | 26    | Pitch180Yaw90        | |
| +-------+----------------------+ |
| | 27    | Pitch180Yaw270       | |
| +-------+----------------------+ |
| | 28    | Roll90Pitch90        | |
| +-------+----------------------+ |
| | 29    | Roll180Pitch90       | |
| +-------+----------------------+ |
| | 30    | Roll270Pitch90       | |
| +-------+----------------------+ |
| | 31    | Roll90Pitch180       | |
| +-------+----------------------+ |
| | 32    | Roll270Pitch180      | |
| +-------+----------------------+ |
| | 33    | Roll90Pitch270       | |
| +-------+----------------------+ |
| | 34    | Roll180Pitch270      | |
| +-------+----------------------+ |
| | 35    | Roll270Pitch270      | |
| +-------+----------------------+ |
| | 36    | Roll90Pitch180Yaw90  | |
| +-------+----------------------+ |
| | 37    | Roll90Yaw270         | |
| +-------+----------------------+ |
| | 38    | Yaw293Pitch68Roll180 | |
| +-------+----------------------+ |
| | 39    | Pitch315             | |
| +-------+----------------------+ |
| | 40    | Roll90Pitch315       | |
| +-------+----------------------+ |
| | 42    | Roll45               | |
| +-------+----------------------+ |
| | 43    | Roll315              | |
| +-------+----------------------+ |
| | 100   | Custom               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _COMPASS_EXTERN2:

COMPASS\_EXTERN2: Compass2 is attached via an external cable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Configure second compass so it is attached externally\. This is auto\-detected on most boards\. If set to 0 or 1 then auto\-detection by bus connection can override the value\. If set to 2 then auto\-detection will be disabled\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | 0     | Internal       | |
| +-------+----------------+ |
| | 1     | External       | |
| +-------+----------------+ |
| | 2     | ForcedExternal | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _COMPASS_USE3:

COMPASS\_USE3: Compass3 used for yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the tertiary compass for determining heading\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_ORIENT3:

COMPASS\_ORIENT3: Compass3 orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The orientation of a third external compass relative to the vehicle frame\. This value will be ignored unless this compass is set as an external compass\. When set correctly in the northern hemisphere\, pointing the nose and right side down should increase the MagX and MagY values respectively\. Rolling the vehicle upside down should decrease the MagZ value\. For southern hemisphere\, switch increase and decrease\. NOTE\: For internal compasses\, AHRS\_ORIENT is used\.


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | None                 | |
| +-------+----------------------+ |
| | 1     | Yaw45                | |
| +-------+----------------------+ |
| | 2     | Yaw90                | |
| +-------+----------------------+ |
| | 3     | Yaw135               | |
| +-------+----------------------+ |
| | 4     | Yaw180               | |
| +-------+----------------------+ |
| | 5     | Yaw225               | |
| +-------+----------------------+ |
| | 6     | Yaw270               | |
| +-------+----------------------+ |
| | 7     | Yaw315               | |
| +-------+----------------------+ |
| | 8     | Roll180              | |
| +-------+----------------------+ |
| | 9     | Roll180Yaw45         | |
| +-------+----------------------+ |
| | 10    | Roll180Yaw90         | |
| +-------+----------------------+ |
| | 11    | Roll180Yaw135        | |
| +-------+----------------------+ |
| | 12    | Pitch180             | |
| +-------+----------------------+ |
| | 13    | Roll180Yaw225        | |
| +-------+----------------------+ |
| | 14    | Roll180Yaw270        | |
| +-------+----------------------+ |
| | 15    | Roll180Yaw315        | |
| +-------+----------------------+ |
| | 16    | Roll90               | |
| +-------+----------------------+ |
| | 17    | Roll90Yaw45          | |
| +-------+----------------------+ |
| | 18    | Roll90Yaw90          | |
| +-------+----------------------+ |
| | 19    | Roll90Yaw135         | |
| +-------+----------------------+ |
| | 20    | Roll270              | |
| +-------+----------------------+ |
| | 21    | Roll270Yaw45         | |
| +-------+----------------------+ |
| | 22    | Roll270Yaw90         | |
| +-------+----------------------+ |
| | 23    | Roll270Yaw135        | |
| +-------+----------------------+ |
| | 24    | Pitch90              | |
| +-------+----------------------+ |
| | 25    | Pitch270             | |
| +-------+----------------------+ |
| | 26    | Pitch180Yaw90        | |
| +-------+----------------------+ |
| | 27    | Pitch180Yaw270       | |
| +-------+----------------------+ |
| | 28    | Roll90Pitch90        | |
| +-------+----------------------+ |
| | 29    | Roll180Pitch90       | |
| +-------+----------------------+ |
| | 30    | Roll270Pitch90       | |
| +-------+----------------------+ |
| | 31    | Roll90Pitch180       | |
| +-------+----------------------+ |
| | 32    | Roll270Pitch180      | |
| +-------+----------------------+ |
| | 33    | Roll90Pitch270       | |
| +-------+----------------------+ |
| | 34    | Roll180Pitch270      | |
| +-------+----------------------+ |
| | 35    | Roll270Pitch270      | |
| +-------+----------------------+ |
| | 36    | Roll90Pitch180Yaw90  | |
| +-------+----------------------+ |
| | 37    | Roll90Yaw270         | |
| +-------+----------------------+ |
| | 38    | Yaw293Pitch68Roll180 | |
| +-------+----------------------+ |
| | 39    | Pitch315             | |
| +-------+----------------------+ |
| | 40    | Roll90Pitch315       | |
| +-------+----------------------+ |
| | 42    | Roll45               | |
| +-------+----------------------+ |
| | 43    | Roll315              | |
| +-------+----------------------+ |
| | 100   | Custom               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _COMPASS_EXTERN3:

COMPASS\_EXTERN3: Compass3 is attached via an external cable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Configure third compass so it is attached externally\. This is auto\-detected on most boards\. If set to 0 or 1 then auto\-detection by bus connection can override the value\. If set to 2 then auto\-detection will be disabled\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | 0     | Internal       | |
| +-------+----------------+ |
| | 1     | External       | |
| +-------+----------------+ |
| | 2     | ForcedExternal | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _COMPASS_DIA_X:

COMPASS\_DIA\_X: Compass soft\-iron diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_X in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_DIA_Y:

COMPASS\_DIA\_Y: Compass soft\-iron diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_Y in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_DIA_Z:

COMPASS\_DIA\_Z: Compass soft\-iron diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_Z in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI_X:

COMPASS\_ODI\_X: Compass soft\-iron off\-diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_X in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_ODI_Y:

COMPASS\_ODI\_Y: Compass soft\-iron off\-diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_Y in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_ODI_Z:

COMPASS\_ODI\_Z: Compass soft\-iron off\-diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_Z in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA2_X:

COMPASS\_DIA2\_X: Compass2 soft\-iron diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_X in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_DIA2_Y:

COMPASS\_DIA2\_Y: Compass2 soft\-iron diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_Y in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_DIA2_Z:

COMPASS\_DIA2\_Z: Compass2 soft\-iron diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_Z in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI2_X:

COMPASS\_ODI2\_X: Compass2 soft\-iron off\-diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_X in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_ODI2_Y:

COMPASS\_ODI2\_Y: Compass2 soft\-iron off\-diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_Y in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_ODI2_Z:

COMPASS\_ODI2\_Z: Compass2 soft\-iron off\-diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_Z in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA3_X:

COMPASS\_DIA3\_X: Compass3 soft\-iron diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_X in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_DIA3_Y:

COMPASS\_DIA3\_Y: Compass3 soft\-iron diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_Y in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_DIA3_Z:

COMPASS\_DIA3\_Z: Compass3 soft\-iron diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DIA\_Z in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI3_X:

COMPASS\_ODI3\_X: Compass3 soft\-iron off\-diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_X in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_ODI3_Y:

COMPASS\_ODI3\_Y: Compass3 soft\-iron off\-diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_Y in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


+-------------+
| Calibration |
+=============+
| 1           |
+-------------+




.. _COMPASS_ODI3_Z:

COMPASS\_ODI3\_Z: Compass3 soft\-iron off\-diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ODI\_Z in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_CAL_FIT:

COMPASS\_CAL\_FIT: Compass calibration fitness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the fitness level required for a successful compass calibration\. A lower value makes for a stricter fit \(less likely to pass\)\. This is the value used for the primary magnetometer\. Other magnetometers get double the value\.


+-----------+--------+-------------------------+
| Increment | Range  | Values                  |
+===========+========+=========================+
| 0.1       | 4 - 32 | +-------+-------------+ |
|           |        | | Value | Meaning     | |
|           |        | +=======+=============+ |
|           |        | | 4     | Very Strict | |
|           |        | +-------+-------------+ |
|           |        | | 8     | Strict      | |
|           |        | +-------+-------------+ |
|           |        | | 16    | Default     | |
|           |        | +-------+-------------+ |
|           |        | | 32    | Relaxed     | |
|           |        | +-------+-------------+ |
|           |        |                         |
+-----------+--------+-------------------------+




.. _COMPASS_OFFS_MAX:

COMPASS\_OFFS\_MAX: Compass maximum offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the maximum allowed compass offset in calibration and arming checks


+-----------+------------+
| Increment | Range      |
+===========+============+
| 1         | 500 - 3000 |
+-----------+------------+




.. _COMPASS_TYPEMASK:

COMPASS\_TYPEMASK: Compass disable driver type mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is a bitmask of driver types to disable\. If a driver type is set in this mask then that driver will not try to find a sensor at startup


+------------------------+
| Bitmask                |
+========================+
| +-----+--------------+ |
| | Bit | Meaning      | |
| +=====+==============+ |
| | 0   | HMC5883      | |
| +-----+--------------+ |
| | 1   | LSM303D      | |
| +-----+--------------+ |
| | 2   | AK8963       | |
| +-----+--------------+ |
| | 3   | BMM150       | |
| +-----+--------------+ |
| | 4   | LSM9DS1      | |
| +-----+--------------+ |
| | 5   | LIS3MDL      | |
| +-----+--------------+ |
| | 6   | AK09916      | |
| +-----+--------------+ |
| | 7   | IST8310      | |
| +-----+--------------+ |
| | 8   | ICM20948     | |
| +-----+--------------+ |
| | 9   | MMC3416      | |
| +-----+--------------+ |
| | 11  | DroneCAN     | |
| +-----+--------------+ |
| | 12  | QMC5883      | |
| +-----+--------------+ |
| | 14  | MAG3110      | |
| +-----+--------------+ |
| | 15  | IST8308      | |
| +-----+--------------+ |
| | 16  | RM3100       | |
| +-----+--------------+ |
| | 17  | MSP          | |
| +-----+--------------+ |
| | 18  | ExternalAHRS | |
| +-----+--------------+ |
|                        |
+------------------------+




.. _COMPASS_FLTR_RNG:

COMPASS\_FLTR\_RNG: Range in which sample is accepted
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the range around the average value that new samples must be within to be accepted\. This can help reduce the impact of noise on sensors that are on long I2C cables\. The value is a percentage from the average value\. A value of zero disables this filter\.


+-----------+---------+---------+
| Increment | Range   | Units   |
+===========+=========+=========+
| 1         | 0 - 100 | percent |
+-----------+---------+---------+




.. _COMPASS_AUTO_ROT:

COMPASS\_AUTO\_ROT: Automatically check orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When enabled this will automatically check the orientation of compasses on successful completion of compass calibration\. If set to 2 then external compasses will have their orientation automatically corrected\.


+----------------------------------------------------------------+
| Values                                                         |
+================================================================+
| +-------+----------------------------------------------------+ |
| | Value | Meaning                                            | |
| +=======+====================================================+ |
| | 0     | Disabled                                           | |
| +-------+----------------------------------------------------+ |
| | 1     | CheckOnly                                          | |
| +-------+----------------------------------------------------+ |
| | 2     | CheckAndFix                                        | |
| +-------+----------------------------------------------------+ |
| | 3     | use same tolerance to auto rotate 45 deg rotations | |
| +-------+----------------------------------------------------+ |
|                                                                |
+----------------------------------------------------------------+




.. _COMPASS_PRIO1_ID:

COMPASS\_PRIO1\_ID: Compass device id with 1st order priority
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Compass device id with 1st order priority\, set automatically if 0\. Reboot required after change\.


.. _COMPASS_PRIO2_ID:

COMPASS\_PRIO2\_ID: Compass device id with 2nd order priority
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Compass device id with 2nd order priority\, set automatically if 0\. Reboot required after change\.


.. _COMPASS_PRIO3_ID:

COMPASS\_PRIO3\_ID: Compass device id with 3rd order priority
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Compass device id with 3rd order priority\, set automatically if 0\. Reboot required after change\.


.. _COMPASS_ENABLE:

COMPASS\_ENABLE: Enable Compass
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Setting this to Enabled\(1\) will enable the compass\. Setting this to Disabled\(0\) will disable the compass\. Note that this is separate from COMPASS\_USE\. This will enable the low level senor\, and will enable logging of magnetometer data\. To use the compass for navigation you must also set COMPASS\_USE to 1\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_SCALE:

COMPASS\_SCALE: Compass1 scale factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor for first compass to compensate for sensor scaling errors\. If this is 0 then no scaling is done


+---------+
| Range   |
+=========+
| 0 - 1.3 |
+---------+




.. _COMPASS_SCALE2:

COMPASS\_SCALE2: Compass2 scale factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor for 2nd compass to compensate for sensor scaling errors\. If this is 0 then no scaling is done


+---------+
| Range   |
+=========+
| 0 - 1.3 |
+---------+




.. _COMPASS_SCALE3:

COMPASS\_SCALE3: Compass3 scale factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor for 3rd compass to compensate for sensor scaling errors\. If this is 0 then no scaling is done


+---------+
| Range   |
+=========+
| 0 - 1.3 |
+---------+




.. _COMPASS_OPTIONS:

COMPASS\_OPTIONS: Compass options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets options to change the behaviour of the compass


+-------------------------+
| Bitmask                 |
+=========================+
| +-----+---------------+ |
| | Bit | Meaning       | |
| +=====+===============+ |
| | 0   | CalRequireGPS | |
| +-----+---------------+ |
|                         |
+-------------------------+




.. _COMPASS_DEV_ID4:

COMPASS\_DEV\_ID4: Compass4 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra 4th compass\'s device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_DEV_ID5:

COMPASS\_DEV\_ID5: Compass5 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra 5th compass\'s device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_DEV_ID6:

COMPASS\_DEV\_ID6: Compass6 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra 6th compass\'s device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_DEV_ID7:

COMPASS\_DEV\_ID7: Compass7 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra 7th compass\'s device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_DEV_ID8:

COMPASS\_DEV\_ID8: Compass8 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra 8th compass\'s device id\.  Automatically detected\, do not set manually


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _COMPASS_CUS_ROLL:

COMPASS\_CUS\_ROLL: Custom orientation roll offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Compass mounting position roll offset\. Positive values \= roll right\, negative values \= roll left\. This parameter is only used when COMPASS\_ORIENT\/2\/3 is set to CUSTOM\.


+-----------+------------+---------+
| Increment | Range      | Units   |
+===========+============+=========+
| 1         | -180 - 180 | degrees |
+-----------+------------+---------+




.. _COMPASS_CUS_PIT:

COMPASS\_CUS\_PIT: Custom orientation pitch offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Compass mounting position pitch offset\. Positive values \= pitch up\, negative values \= pitch down\. This parameter is only used when COMPASS\_ORIENT\/2\/3 is set to CUSTOM\.


+-----------+------------+---------+
| Increment | Range      | Units   |
+===========+============+=========+
| 1         | -180 - 180 | degrees |
+-----------+------------+---------+




.. _COMPASS_CUS_YAW:

COMPASS\_CUS\_YAW: Custom orientation yaw offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Compass mounting position yaw offset\. Positive values \= yaw right\, negative values \= yaw left\. This parameter is only used when COMPASS\_ORIENT\/2\/3 is set to CUSTOM\.


+-----------+------------+---------+
| Increment | Range      | Units   |
+===========+============+=========+
| 1         | -180 - 180 | degrees |
+-----------+------------+---------+





.. _parameters_COMPASS_PMOT:

COMPASS\_PMOT Parameters
------------------------


.. _COMPASS_PMOT_EN:

COMPASS\_PMOT\_EN: per\-motor compass correction enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables per\-motor compass corrections


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_PMOT_EXP:

COMPASS\_PMOT\_EXP: per\-motor exponential correction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the exponential correction for the power output of the motor for per\-motor compass correction


+-----------+-------+
| Increment | Range |
+===========+=======+
| 0.01      | 0 - 2 |
+-----------+-------+




.. _COMPASS_PMOT1_X:

COMPASS\_PMOT1\_X: Compass per\-motor1 X
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for X axis of motor1


.. _COMPASS_PMOT1_Y:

COMPASS\_PMOT1\_Y: Compass per\-motor1 Y
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Y axis of motor1


.. _COMPASS_PMOT1_Z:

COMPASS\_PMOT1\_Z: Compass per\-motor1 Z
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Z axis of motor1


.. _COMPASS_PMOT2_X:

COMPASS\_PMOT2\_X: Compass per\-motor2 X
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for X axis of motor2


.. _COMPASS_PMOT2_Y:

COMPASS\_PMOT2\_Y: Compass per\-motor2 Y
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Y axis of motor2


.. _COMPASS_PMOT2_Z:

COMPASS\_PMOT2\_Z: Compass per\-motor2 Z
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Z axis of motor2


.. _COMPASS_PMOT3_X:

COMPASS\_PMOT3\_X: Compass per\-motor3 X
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for X axis of motor3


.. _COMPASS_PMOT3_Y:

COMPASS\_PMOT3\_Y: Compass per\-motor3 Y
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Y axis of motor3


.. _COMPASS_PMOT3_Z:

COMPASS\_PMOT3\_Z: Compass per\-motor3 Z
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Z axis of motor3


.. _COMPASS_PMOT4_X:

COMPASS\_PMOT4\_X: Compass per\-motor4 X
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for X axis of motor4


.. _COMPASS_PMOT4_Y:

COMPASS\_PMOT4\_Y: Compass per\-motor4 Y
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Y axis of motor4


.. _COMPASS_PMOT4_Z:

COMPASS\_PMOT4\_Z: Compass per\-motor4 Z
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Compensation for Z axis of motor4



.. _parameters_EAHRS:

EAHRS Parameters
----------------


.. _EAHRS_TYPE:

EAHRS\_TYPE: AHRS type
~~~~~~~~~~~~~~~~~~~~~~


Type of AHRS device


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | None      | |
| +-------+-----------+ |
| | 1     | VectorNav | |
| +-------+-----------+ |
| | 2     | LORD      | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _EAHRS_RATE:

EAHRS\_RATE: AHRS data rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Requested rate for AHRS device


+-------+
| Units |
+=======+
| hertz |
+-------+





.. _parameters_EFI:

EFI Parameters
--------------


.. _EFI_TYPE:

EFI\_TYPE: EFI communication type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

What method of communication is used for EFI \#1


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | None         | |
| +-------+--------------+ |
| | 1     | Serial-MS    | |
| +-------+--------------+ |
| | 2     | NWPMU        | |
| +-------+--------------+ |
| | 3     | Serial-Lutan | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _EFI_COEF1:

EFI\_COEF1: EFI Calibration Coefficient 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to calibrate fuel flow for MS protocol \(Slope\)


+-------+
| Range |
+=======+
| 0 - 1 |
+-------+




.. _EFI_COEF2:

EFI\_COEF2: EFI Calibration Coefficient 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to calibrate fuel flow for MS protocol \(Offset\)


+--------+
| Range  |
+========+
| 0 - 10 |
+--------+





.. _parameters_FFT_:

FFT\_ Parameters
----------------


.. _FFT_ENABLE:

FFT\_ENABLE: Enable
~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enable Gyro FFT analyser


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _FFT_MINHZ:

FFT\_MINHZ: Minimum Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Lower bound of FFT frequency detection in Hz\. On larger vehicles the minimum motor frequency is likely to be significantly lower than for smaller vehicles\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 20 - 400 | hertz |
+----------+-------+




.. _FFT_MAXHZ:

FFT\_MAXHZ: Maximum Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Upper bound of FFT frequency detection in Hz\. On smaller vehicles the maximum motor frequency is likely to be significantly higher than for larger vehicles\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 20 - 495 | hertz |
+----------+-------+




.. _FFT_SAMPLE_MODE:

FFT\_SAMPLE\_MODE: Sample Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Sampling mode \(and therefore rate\)\. 0\: Gyro rate sampling\, 1\: Fast loop rate sampling\, 2\: Fast loop rate \/ 2 sampling\, 3\: Fast loop rate \/ 3 sampling\. Takes effect on reboot\.


+-------+
| Range |
+=======+
| 0 - 4 |
+-------+




.. _FFT_WINDOW_SIZE:

FFT\_WINDOW\_SIZE: FFT window size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Size of window to be used in FFT calculations\. Takes effect on reboot\. Must be a power of 2 and between 32 and 512\. Larger windows give greater frequency resolution but poorer time resolution\, consume more CPU time and may not be appropriate for all vehicles\. Time and frequency resolution are given by the sample\-rate \/ window\-size\. Windows of 256 are only really recommended for F7 class boards\, windows of 512 or more H7 class\.


+-----------+
| Range     |
+===========+
| 32 - 1024 |
+-----------+




.. _FFT_WINDOW_OLAP:

FFT\_WINDOW\_OLAP: FFT window overlap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Percentage of window to be overlapped before another frame is process\. Takes effect on reboot\. A good default is 50\% overlap\. Higher overlap results in more processed frames but not necessarily more temporal resolution\. Lower overlap results in lost information at the frame edges\.


+---------+
| Range   |
+=========+
| 0 - 0.9 |
+---------+




.. _FFT_FREQ_HOVER:

FFT\_FREQ\_HOVER: FFT learned hover frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The learned hover noise frequency


+---------+
| Range   |
+=========+
| 0 - 250 |
+---------+




.. _FFT_THR_REF:

FFT\_THR\_REF: FFT learned thrust reference
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT learned thrust reference for the hover frequency and FFT minimum frequency\.


+------------+
| Range      |
+============+
| 0.01 - 0.9 |
+------------+




.. _FFT_SNR_REF:

FFT\_SNR\_REF: FFT SNR reference threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT SNR reference threshold in dB at which a signal is determined to be present\.


+-------------+
| Range       |
+=============+
| 0.0 - 100.0 |
+-------------+




.. _FFT_ATT_REF:

FFT\_ATT\_REF: FFT attenuation for bandwidth calculation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT attenuation level in dB for bandwidth calculation and peak detection\. The bandwidth is calculated by comparing peak power output with the attenuated version\. The default of 15 has shown to be a good compromise in both simulations and real flight\.


+---------+
| Range   |
+=========+
| 0 - 100 |
+---------+




.. _FFT_BW_HOVER:

FFT\_BW\_HOVER: FFT learned bandwidth at hover
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT learned bandwidth at hover for the attenuation frequencies\.


+---------+
| Range   |
+=========+
| 0 - 200 |
+---------+




.. _FFT_HMNC_FIT:

FFT\_HMNC\_FIT: FFT harmonic fit frequency threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

FFT harmonic fit frequency threshold percentage at which a signal of the appropriate frequency is determined to be the harmonic of another\. Signals that have a harmonic relationship that varies at most by this percentage are considered harmonics of each other for the purpose of selecting the harmonic notch frequency\. If a match is found then the lower frequency harmonic is always used as the basis for the dynamic harmonic notch\. A value of zero completely disables harmonic matching\.


+---------+
| Range   |
+=========+
| 0 - 100 |
+---------+




.. _FFT_HMNC_PEAK:

FFT\_HMNC\_PEAK: FFT harmonic peak target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The FFT harmonic peak target that should be returned by FTN1\.PkAvg\. The resulting value will be used by the harmonic notch if configured to track the FFT frequency\. By default the appropriate peak is auto\-detected based on the harmonic fit between peaks and the energy\-weighted average frequency on roll on pitch is used\. Setting this to 1 will always target the highest energy peak\. Setting this to 2 will target the highest energy peak that is lower in frequency than the highest energy peak\. Setting this to 3 will target the highest energy peak that is higher in frequency than the highest energy peak\. Setting this to 4 will target the highest energy peak on the roll axis only and only the roll frequency will be used \(some vehicles have a much more pronounced peak on roll\)\. Setting this to 5 will target the highest energy peak on the pitch axis only and only the pitch frequency will be used \(some vehicles have a much more pronounced peak on roll\)\.


+--------------------------------------+
| Values                               |
+======================================+
| +-------+--------------------------+ |
| | Value | Meaning                  | |
| +=======+==========================+ |
| | 0     | Auto                     | |
| +-------+--------------------------+ |
| | 1     | Center Frequency         | |
| +-------+--------------------------+ |
| | 2     | Lower-Shoulder Frequency | |
| +-------+--------------------------+ |
| | 3     | Upper-Shoulder Frequency | |
| +-------+--------------------------+ |
| | 4     | Roll-Axis                | |
| +-------+--------------------------+ |
| | 5     | Pitch-Axis               | |
| +-------+--------------------------+ |
|                                      |
+--------------------------------------+





.. _parameters_FRSKY_:

FRSKY\_ Parameters
------------------


.. _FRSKY_UPLINK_ID:

FRSKY\_UPLINK\_ID: Uplink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the uplink sensor id \(SPort only\)


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | -1    | Disable | |
| +-------+---------+ |
| | 7     | 7       | |
| +-------+---------+ |
| | 8     | 8       | |
| +-------+---------+ |
| | 9     | 9       | |
| +-------+---------+ |
| | 10    | 10      | |
| +-------+---------+ |
| | 11    | 11      | |
| +-------+---------+ |
| | 12    | 12      | |
| +-------+---------+ |
| | 13    | 13      | |
| +-------+---------+ |
| | 14    | 14      | |
| +-------+---------+ |
| | 15    | 15      | |
| +-------+---------+ |
| | 16    | 16      | |
| +-------+---------+ |
| | 17    | 17      | |
| +-------+---------+ |
| | 18    | 18      | |
| +-------+---------+ |
| | 19    | 19      | |
| +-------+---------+ |
| | 20    | 20      | |
| +-------+---------+ |
| | 21    | 21      | |
| +-------+---------+ |
| | 22    | 22      | |
| +-------+---------+ |
| | 23    | 23      | |
| +-------+---------+ |
| | 24    | 24      | |
| +-------+---------+ |
| | 25    | 25      | |
| +-------+---------+ |
| | 26    | 26      | |
| +-------+---------+ |
|                     |
+---------------------+




.. _FRSKY_DNLINK1_ID:

FRSKY\_DNLINK1\_ID: First downlink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the first extra downlink sensor id \(SPort only\)


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | -1    | Disable | |
| +-------+---------+ |
| | 7     | 7       | |
| +-------+---------+ |
| | 8     | 8       | |
| +-------+---------+ |
| | 9     | 9       | |
| +-------+---------+ |
| | 10    | 10      | |
| +-------+---------+ |
| | 11    | 11      | |
| +-------+---------+ |
| | 12    | 12      | |
| +-------+---------+ |
| | 13    | 13      | |
| +-------+---------+ |
| | 14    | 14      | |
| +-------+---------+ |
| | 15    | 15      | |
| +-------+---------+ |
| | 16    | 16      | |
| +-------+---------+ |
| | 17    | 17      | |
| +-------+---------+ |
| | 18    | 18      | |
| +-------+---------+ |
| | 19    | 19      | |
| +-------+---------+ |
| | 20    | 20      | |
| +-------+---------+ |
| | 21    | 21      | |
| +-------+---------+ |
| | 22    | 22      | |
| +-------+---------+ |
| | 23    | 23      | |
| +-------+---------+ |
| | 24    | 24      | |
| +-------+---------+ |
| | 25    | 25      | |
| +-------+---------+ |
| | 26    | 26      | |
| +-------+---------+ |
|                     |
+---------------------+




.. _FRSKY_DNLINK2_ID:

FRSKY\_DNLINK2\_ID: Second downlink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the second extra downlink sensor id \(SPort only\)


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | -1    | Disable | |
| +-------+---------+ |
| | 7     | 7       | |
| +-------+---------+ |
| | 8     | 8       | |
| +-------+---------+ |
| | 9     | 9       | |
| +-------+---------+ |
| | 10    | 10      | |
| +-------+---------+ |
| | 11    | 11      | |
| +-------+---------+ |
| | 12    | 12      | |
| +-------+---------+ |
| | 13    | 13      | |
| +-------+---------+ |
| | 14    | 14      | |
| +-------+---------+ |
| | 15    | 15      | |
| +-------+---------+ |
| | 16    | 16      | |
| +-------+---------+ |
| | 17    | 17      | |
| +-------+---------+ |
| | 18    | 18      | |
| +-------+---------+ |
| | 19    | 19      | |
| +-------+---------+ |
| | 20    | 20      | |
| +-------+---------+ |
| | 21    | 21      | |
| +-------+---------+ |
| | 22    | 22      | |
| +-------+---------+ |
| | 23    | 23      | |
| +-------+---------+ |
| | 24    | 24      | |
| +-------+---------+ |
| | 25    | 25      | |
| +-------+---------+ |
| | 26    | 26      | |
| +-------+---------+ |
|                     |
+---------------------+




.. _FRSKY_DNLINK_ID:

FRSKY\_DNLINK\_ID: Default downlink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the default downlink sensor id \(SPort only\)


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | -1    | Disable | |
| +-------+---------+ |
| | 7     | 7       | |
| +-------+---------+ |
| | 8     | 8       | |
| +-------+---------+ |
| | 9     | 9       | |
| +-------+---------+ |
| | 10    | 10      | |
| +-------+---------+ |
| | 11    | 11      | |
| +-------+---------+ |
| | 12    | 12      | |
| +-------+---------+ |
| | 13    | 13      | |
| +-------+---------+ |
| | 14    | 14      | |
| +-------+---------+ |
| | 15    | 15      | |
| +-------+---------+ |
| | 16    | 16      | |
| +-------+---------+ |
| | 17    | 17      | |
| +-------+---------+ |
| | 18    | 18      | |
| +-------+---------+ |
| | 19    | 19      | |
| +-------+---------+ |
| | 20    | 20      | |
| +-------+---------+ |
| | 21    | 21      | |
| +-------+---------+ |
| | 22    | 22      | |
| +-------+---------+ |
| | 23    | 23      | |
| +-------+---------+ |
| | 24    | 24      | |
| +-------+---------+ |
| | 25    | 25      | |
| +-------+---------+ |
| | 26    | 26      | |
| +-------+---------+ |
| | 27    | 27      | |
| +-------+---------+ |
|                     |
+---------------------+




.. _FRSKY_OPTIONS:

FRSKY\_OPTIONS: FRSky Telemetry Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


A bitmask to set some FRSky Telemetry specific options


+----------------------------------------+
| Bitmask                                |
+========================================+
| +-----+------------------------------+ |
| | Bit | Meaning                      | |
| +=====+==============================+ |
| | 0   | EnableAirspeedAndGroundspeed | |
| +-----+------------------------------+ |
|                                        |
+----------------------------------------+





.. _parameters_GEN_:

GEN\_ Parameters
----------------


.. _GEN_TYPE:

GEN\_TYPE: Generator type
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Generator type


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | Disabled               | |
| +-------+------------------------+ |
| | 1     | IE 650w 800w Fuel Cell | |
| +-------+------------------------+ |
| | 2     | IE 2.4kW Fuel Cell     | |
| +-------+------------------------+ |
| | 3     | Richenpower            | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+





.. _parameters_GPS:

GPS Parameters
--------------


.. _GPS_TYPE:

GPS\_TYPE: 1st GPS type
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

GPS type of 1st GPS


+-------------------------------------------+
| Values                                    |
+===========================================+
| +-------+-------------------------------+ |
| | Value | Meaning                       | |
| +=======+===============================+ |
| | 0     | None                          | |
| +-------+-------------------------------+ |
| | 1     | AUTO                          | |
| +-------+-------------------------------+ |
| | 2     | uBlox                         | |
| +-------+-------------------------------+ |
| | 5     | NMEA                          | |
| +-------+-------------------------------+ |
| | 6     | SiRF                          | |
| +-------+-------------------------------+ |
| | 7     | HIL                           | |
| +-------+-------------------------------+ |
| | 8     | SwiftNav                      | |
| +-------+-------------------------------+ |
| | 9     | DroneCAN                      | |
| +-------+-------------------------------+ |
| | 10    | SBF                           | |
| +-------+-------------------------------+ |
| | 11    | GSOF                          | |
| +-------+-------------------------------+ |
| | 13    | ERB                           | |
| +-------+-------------------------------+ |
| | 14    | MAV                           | |
| +-------+-------------------------------+ |
| | 15    | NOVA                          | |
| +-------+-------------------------------+ |
| | 16    | HemisphereNMEA                | |
| +-------+-------------------------------+ |
| | 17    | uBlox-MovingBaseline-Base     | |
| +-------+-------------------------------+ |
| | 18    | uBlox-MovingBaseline-Rover    | |
| +-------+-------------------------------+ |
| | 19    | MSP                           | |
| +-------+-------------------------------+ |
| | 20    | AllyStar                      | |
| +-------+-------------------------------+ |
| | 21    | ExternalAHRS                  | |
| +-------+-------------------------------+ |
| | 22    | DroneCAN-MovingBaseline-Base  | |
| +-------+-------------------------------+ |
| | 23    | DroneCAN-MovingBaseline-Rover | |
| +-------+-------------------------------+ |
|                                           |
+-------------------------------------------+




.. _GPS_TYPE2:

GPS\_TYPE2: 2nd GPS type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

GPS type of 2nd GPS


+-------------------------------------------+
| Values                                    |
+===========================================+
| +-------+-------------------------------+ |
| | Value | Meaning                       | |
| +=======+===============================+ |
| | 0     | None                          | |
| +-------+-------------------------------+ |
| | 1     | AUTO                          | |
| +-------+-------------------------------+ |
| | 2     | uBlox                         | |
| +-------+-------------------------------+ |
| | 5     | NMEA                          | |
| +-------+-------------------------------+ |
| | 6     | SiRF                          | |
| +-------+-------------------------------+ |
| | 7     | HIL                           | |
| +-------+-------------------------------+ |
| | 8     | SwiftNav                      | |
| +-------+-------------------------------+ |
| | 9     | DroneCAN                      | |
| +-------+-------------------------------+ |
| | 10    | SBF                           | |
| +-------+-------------------------------+ |
| | 11    | GSOF                          | |
| +-------+-------------------------------+ |
| | 13    | ERB                           | |
| +-------+-------------------------------+ |
| | 14    | MAV                           | |
| +-------+-------------------------------+ |
| | 15    | NOVA                          | |
| +-------+-------------------------------+ |
| | 16    | HemisphereNMEA                | |
| +-------+-------------------------------+ |
| | 17    | uBlox-MovingBaseline-Base     | |
| +-------+-------------------------------+ |
| | 18    | uBlox-MovingBaseline-Rover    | |
| +-------+-------------------------------+ |
| | 19    | MSP                           | |
| +-------+-------------------------------+ |
| | 20    | AllyStar                      | |
| +-------+-------------------------------+ |
| | 21    | ExternalAHRS                  | |
| +-------+-------------------------------+ |
| | 22    | DroneCAN-MovingBaseline-Base  | |
| +-------+-------------------------------+ |
| | 23    | DroneCAN-MovingBaseline-Rover | |
| +-------+-------------------------------+ |
|                                           |
+-------------------------------------------+




.. _GPS_NAVFILTER:

GPS\_NAVFILTER: Navigation filter setting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Navigation filter engine setting


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Portable   | |
| +-------+------------+ |
| | 2     | Stationary | |
| +-------+------------+ |
| | 3     | Pedestrian | |
| +-------+------------+ |
| | 4     | Automotive | |
| +-------+------------+ |
| | 5     | Sea        | |
| +-------+------------+ |
| | 6     | Airborne1G | |
| +-------+------------+ |
| | 7     | Airborne2G | |
| +-------+------------+ |
| | 8     | Airborne4G | |
| +-------+------------+ |
|                        |
+------------------------+




.. _GPS_AUTO_SWITCH:

GPS\_AUTO\_SWITCH: Automatic Switchover Setting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Automatic switchover to GPS reporting best lock\, 1\:UseBest selects the GPS with highest status\, if both are equal the GPS with highest satellite count is used 4\:Use primary if 3D fix or better\, will revert to \'UseBest\' behaviour if 3D fix is lost on primary


+---------------------------------------------+
| Values                                      |
+=============================================+
| +-------+---------------------------------+ |
| | Value | Meaning                         | |
| +=======+=================================+ |
| | 0     | Use primary                     | |
| +-------+---------------------------------+ |
| | 1     | UseBest                         | |
| +-------+---------------------------------+ |
| | 2     | Blend                           | |
| +-------+---------------------------------+ |
| | 4     | Use primary if 3D fix or better | |
| +-------+---------------------------------+ |
|                                             |
+---------------------------------------------+




.. _GPS_MIN_DGPS:

GPS\_MIN\_DGPS: Minimum Lock Type Accepted for DGPS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Any        | |
| +-------+------------+ |
| | 50    | FloatRTK   | |
| +-------+------------+ |
| | 100   | IntegerRTK | |
| +-------+------------+ |
|                        |
+------------------------+




.. _GPS_SBAS_MODE:

GPS\_SBAS\_MODE: SBAS Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the SBAS \(satellite based augmentation system\) mode if available on this GPS\. If set to 2 then the SBAS mode is not changed in the GPS\. Otherwise the GPS will be reconfigured to enable\/disable SBAS\. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
| | 2     | NoChange | |
| +-------+----------+ |
|                      |
+----------------------+




.. _GPS_MIN_ELEV:

GPS\_MIN\_ELEV: Minimum elevation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the minimum elevation of satellites above the horizon for them to be used for navigation\. Setting this to \-100 leaves the minimum elevation set to the GPS modules default\.


+-----------+---------+
| Range     | Units   |
+===========+=========+
| -100 - 90 | degrees |
+-----------+---------+




.. _GPS_INJECT_TO:

GPS\_INJECT\_TO: Destination for GPS\_INJECT\_DATA MAVLink packets
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The GGS can send raw serial packets to inject data to multiple GPSes\.


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | send to first GPS | |
| +-------+-------------------+ |
| | 1     | send to 2nd GPS   | |
| +-------+-------------------+ |
| | 127   | send to all       | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _GPS_SBP_LOGMASK:

GPS\_SBP\_LOGMASK: Swift Binary Protocol Logging Mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Masked with the SBP msg\_type field to determine whether SBR1\/SBR2 data is logged


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None (0x0000)          | |
| +-------+------------------------+ |
| | -1    | All (0xFFFF)           | |
| +-------+------------------------+ |
| | -256  | External only (0xFF00) | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _GPS_RAW_DATA:

GPS\_RAW\_DATA: Raw data logging
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Handles logging raw data\; on uBlox chips that support raw data this will log RXM messages into logger\; on Septentrio this will log on the equipment\'s SD card and when set to 2\, the autopilot will try to stop logging after disarming and restart after arming


+------------------------------------------------------+
| Values                                               |
+======================================================+
| +-------+------------------------------------------+ |
| | Value | Meaning                                  | |
| +=======+==========================================+ |
| | 0     | Ignore                                   | |
| +-------+------------------------------------------+ |
| | 1     | Always log                               | |
| +-------+------------------------------------------+ |
| | 2     | Stop logging when disarmed (SBF only)    | |
| +-------+------------------------------------------+ |
| | 5     | Only log every five samples (uBlox only) | |
| +-------+------------------------------------------+ |
|                                                      |
+------------------------------------------------------+




.. _GPS_GNSS_MODE:

GPS\_GNSS\_MODE: GNSS system configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the first GPS \(all unchecked or zero to leave GPS as configured\)


+-------------------+
| Bitmask           |
+===================+
| +-----+---------+ |
| | Bit | Meaning | |
| +=====+=========+ |
| | 0   | GPS     | |
| +-----+---------+ |
| | 1   | SBAS    | |
| +-----+---------+ |
| | 2   | Galileo | |
| +-----+---------+ |
| | 3   | Beidou  | |
| +-----+---------+ |
| | 4   | IMES    | |
| +-----+---------+ |
| | 5   | QZSS    | |
| +-----+---------+ |
| | 6   | GLONASS | |
| +-----+---------+ |
|                   |
+-------------------+




.. _GPS_SAVE_CFG:

GPS\_SAVE\_CFG: Save GPS configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Determines whether the configuration for this GPS should be written to non\-volatile memory on the GPS\. Currently working for UBlox 6 series and above\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | 0     | Do not save config    | |
| +-------+-----------------------+ |
| | 1     | Save config           | |
| +-------+-----------------------+ |
| | 2     | Save only when needed | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _GPS_GNSS_MODE2:

GPS\_GNSS\_MODE2: GNSS system configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the second GPS \(all unchecked or zero to leave GPS as configured\)


+-------------------+
| Bitmask           |
+===================+
| +-----+---------+ |
| | Bit | Meaning | |
| +=====+=========+ |
| | 0   | GPS     | |
| +-----+---------+ |
| | 1   | SBAS    | |
| +-----+---------+ |
| | 2   | Galileo | |
| +-----+---------+ |
| | 3   | Beidou  | |
| +-----+---------+ |
| | 4   | IMES    | |
| +-----+---------+ |
| | 5   | QZSS    | |
| +-----+---------+ |
| | 6   | GLONASS | |
| +-----+---------+ |
|                   |
+-------------------+




.. _GPS_AUTO_CONFIG:

GPS\_AUTO\_CONFIG: Automatic GPS configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls if the autopilot should automatically configure the GPS based on the parameters and default settings


+------------------------------------------------------------------+
| Values                                                           |
+==================================================================+
| +-------+------------------------------------------------------+ |
| | Value | Meaning                                              | |
| +=======+======================================================+ |
| | 0     | Disables automatic configuration                     | |
| +-------+------------------------------------------------------+ |
| | 1     | Enable automatic configuration for Serial GPSes only | |
| +-------+------------------------------------------------------+ |
| | 2     | Enable automatic configuration for DroneCAN as well  | |
| +-------+------------------------------------------------------+ |
|                                                                  |
+------------------------------------------------------------------+




.. _GPS_RATE_MS:

GPS\_RATE\_MS: GPS update rate in milliseconds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update\. Lowering below 5Hz\(default\) is not allowed\. Raising the rate above 5Hz usually provides little benefit and for some GPS \(eg Ublox M9N\) can severely impact performance\.


+----------+--------------+---------------------+
| Range    | Units        | Values              |
+==========+==============+=====================+
| 50 - 200 | milliseconds | +-------+---------+ |
|          |              | | Value | Meaning | |
|          |              | +=======+=========+ |
|          |              | | 100   | 10Hz    | |
|          |              | +-------+---------+ |
|          |              | | 125   | 8Hz     | |
|          |              | +-------+---------+ |
|          |              | | 200   | 5Hz     | |
|          |              | +-------+---------+ |
|          |              |                     |
+----------+--------------+---------------------+




.. _GPS_RATE_MS2:

GPS\_RATE\_MS2: GPS 2 update rate in milliseconds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update\. Lowering below 5Hz\(default\) is not allowed\. Raising the rate above 5Hz usually provides little benefit and for some GPS \(eg Ublox M9N\) can severely impact performance\.


+----------+--------------+---------------------+
| Range    | Units        | Values              |
+==========+==============+=====================+
| 50 - 200 | milliseconds | +-------+---------+ |
|          |              | | Value | Meaning | |
|          |              | +=======+=========+ |
|          |              | | 100   | 10Hz    | |
|          |              | +-------+---------+ |
|          |              | | 125   | 8Hz     | |
|          |              | +-------+---------+ |
|          |              | | 200   | 5Hz     | |
|          |              | +-------+---------+ |
|          |              |                     |
+----------+--------------+---------------------+




.. _GPS_POS1_X:

GPS\_POS1\_X: Antenna X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the first GPS antenna in body frame\. Positive X is forward of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_POS1_Y:

GPS\_POS1\_Y: Antenna Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the first GPS antenna in body frame\. Positive Y is to the right of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_POS1_Z:

GPS\_POS1\_Z: Antenna Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the first GPS antenna in body frame\. Positive Z is down from the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_POS2_X:

GPS\_POS2\_X: Antenna X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the second GPS antenna in body frame\. Positive X is forward of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_POS2_Y:

GPS\_POS2\_Y: Antenna Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the second GPS antenna in body frame\. Positive Y is to the right of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_POS2_Z:

GPS\_POS2\_Z: Antenna Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the second GPS antenna in body frame\. Positive Z is down from the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_DELAY_MS:

GPS\_DELAY\_MS: GPS delay in milliseconds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Controls the amount of GPS  measurement delay that the autopilot compensates for\. Set to zero to use the default delay for the detected GPS type\.


+---------+--------------+
| Range   | Units        |
+=========+==============+
| 0 - 250 | milliseconds |
+---------+--------------+




.. _GPS_DELAY_MS2:

GPS\_DELAY\_MS2: GPS 2 delay in milliseconds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Controls the amount of GPS  measurement delay that the autopilot compensates for\. Set to zero to use the default delay for the detected GPS type\.


+---------+--------------+
| Range   | Units        |
+=========+==============+
| 0 - 250 | milliseconds |
+---------+--------------+




.. _GPS_BLEND_MASK:

GPS\_BLEND\_MASK: Multi GPS Blending Mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Determines which of the accuracy measures Horizontal position\, Vertical Position and Speed are used to calculate the weighting on each GPS receiver when soft switching has been selected by setting GPS\_AUTO\_SWITCH to 2\(Blend\)


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | Horiz Pos | |
| +-----+-----------+ |
| | 1   | Vert Pos  | |
| +-----+-----------+ |
| | 2   | Speed     | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _GPS_BLEND_TC:

GPS\_BLEND\_TC: Blending time constant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls the slowest time constant applied to the calculation of GPS position and height offsets used to adjust different GPS receivers for steady state position differences\.


+------------+---------+
| Range      | Units   |
+============+=========+
| 5.0 - 30.0 | seconds |
+------------+---------+




.. _GPS_DRV_OPTIONS:

GPS\_DRV\_OPTIONS: driver options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Additional backend specific options


+----------------------------------------------------------------+
| Bitmask                                                        |
+================================================================+
| +-----+------------------------------------------------------+ |
| | Bit | Meaning                                              | |
| +=====+======================================================+ |
| | 0   | Use UART2 for moving baseline on ublox               | |
| +-----+------------------------------------------------------+ |
| | 1   | Use base station for GPS yaw on SBF                  | |
| +-----+------------------------------------------------------+ |
| | 2   | Use baudrate 115200                                  | |
| +-----+------------------------------------------------------+ |
| | 3   | Use dedicated CAN port b/w GPSes for moving baseline | |
| +-----+------------------------------------------------------+ |
|                                                                |
+----------------------------------------------------------------+




.. _GPS_COM_PORT:

GPS\_COM\_PORT: GPS physical COM port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

The physical COM port on the connected device\, currently only applies to SBF GPS


+-----------+--------+
| Increment | Range  |
+===========+========+
| 1         | 0 - 10 |
+-----------+--------+




.. _GPS_COM_PORT2:

GPS\_COM\_PORT2: GPS physical COM port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

The physical COM port on the connected device\, currently only applies to SBF GPS


+-----------+--------+
| Increment | Range  |
+===========+========+
| 1         | 0 - 10 |
+-----------+--------+




.. _GPS_PRIMARY:

GPS\_PRIMARY: Primary GPS
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This GPS will be used when GPS\_AUTO\_SWITCH is 0 and used preferentially with GPS\_AUTO\_SWITCH \= 4\.


+-----------+-----------------------+
| Increment | Values                |
+===========+=======================+
| 1         | +-------+-----------+ |
|           | | Value | Meaning   | |
|           | +=======+===========+ |
|           | | 0     | FirstGPS  | |
|           | +-------+-----------+ |
|           | | 1     | SecondGPS | |
|           | +-------+-----------+ |
|           |                       |
+-----------+-----------------------+




.. _GPS_CAN_NODEID1:

GPS\_CAN\_NODEID1: GPS Node ID 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

GPS Node id for discovered first\.


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _GPS_CAN_NODEID2:

GPS\_CAN\_NODEID2: GPS Node ID 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

GPS Node id for discovered second\.


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _GPS1_CAN_OVRIDE:

GPS1\_CAN\_OVRIDE: First DroneCAN GPS NODE ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

GPS Node id for first GPS\. If 0 the gps will be automatically selected on first come basis\.


.. _GPS2_CAN_OVRIDE:

GPS2\_CAN\_OVRIDE: Second DroneCAN GPS NODE ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

GPS Node id for second GPS\. If 0 the gps will be automatically selected on first come basis\.



.. _parameters_GPS_MB1_:

GPS\_MB1\_ Parameters
---------------------


.. _GPS_MB1_TYPE:

GPS\_MB1\_TYPE: Moving base type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Controls the type of moving base used if using moving base\.


+------------------------------------------------+
| Values                                         |
+================================================+
| +-------+------------------------------------+ |
| | Value | Meaning                            | |
| +=======+====================================+ |
| | 0     | Relative to alternate GPS instance | |
| +-------+------------------------------------+ |
| | 1     | RelativeToCustomBase               | |
| +-------+------------------------------------+ |
|                                                |
+------------------------------------------------+




.. _GPS_MB1_OFS_X:

GPS\_MB1\_OFS\_X: Base antenna X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the base GPS antenna in body frame\. Positive X is forward of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_MB1_OFS_Y:

GPS\_MB1\_OFS\_Y: Base antenna Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the base GPS antenna in body frame\. Positive Y is to the right of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_MB1_OFS_Z:

GPS\_MB1\_OFS\_Z: Base antenna Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the base GPS antenna in body frame\. Positive Z is down from the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+





.. _parameters_GPS_MB2_:

GPS\_MB2\_ Parameters
---------------------


.. _GPS_MB2_TYPE:

GPS\_MB2\_TYPE: Moving base type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Controls the type of moving base used if using moving base\.


+------------------------------------------------+
| Values                                         |
+================================================+
| +-------+------------------------------------+ |
| | Value | Meaning                            | |
| +=======+====================================+ |
| | 0     | Relative to alternate GPS instance | |
| +-------+------------------------------------+ |
| | 1     | RelativeToCustomBase               | |
| +-------+------------------------------------+ |
|                                                |
+------------------------------------------------+




.. _GPS_MB2_OFS_X:

GPS\_MB2\_OFS\_X: Base antenna X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the base GPS antenna in body frame\. Positive X is forward of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_MB2_OFS_Y:

GPS\_MB2\_OFS\_Y: Base antenna Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the base GPS antenna in body frame\. Positive Y is to the right of the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _GPS_MB2_OFS_Z:

GPS\_MB2\_OFS\_Z: Base antenna Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the base GPS antenna in body frame\. Positive Z is down from the origin\. Use antenna phase centroid location if provided by the manufacturer\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+





.. _parameters_LOG:

LOG Parameters
--------------


.. _LOG_BACKEND_TYPE:

LOG\_BACKEND\_TYPE: AP\_Logger Backend Storage type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Bitmap of what Logger backend types to enable\. Block\-based logging is available on SITL and boards with dataflash chips\. Multiple backends can be selected\.


+-------------------+
| Bitmask           |
+===================+
| +-----+---------+ |
| | Bit | Meaning | |
| +=====+=========+ |
| | 0   | File    | |
| +-----+---------+ |
| | 1   | MAVLink | |
| +-----+---------+ |
| | 2   | Block   | |
| +-----+---------+ |
|                   |
+-------------------+




.. _LOG_FILE_BUFSIZE:

LOG\_FILE\_BUFSIZE: Maximum AP\_Logger File and Block Backend buffer size \(in kilobytes\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The File and Block backends use a buffer to store data before writing to the block device\.  Raising this value may reduce \"gaps\" in your SD card logging\.  This buffer size may be reduced depending on available memory\.  PixHawk requires at least 4 kilobytes\.  Maximum value available here is 64 kilobytes\.


.. _LOG_DISARMED:

LOG\_DISARMED: Enable logging while disarmed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If LOG\_DISARMED is set to 1 then logging will be enabled while disarmed\. This can make for very large logfiles but can help a lot when tracking down startup issues


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _LOG_REPLAY:

LOG\_REPLAY: Enable logging of information needed for Replay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If LOG\_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter\. It is suggested that you also raise LOG\_FILE\_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _LOG_FILE_DSRMROT:

LOG\_FILE\_DSRMROT: Stop logging to current file on disarm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When set\, the current log file is closed when the vehicle is disarmed\.  If LOG\_DISARMED is set then a fresh log will be opened\. Applies to the File and Block logging backends\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _LOG_MAV_BUFSIZE:

LOG\_MAV\_BUFSIZE: Maximum AP\_Logger MAVLink Backend buffer size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum amount of memory to allocate to AP\_Logger\-over\-mavlink


+-----------+
| Units     |
+===========+
| kilobytes |
+-----------+




.. _LOG_FILE_TIMEOUT:

LOG\_FILE\_TIMEOUT: Timeout before giving up on file writes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls the amount of time before failing writes to a log file cause the file to be closed and logging stopped\.


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _LOG_FILE_MB_FREE:

LOG\_FILE\_MB\_FREE: Old logs on the SD card will be deleted to maintain this amount of free space
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Set this such that the free space is larger than your largest typical flight log


+-----------+----------+
| Range     | Units    |
+===========+==========+
| 10 - 1000 | megabyte |
+-----------+----------+




.. _LOG_FILE_RATEMAX:

LOG\_FILE\_RATEMAX: Maximum logging rate for file backend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum rate that streaming log messages will be logged to the file backend\. A value of zero means that rate limiting is disabled\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 0 - 1000 | hertz |
+----------+-------+




.. _LOG_MAV_RATEMAX:

LOG\_MAV\_RATEMAX: Maximum logging rate for mavlink backend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum rate that streaming log messages will be logged to the mavlink backend\. A value of zero means that rate limiting is disabled\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 0 - 1000 | hertz |
+----------+-------+




.. _LOG_BLK_RATEMAX:

LOG\_BLK\_RATEMAX: Maximum logging rate for block backend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum rate that streaming log messages will be logged to the mavlink backend\. A value of zero means that rate limiting is disabled\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 0 - 1000 | hertz |
+----------+-------+





.. _parameters_MSP:

MSP Parameters
--------------


.. _MSP_OSD_NCELLS:

MSP\_OSD\_NCELLS: Cell count override
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Used for average cell voltage calculation


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Auto    | |
| +-------+---------+ |
| | 1     | 1       | |
| +-------+---------+ |
| | 2     | 2       | |
| +-------+---------+ |
| | 3     | 3       | |
| +-------+---------+ |
| | 4     | 4       | |
| +-------+---------+ |
| | 5     | 5       | |
| +-------+---------+ |
| | 6     | 6       | |
| +-------+---------+ |
| | 7     | 7       | |
| +-------+---------+ |
| | 8     | 8       | |
| +-------+---------+ |
| | 9     | 9       | |
| +-------+---------+ |
| | 10    | 10      | |
| +-------+---------+ |
| | 11    | 11      | |
| +-------+---------+ |
| | 12    | 12      | |
| +-------+---------+ |
| | 13    | 13      | |
| +-------+---------+ |
| | 14    | 14      | |
| +-------+---------+ |
|                     |
+---------------------+




.. _MSP_OPTIONS:

MSP\_OPTIONS: MSP OSD Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


A bitmask to set some MSP specific options


+---------------------------------+
| Bitmask                         |
+=================================+
| +-----+-----------------------+ |
| | Bit | Meaning               | |
| +=====+=======================+ |
| | 0   | EnableTelemetryMode   | |
| +-----+-----------------------+ |
| | 1   | DisableDJIWorkarounds | |
| +-----+-----------------------+ |
| | 2   | EnableBTFLFonts       | |
| +-----+-----------------------+ |
|                                 |
+---------------------------------+





.. _parameters_NTF_:

NTF\_ Parameters
----------------


.. _NTF_LED_BRIGHT:

NTF\_LED\_BRIGHT: LED Brightness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Select the RGB LED brightness level\. When USB is connected brightness will never be higher than low regardless of the setting\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Off     | |
| +-------+---------+ |
| | 1     | Low     | |
| +-------+---------+ |
| | 2     | Medium  | |
| +-------+---------+ |
| | 3     | High    | |
| +-------+---------+ |
|                     |
+---------------------+




.. _NTF_BUZZ_TYPES:

NTF\_BUZZ\_TYPES: Buzzer Driver Types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls what types of Buzzer will be enabled


+---------------------------+
| Bitmask                   |
+===========================+
| +-----+-----------------+ |
| | Bit | Meaning         | |
| +=====+=================+ |
| | 0   | Built-in buzzer | |
| +-----+-----------------+ |
| | 1   | DShot           | |
| +-----+-----------------+ |
| | 2   | DroneCAN        | |
| +-----+-----------------+ |
|                           |
+---------------------------+




.. _NTF_LED_OVERRIDE:

NTF\_LED\_OVERRIDE: Specifies colour source for the RGBLed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Specifies the source for the colours and brightness for the LED\.  OutbackChallenge conforms to the MedicalExpress \(https\:\/\/uavchallenge\.org\/medical\-express\/\) rules\, essentially \"Green\" is disarmed \(safe\-to\-approach\)\, \"Red\" is armed \(not safe\-to\-approach\)\. Traffic light is a simplified color set\, red when armed\, yellow when the safety switch is not surpressing outputs \(but disarmed\)\, and green when outputs are surpressed and disarmed\, the LED will blink faster if disarmed and failing arming checks\.


+-----------------------------------------+
| Values                                  |
+=========================================+
| +-------+-----------------------------+ |
| | Value | Meaning                     | |
| +=======+=============================+ |
| | 0     | Standard                    | |
| +-------+-----------------------------+ |
| | 1     | MAVLink/Scripting/AP_Periph | |
| +-------+-----------------------------+ |
| | 2     | OutbackChallenge            | |
| +-------+-----------------------------+ |
| | 3     | TrafficLight                | |
| +-------+-----------------------------+ |
|                                         |
+-----------------------------------------+




.. _NTF_DISPLAY_TYPE:

NTF\_DISPLAY\_TYPE: Type of on\-board I2C display
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets up the type of on\-board I2C display\. Disabled by default\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | ssd1306 | |
| +-------+---------+ |
| | 2     | sh1106  | |
| +-------+---------+ |
| | 10    | SITL    | |
| +-------+---------+ |
|                     |
+---------------------+




.. _NTF_OREO_THEME:

NTF\_OREO\_THEME: OreoLED Theme
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable\/Disable Solo Oreo LED driver\, 0 to disable\, 1 for Aircraft theme\, 2 for Rover theme


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Aircraft | |
| +-------+----------+ |
| | 2     | Rover    | |
| +-------+----------+ |
|                      |
+----------------------+




.. _NTF_BUZZ_PIN:

NTF\_BUZZ\_PIN: Buzzer pin
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enables to connect active buzzer to arbitrary pin\. Requires 3\-pin buzzer or additional MOSFET\!


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
|                      |
+----------------------+




.. _NTF_LED_TYPES:

NTF\_LED\_TYPES: LED Driver Types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls what types of LEDs will be enabled


+-------------------------------+
| Bitmask                       |
+===============================+
| +-----+---------------------+ |
| | Bit | Meaning             | |
| +=====+=====================+ |
| | 0   | Built-in LED        | |
| +-----+---------------------+ |
| | 1   | Internal ToshibaLED | |
| +-----+---------------------+ |
| | 2   | External ToshibaLED | |
| +-----+---------------------+ |
| | 3   | External PCA9685    | |
| +-----+---------------------+ |
| | 4   | Oreo LED            | |
| +-----+---------------------+ |
| | 5   | DroneCAN            | |
| +-----+---------------------+ |
| | 6   | NCP5623 External    | |
| +-----+---------------------+ |
| | 7   | NCP5623 Internal    | |
| +-----+---------------------+ |
| | 8   | NeoPixel            | |
| +-----+---------------------+ |
| | 9   | ProfiLED            | |
| +-----+---------------------+ |
| | 10  | Scripting           | |
| +-----+---------------------+ |
| | 11  | DShot               | |
| +-----+---------------------+ |
| | 12  | ProfiLED_SPI        | |
| +-----+---------------------+ |
|                               |
+-------------------------------+




.. _NTF_BUZZ_ON_LVL:

NTF\_BUZZ\_ON\_LVL: Buzzer\-on pin logic level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Specifies pin level that indicates buzzer should play


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | LowIsOn  | |
| +-------+----------+ |
| | 1     | HighIsOn | |
| +-------+----------+ |
|                      |
+----------------------+




.. _NTF_BUZZ_VOLUME:

NTF\_BUZZ\_VOLUME: Buzzer volume
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control the volume of the buzzer


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 100 | percent |
+---------+---------+




.. _NTF_LED_LEN:

NTF\_LED\_LEN: Serial LED String Length
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

The number of Serial LED\'s to use for notifications \(NeoPixel\'s and ProfiLED\)


+--------+
| Range  |
+========+
| 1 - 32 |
+--------+





.. _parameters_OUT:

OUT Parameters
--------------


.. _OUT_RATE:

OUT\_RATE: Servo default output rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the default output rate in Hz for all outputs\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 25 - 400 | hertz |
+----------+-------+




.. _OUT_DSHOT_RATE:

OUT\_DSHOT\_RATE: Servo DShot output rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the DShot output rate for all outputs as a multiple of the loop rate\. 0 sets the output rate to be fixed at 1Khz for low loop rates\. This value should never be set below 500Hz\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | 1Khz                | |
| +-------+---------------------+ |
| | 1     | loop-rate           | |
| +-------+---------------------+ |
| | 2     | double loop-rate    | |
| +-------+---------------------+ |
| | 3     | triple loop-rate    | |
| +-------+---------------------+ |
| | 4     | quadruple loop rate | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _OUT_DSHOT_ESC:

OUT\_DSHOT\_ESC: Servo DShot ESC type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the DShot ESC type for all outputs\. The ESC type affects the range of DShot commands available\. None means that no dshot commands will be executed\.


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | BLHeli32/BLHeli_S/Kiss | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _OUT_GPIO_MASK:

OUT\_GPIO\_MASK: Servo GPIO mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This sets a bitmask of outputs which will be available as GPIOs\. Any auxillary output with either the function set to \-1 or with the corresponding bit set in this mask will be available for use as a GPIO pin


+--------------------+
| Bitmask            |
+====================+
| +-----+----------+ |
| | Bit | Meaning  | |
| +=====+==========+ |
| | 0   | Servo 1  | |
| +-----+----------+ |
| | 1   | Servo 2  | |
| +-----+----------+ |
| | 2   | Servo 3  | |
| +-----+----------+ |
| | 3   | Servo 4  | |
| +-----+----------+ |
| | 4   | Servo 5  | |
| +-----+----------+ |
| | 5   | Servo 6  | |
| +-----+----------+ |
| | 6   | Servo 7  | |
| +-----+----------+ |
| | 7   | Servo 8  | |
| +-----+----------+ |
| | 8   | Servo 9  | |
| +-----+----------+ |
| | 9   | Servo 10 | |
| +-----+----------+ |
| | 10  | Servo 11 | |
| +-----+----------+ |
| | 11  | Servo 12 | |
| +-----+----------+ |
| | 12  | Servo 13 | |
| +-----+----------+ |
| | 13  | Servo 14 | |
| +-----+----------+ |
| | 14  | Servo 15 | |
| +-----+----------+ |
| | 15  | Servo 16 | |
| +-----+----------+ |
|                    |
+--------------------+





.. _parameters_OUT10_:

OUT10\_ Parameters
------------------


.. _OUT10_MIN:

OUT10\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT10_MAX:

OUT10\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT10_TRIM:

OUT10\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT10_REVERSED:

OUT10\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT10_FUNCTION:

OUT10\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT11_:

OUT11\_ Parameters
------------------


.. _OUT11_MIN:

OUT11\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT11_MAX:

OUT11\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT11_TRIM:

OUT11\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT11_REVERSED:

OUT11\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT11_FUNCTION:

OUT11\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT12_:

OUT12\_ Parameters
------------------


.. _OUT12_MIN:

OUT12\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT12_MAX:

OUT12\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT12_TRIM:

OUT12\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT12_REVERSED:

OUT12\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT12_FUNCTION:

OUT12\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT13_:

OUT13\_ Parameters
------------------


.. _OUT13_MIN:

OUT13\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT13_MAX:

OUT13\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT13_TRIM:

OUT13\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT13_REVERSED:

OUT13\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT13_FUNCTION:

OUT13\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT14_:

OUT14\_ Parameters
------------------


.. _OUT14_MIN:

OUT14\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT14_MAX:

OUT14\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT14_TRIM:

OUT14\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT14_REVERSED:

OUT14\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT14_FUNCTION:

OUT14\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT15_:

OUT15\_ Parameters
------------------


.. _OUT15_MIN:

OUT15\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT15_MAX:

OUT15\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT15_TRIM:

OUT15\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT15_REVERSED:

OUT15\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT15_FUNCTION:

OUT15\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT16_:

OUT16\_ Parameters
------------------


.. _OUT16_MIN:

OUT16\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT16_MAX:

OUT16\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT16_TRIM:

OUT16\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT16_REVERSED:

OUT16\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT16_FUNCTION:

OUT16\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT1_:

OUT1\_ Parameters
-----------------


.. _OUT1_MIN:

OUT1\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT1_MAX:

OUT1\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT1_TRIM:

OUT1\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT1_REVERSED:

OUT1\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT1_FUNCTION:

OUT1\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT2_:

OUT2\_ Parameters
-----------------


.. _OUT2_MIN:

OUT2\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT2_MAX:

OUT2\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT2_TRIM:

OUT2\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT2_REVERSED:

OUT2\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT2_FUNCTION:

OUT2\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT3_:

OUT3\_ Parameters
-----------------


.. _OUT3_MIN:

OUT3\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT3_MAX:

OUT3\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT3_TRIM:

OUT3\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT3_REVERSED:

OUT3\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT3_FUNCTION:

OUT3\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT4_:

OUT4\_ Parameters
-----------------


.. _OUT4_MIN:

OUT4\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT4_MAX:

OUT4\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT4_TRIM:

OUT4\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT4_REVERSED:

OUT4\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT4_FUNCTION:

OUT4\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT5_:

OUT5\_ Parameters
-----------------


.. _OUT5_MIN:

OUT5\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT5_MAX:

OUT5\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT5_TRIM:

OUT5\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT5_REVERSED:

OUT5\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT5_FUNCTION:

OUT5\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT6_:

OUT6\_ Parameters
-----------------


.. _OUT6_MIN:

OUT6\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT6_MAX:

OUT6\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT6_TRIM:

OUT6\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT6_REVERSED:

OUT6\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT6_FUNCTION:

OUT6\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT7_:

OUT7\_ Parameters
-----------------


.. _OUT7_MIN:

OUT7\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT7_MAX:

OUT7\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT7_TRIM:

OUT7\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT7_REVERSED:

OUT7\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT7_FUNCTION:

OUT7\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT8_:

OUT8\_ Parameters
-----------------


.. _OUT8_MIN:

OUT8\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT8_MAX:

OUT8\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT8_TRIM:

OUT8\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT8_REVERSED:

OUT8\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT8_FUNCTION:

OUT8\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT9_:

OUT9\_ Parameters
-----------------


.. _OUT9_MIN:

OUT9\_MIN: Minimum PWM
~~~~~~~~~~~~~~~~~~~~~~


minimum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 500 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT9_MAX:

OUT9\_MAX: Maximum PWM
~~~~~~~~~~~~~~~~~~~~~~


maximum PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT9_TRIM:

OUT9\_TRIM: Trim PWM
~~~~~~~~~~~~~~~~~~~~


Trim PWM pulse width in microseconds\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+-----------+------------+---------------------+
| Increment | Range      | Units               |
+===========+============+=====================+
| 1         | 800 - 2200 | PWM in microseconds |
+-----------+------------+---------------------+




.. _OUT9_REVERSED:

OUT9\_REVERSED: Servo reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse servo operation\. Set to 0 for normal operation\. Set to 1 to reverse this output channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Normal   | |
| +-------+----------+ |
| | 1     | Reversed | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT9_FUNCTION:

OUT9\_FUNCTION: Servo output function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Function assigned to this servo\. Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | GPIO                      | |
| +-------+---------------------------+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | RCPassThru                | |
| +-------+---------------------------+ |
| | 2     | Flap                      | |
| +-------+---------------------------+ |
| | 3     | FlapAuto                  | |
| +-------+---------------------------+ |
| | 4     | Aileron                   | |
| +-------+---------------------------+ |
| | 6     | MountPan                  | |
| +-------+---------------------------+ |
| | 7     | MountTilt                 | |
| +-------+---------------------------+ |
| | 8     | MountRoll                 | |
| +-------+---------------------------+ |
| | 9     | MountOpen                 | |
| +-------+---------------------------+ |
| | 10    | CameraTrigger             | |
| +-------+---------------------------+ |
| | 12    | Mount2Pan                 | |
| +-------+---------------------------+ |
| | 13    | Mount2Tilt                | |
| +-------+---------------------------+ |
| | 14    | Mount2Roll                | |
| +-------+---------------------------+ |
| | 15    | Mount2Open                | |
| +-------+---------------------------+ |
| | 16    | DifferentialSpoilerLeft1  | |
| +-------+---------------------------+ |
| | 17    | DifferentialSpoilerRight1 | |
| +-------+---------------------------+ |
| | 19    | Elevator                  | |
| +-------+---------------------------+ |
| | 21    | Rudder                    | |
| +-------+---------------------------+ |
| | 22    | SprayerPump               | |
| +-------+---------------------------+ |
| | 23    | SprayerSpinner            | |
| +-------+---------------------------+ |
| | 24    | FlaperonLeft              | |
| +-------+---------------------------+ |
| | 25    | FlaperonRight             | |
| +-------+---------------------------+ |
| | 26    | GroundSteering            | |
| +-------+---------------------------+ |
| | 27    | Parachute                 | |
| +-------+---------------------------+ |
| | 28    | Gripper                   | |
| +-------+---------------------------+ |
| | 29    | LandingGear               | |
| +-------+---------------------------+ |
| | 30    | EngineRunEnable           | |
| +-------+---------------------------+ |
| | 31    | HeliRSC                   | |
| +-------+---------------------------+ |
| | 32    | HeliTailRSC               | |
| +-------+---------------------------+ |
| | 33    | Motor1                    | |
| +-------+---------------------------+ |
| | 34    | Motor2                    | |
| +-------+---------------------------+ |
| | 35    | Motor3                    | |
| +-------+---------------------------+ |
| | 36    | Motor4                    | |
| +-------+---------------------------+ |
| | 37    | Motor5                    | |
| +-------+---------------------------+ |
| | 38    | Motor6                    | |
| +-------+---------------------------+ |
| | 39    | Motor7                    | |
| +-------+---------------------------+ |
| | 40    | Motor8                    | |
| +-------+---------------------------+ |
| | 41    | TiltMotorsFront           | |
| +-------+---------------------------+ |
| | 45    | TiltMotorsRear            | |
| +-------+---------------------------+ |
| | 46    | TiltMotorRearLeft         | |
| +-------+---------------------------+ |
| | 47    | TiltMotorRearRight        | |
| +-------+---------------------------+ |
| | 51    | RCIN1                     | |
| +-------+---------------------------+ |
| | 52    | RCIN2                     | |
| +-------+---------------------------+ |
| | 53    | RCIN3                     | |
| +-------+---------------------------+ |
| | 54    | RCIN4                     | |
| +-------+---------------------------+ |
| | 55    | RCIN5                     | |
| +-------+---------------------------+ |
| | 56    | RCIN6                     | |
| +-------+---------------------------+ |
| | 57    | RCIN7                     | |
| +-------+---------------------------+ |
| | 58    | RCIN8                     | |
| +-------+---------------------------+ |
| | 59    | RCIN9                     | |
| +-------+---------------------------+ |
| | 60    | RCIN10                    | |
| +-------+---------------------------+ |
| | 61    | RCIN11                    | |
| +-------+---------------------------+ |
| | 62    | RCIN12                    | |
| +-------+---------------------------+ |
| | 63    | RCIN13                    | |
| +-------+---------------------------+ |
| | 64    | RCIN14                    | |
| +-------+---------------------------+ |
| | 65    | RCIN15                    | |
| +-------+---------------------------+ |
| | 66    | RCIN16                    | |
| +-------+---------------------------+ |
| | 67    | Ignition                  | |
| +-------+---------------------------+ |
| | 69    | Starter                   | |
| +-------+---------------------------+ |
| | 70    | Throttle                  | |
| +-------+---------------------------+ |
| | 71    | TrackerYaw                | |
| +-------+---------------------------+ |
| | 72    | TrackerPitch              | |
| +-------+---------------------------+ |
| | 73    | ThrottleLeft              | |
| +-------+---------------------------+ |
| | 74    | ThrottleRight             | |
| +-------+---------------------------+ |
| | 75    | TiltMotorFrontLeft        | |
| +-------+---------------------------+ |
| | 76    | TiltMotorFrontRight       | |
| +-------+---------------------------+ |
| | 77    | ElevonLeft                | |
| +-------+---------------------------+ |
| | 78    | ElevonRight               | |
| +-------+---------------------------+ |
| | 79    | VTailLeft                 | |
| +-------+---------------------------+ |
| | 80    | VTailRight                | |
| +-------+---------------------------+ |
| | 81    | BoostThrottle             | |
| +-------+---------------------------+ |
| | 82    | Motor9                    | |
| +-------+---------------------------+ |
| | 83    | Motor10                   | |
| +-------+---------------------------+ |
| | 84    | Motor11                   | |
| +-------+---------------------------+ |
| | 85    | Motor12                   | |
| +-------+---------------------------+ |
| | 86    | DifferentialSpoilerLeft2  | |
| +-------+---------------------------+ |
| | 87    | DifferentialSpoilerRight2 | |
| +-------+---------------------------+ |
| | 88    | Winch                     | |
| +-------+---------------------------+ |
| | 89    | Main Sail                 | |
| +-------+---------------------------+ |
| | 90    | CameraISO                 | |
| +-------+---------------------------+ |
| | 91    | CameraAperture            | |
| +-------+---------------------------+ |
| | 92    | CameraFocus               | |
| +-------+---------------------------+ |
| | 93    | CameraShutterSpeed        | |
| +-------+---------------------------+ |
| | 94    | Script1                   | |
| +-------+---------------------------+ |
| | 95    | Script2                   | |
| +-------+---------------------------+ |
| | 96    | Script3                   | |
| +-------+---------------------------+ |
| | 97    | Script4                   | |
| +-------+---------------------------+ |
| | 98    | Script5                   | |
| +-------+---------------------------+ |
| | 99    | Script6                   | |
| +-------+---------------------------+ |
| | 100   | Script7                   | |
| +-------+---------------------------+ |
| | 101   | Script8                   | |
| +-------+---------------------------+ |
| | 102   | Script9                   | |
| +-------+---------------------------+ |
| | 103   | Script10                  | |
| +-------+---------------------------+ |
| | 104   | Script11                  | |
| +-------+---------------------------+ |
| | 105   | Script12                  | |
| +-------+---------------------------+ |
| | 106   | Script13                  | |
| +-------+---------------------------+ |
| | 107   | Script14                  | |
| +-------+---------------------------+ |
| | 108   | Script15                  | |
| +-------+---------------------------+ |
| | 109   | Script16                  | |
| +-------+---------------------------+ |
| | 120   | NeoPixel1                 | |
| +-------+---------------------------+ |
| | 121   | NeoPixel2                 | |
| +-------+---------------------------+ |
| | 122   | NeoPixel3                 | |
| +-------+---------------------------+ |
| | 123   | NeoPixel4                 | |
| +-------+---------------------------+ |
| | 124   | RateRoll                  | |
| +-------+---------------------------+ |
| | 125   | RatePitch                 | |
| +-------+---------------------------+ |
| | 126   | RateThrust                | |
| +-------+---------------------------+ |
| | 127   | RateYaw                   | |
| +-------+---------------------------+ |
| | 128   | WingSailElevator          | |
| +-------+---------------------------+ |
| | 129   | ProfiLED1                 | |
| +-------+---------------------------+ |
| | 130   | ProfiLED2                 | |
| +-------+---------------------------+ |
| | 131   | ProfiLED3                 | |
| +-------+---------------------------+ |
| | 132   | ProfiLEDClock             | |
| +-------+---------------------------+ |
| | 133   | Winch Clutch              | |
| +-------+---------------------------+ |
| | 134   | SERVOn_MIN                | |
| +-------+---------------------------+ |
| | 135   | SERVOn_TRIM               | |
| +-------+---------------------------+ |
| | 136   | SERVOn_MAX                | |
| +-------+---------------------------+ |
| | 137   | SailMastRotation          | |
| +-------+---------------------------+ |
| | 138   | Alarm                     | |
| +-------+---------------------------+ |
| | 139   | Alarm Inverted            | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+





.. _parameters_OUT_BLH_:

OUT\_BLH\_ Parameters
---------------------


.. _OUT_BLH_MASK:

OUT\_BLH\_MASK: BLHeli Channel Bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable of BLHeli pass\-thru servo protocol support to specific channels\. This mask is in addition to motors enabled using SERVO\_BLH\_AUTO \(if any\)


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | Channel1  | |
| +-----+-----------+ |
| | 1   | Channel2  | |
| +-----+-----------+ |
| | 2   | Channel3  | |
| +-----+-----------+ |
| | 3   | Channel4  | |
| +-----+-----------+ |
| | 4   | Channel5  | |
| +-----+-----------+ |
| | 5   | Channel6  | |
| +-----+-----------+ |
| | 6   | Channel7  | |
| +-----+-----------+ |
| | 7   | Channel8  | |
| +-----+-----------+ |
| | 8   | Channel9  | |
| +-----+-----------+ |
| | 9   | Channel10 | |
| +-----+-----------+ |
| | 10  | Channel11 | |
| +-----+-----------+ |
| | 11  | Channel12 | |
| +-----+-----------+ |
| | 12  | Channel13 | |
| +-----+-----------+ |
| | 13  | Channel14 | |
| +-----+-----------+ |
| | 14  | Channel15 | |
| +-----+-----------+ |
| | 15  | Channel16 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _OUT_BLH_AUTO:

OUT\_BLH\_AUTO: BLHeli pass\-thru auto\-enable for multicopter motors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If set to 1 this auto\-enables BLHeli pass\-thru support for all multicopter motors


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT_BLH_TEST:

OUT\_BLH\_TEST: BLHeli internal interface test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Setting SERVO\_BLH\_TEST to a motor number enables an internal test of the BLHeli ESC protocol to the corresponding ESC\. The debug output is displayed on the USB console\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Disabled   | |
| +-------+------------+ |
| | 1     | TestMotor1 | |
| +-------+------------+ |
| | 2     | TestMotor2 | |
| +-------+------------+ |
| | 3     | TestMotor3 | |
| +-------+------------+ |
| | 4     | TestMotor4 | |
| +-------+------------+ |
| | 5     | TestMotor5 | |
| +-------+------------+ |
| | 6     | TestMotor6 | |
| +-------+------------+ |
| | 7     | TestMotor7 | |
| +-------+------------+ |
| | 8     | TestMotor8 | |
| +-------+------------+ |
|                        |
+------------------------+




.. _OUT_BLH_TMOUT:

OUT\_BLH\_TMOUT: BLHeli protocol timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the inactivity timeout for the BLHeli protocol in seconds\. If no packets are received in this time normal MAVLink operations are resumed\. A value of 0 means no timeout


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 300 | seconds |
+---------+---------+




.. _OUT_BLH_TRATE:

OUT\_BLH\_TRATE: BLHeli telemetry rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the rate in Hz for requesting telemetry from ESCs\. It is the rate per ESC\. Setting to zero disables telemetry requests


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 500 | hertz |
+---------+-------+




.. _OUT_BLH_DEBUG:

OUT\_BLH\_DEBUG: BLHeli debug level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When set to 1 this enabled verbose debugging output over MAVLink when the blheli protocol is active\. This can be used to diagnose failures\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _OUT_BLH_OTYPE:

OUT\_BLH\_OTYPE: BLHeli output type override
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

When set to a non\-zero value this overrides the output type for the output channels given by SERVO\_BLH\_MASK\. This can be used to enable DShot on outputs that are not part of the multicopter motors group\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | None       | |
| +-------+------------+ |
| | 1     | OneShot    | |
| +-------+------------+ |
| | 2     | OneShot125 | |
| +-------+------------+ |
| | 3     | Brushed    | |
| +-------+------------+ |
| | 4     | DShot150   | |
| +-------+------------+ |
| | 5     | DShot300   | |
| +-------+------------+ |
| | 6     | DShot600   | |
| +-------+------------+ |
| | 7     | DShot1200  | |
| +-------+------------+ |
|                        |
+------------------------+




.. _OUT_BLH_PORT:

OUT\_BLH\_PORT: Control port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the mavlink channel to use for blheli pass\-thru\. The channel number is determined by the number of serial ports configured to use mavlink\. So 0 is always the console\, 1 is the next serial port using mavlink\, 2 the next after that and so on\.


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | Console                 | |
| +-------+-------------------------+ |
| | 1     | Mavlink Serial Channel1 | |
| +-------+-------------------------+ |
| | 2     | Mavlink Serial Channel2 | |
| +-------+-------------------------+ |
| | 3     | Mavlink Serial Channel3 | |
| +-------+-------------------------+ |
| | 4     | Mavlink Serial Channel4 | |
| +-------+-------------------------+ |
| | 5     | Mavlink Serial Channel5 | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _OUT_BLH_POLES:

OUT\_BLH\_POLES: BLHeli Motor Poles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This allows calculation of true RPM from ESC\'s eRPM\. The default is 14\.


+---------+
| Range   |
+=========+
| 1 - 127 |
+---------+




.. _OUT_BLH_3DMASK:

OUT\_BLH\_3DMASK: BLHeli bitmask of 3D channels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Mask of channels which are dynamically reversible\. This is used to configure ESCs in \'3D\' mode\, allowing for the motor to spin in either direction


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | Channel1  | |
| +-----+-----------+ |
| | 1   | Channel2  | |
| +-----+-----------+ |
| | 2   | Channel3  | |
| +-----+-----------+ |
| | 3   | Channel4  | |
| +-----+-----------+ |
| | 4   | Channel5  | |
| +-----+-----------+ |
| | 5   | Channel6  | |
| +-----+-----------+ |
| | 6   | Channel7  | |
| +-----+-----------+ |
| | 7   | Channel8  | |
| +-----+-----------+ |
| | 8   | Channel9  | |
| +-----+-----------+ |
| | 9   | Channel10 | |
| +-----+-----------+ |
| | 10  | Channel11 | |
| +-----+-----------+ |
| | 11  | Channel12 | |
| +-----+-----------+ |
| | 12  | Channel13 | |
| +-----+-----------+ |
| | 13  | Channel14 | |
| +-----+-----------+ |
| | 14  | Channel15 | |
| +-----+-----------+ |
| | 15  | Channel16 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _OUT_BLH_BDMASK:

OUT\_BLH\_BDMASK: BLHeli bitmask of bi\-directional dshot channels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Mask of channels which support bi\-directional dshot\. This is used for ESCs which have firmware that supports bi\-directional dshot allowing fast rpm telemetry values to be returned for the harmonic notch\.


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | Channel1  | |
| +-----+-----------+ |
| | 1   | Channel2  | |
| +-----+-----------+ |
| | 2   | Channel3  | |
| +-----+-----------+ |
| | 3   | Channel4  | |
| +-----+-----------+ |
| | 4   | Channel5  | |
| +-----+-----------+ |
| | 5   | Channel6  | |
| +-----+-----------+ |
| | 6   | Channel7  | |
| +-----+-----------+ |
| | 7   | Channel8  | |
| +-----+-----------+ |
| | 8   | Channel9  | |
| +-----+-----------+ |
| | 9   | Channel10 | |
| +-----+-----------+ |
| | 10  | Channel11 | |
| +-----+-----------+ |
| | 11  | Channel12 | |
| +-----+-----------+ |
| | 12  | Channel13 | |
| +-----+-----------+ |
| | 13  | Channel14 | |
| +-----+-----------+ |
| | 14  | Channel15 | |
| +-----+-----------+ |
| | 15  | Channel16 | |
| +-----+-----------+ |
|                     |
+---------------------+




.. _OUT_BLH_RVMASK:

OUT\_BLH\_RVMASK: BLHeli bitmask of reversed channels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Mask of channels which are reversed\. This is used to configure ESCs in reversed mode


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | Channel1  | |
| +-----+-----------+ |
| | 1   | Channel2  | |
| +-----+-----------+ |
| | 2   | Channel3  | |
| +-----+-----------+ |
| | 3   | Channel4  | |
| +-----+-----------+ |
| | 4   | Channel5  | |
| +-----+-----------+ |
| | 5   | Channel6  | |
| +-----+-----------+ |
| | 6   | Channel7  | |
| +-----+-----------+ |
| | 7   | Channel8  | |
| +-----+-----------+ |
| | 8   | Channel9  | |
| +-----+-----------+ |
| | 9   | Channel10 | |
| +-----+-----------+ |
| | 10  | Channel11 | |
| +-----+-----------+ |
| | 11  | Channel12 | |
| +-----+-----------+ |
| | 12  | Channel13 | |
| +-----+-----------+ |
| | 13  | Channel14 | |
| +-----+-----------+ |
| | 14  | Channel15 | |
| +-----+-----------+ |
| | 15  | Channel16 | |
| +-----+-----------+ |
|                     |
+---------------------+





.. _parameters_OUT_FTW_:

OUT\_FTW\_ Parameters
---------------------


.. _OUT_FTW_MASK:

OUT\_FTW\_MASK: Servo channel output bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Servo channel mask specifying FETtec ESC output\.


+-------------------+
| Bitmask           |
+===================+
| +-----+---------+ |
| | Bit | Meaning | |
| +=====+=========+ |
| | 0   | SERVO1  | |
| +-----+---------+ |
| | 1   | SERVO2  | |
| +-----+---------+ |
| | 2   | SERVO3  | |
| +-----+---------+ |
| | 3   | SERVO4  | |
| +-----+---------+ |
| | 4   | SERVO5  | |
| +-----+---------+ |
| | 5   | SERVO6  | |
| +-----+---------+ |
| | 6   | SERVO7  | |
| +-----+---------+ |
| | 7   | SERVO8  | |
| +-----+---------+ |
| | 8   | SERVO9  | |
| +-----+---------+ |
| | 9   | SERVO10 | |
| +-----+---------+ |
| | 10  | SERVO11 | |
| +-----+---------+ |
| | 11  | SERVO12 | |
| +-----+---------+ |
|                   |
+-------------------+




.. _OUT_FTW_RVMASK:

OUT\_FTW\_RVMASK: Servo channel reverse rotation bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Servo channel mask to reverse rotation of FETtec ESC outputs\.


+-------------------+
| Bitmask           |
+===================+
| +-----+---------+ |
| | Bit | Meaning | |
| +=====+=========+ |
| | 0   | SERVO1  | |
| +-----+---------+ |
| | 1   | SERVO2  | |
| +-----+---------+ |
| | 2   | SERVO3  | |
| +-----+---------+ |
| | 3   | SERVO4  | |
| +-----+---------+ |
| | 4   | SERVO5  | |
| +-----+---------+ |
| | 5   | SERVO6  | |
| +-----+---------+ |
| | 6   | SERVO7  | |
| +-----+---------+ |
| | 7   | SERVO8  | |
| +-----+---------+ |
| | 8   | SERVO9  | |
| +-----+---------+ |
| | 9   | SERVO10 | |
| +-----+---------+ |
| | 10  | SERVO11 | |
| +-----+---------+ |
| | 11  | SERVO12 | |
| +-----+---------+ |
|                   |
+-------------------+




.. _OUT_FTW_POLES:

OUT\_FTW\_POLES: Nr\. electrical poles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of motor electrical poles


+--------+
| Range  |
+========+
| 2 - 50 |
+--------+





.. _parameters_OUT_ROB_:

OUT\_ROB\_ Parameters
---------------------


.. _OUT_ROB_POSMIN:

OUT\_ROB\_POSMIN: Robotis servo position min
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position minimum at servo min value\. This should be within the position control range of the servos\, normally 0 to 4095


+----------+
| Range    |
+==========+
| 0 - 4095 |
+----------+




.. _OUT_ROB_POSMAX:

OUT\_ROB\_POSMAX: Robotis servo position max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position maximum at servo max value\. This should be within the position control range of the servos\, normally 0 to 4095


+----------+
| Range    |
+==========+
| 0 - 4095 |
+----------+





.. _parameters_OUT_SBUS_:

OUT\_SBUS\_ Parameters
----------------------


.. _OUT_SBUS_RATE:

OUT\_SBUS\_RATE: SBUS default output rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the SBUS output frame rate in Hz\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 25 - 250 | hertz |
+----------+-------+





.. _parameters_OUT_VOLZ_:

OUT\_VOLZ\_ Parameters
----------------------


.. _OUT_VOLZ_MASK:

OUT\_VOLZ\_MASK: Channel Bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable of volz servo protocol to specific channels


+---------------------+
| Bitmask             |
+=====================+
| +-----+-----------+ |
| | Bit | Meaning   | |
| +=====+===========+ |
| | 0   | Channel1  | |
| +-----+-----------+ |
| | 1   | Channel2  | |
| +-----+-----------+ |
| | 2   | Channel3  | |
| +-----+-----------+ |
| | 3   | Channel4  | |
| +-----+-----------+ |
| | 4   | Channel5  | |
| +-----+-----------+ |
| | 5   | Channel6  | |
| +-----+-----------+ |
| | 6   | Channel7  | |
| +-----+-----------+ |
| | 7   | Channel8  | |
| +-----+-----------+ |
| | 8   | Channel9  | |
| +-----+-----------+ |
| | 9   | Channel10 | |
| +-----+-----------+ |
| | 10  | Channel11 | |
| +-----+-----------+ |
| | 11  | Channel12 | |
| +-----+-----------+ |
| | 12  | Channel13 | |
| +-----+-----------+ |
| | 13  | Channel14 | |
| +-----+-----------+ |
| | 14  | Channel15 | |
| +-----+-----------+ |
| | 15  | Channel16 | |
| +-----+-----------+ |
|                     |
+---------------------+





.. _parameters_RNGFND1_:

RNGFND1\_ Parameters
--------------------


.. _RNGFND1_TYPE:

RNGFND1\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND1_PIN:

RNGFND1\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND1_SCALING:

RNGFND1\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND1_OFFSET:

RNGFND1\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND1_FUNCTION:

RNGFND1\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND1_MIN_CM:

RNGFND1\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND1_MAX_CM:

RNGFND1\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND1_STOP_PIN:

RNGFND1\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND1_RMETRIC:

RNGFND1\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND1_PWRRNG:

RNGFND1\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND1_GNDCLEAR:

RNGFND1\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND1_ADDR:

RNGFND1\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND1_POS_X:

RNGFND1\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND1_POS_Y:

RNGFND1\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND1_POS_Z:

RNGFND1\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND1_ORIENT:

RNGFND1\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND1_WSP_MAVG:

RNGFND1\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND1_WSP_MEDF:

RNGFND1\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND1_WSP_FRQ:

RNGFND1\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND1_WSP_AVG:

RNGFND1\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND1_WSP_THR:

RNGFND1\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND1_WSP_BAUD:

RNGFND1\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND1_RECV_ID:

RNGFND1\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND1_SNR_MIN:

RNGFND1\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND2_:

RNGFND2\_ Parameters
--------------------


.. _RNGFND2_TYPE:

RNGFND2\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND2_PIN:

RNGFND2\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND2_SCALING:

RNGFND2\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND2_OFFSET:

RNGFND2\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND2_FUNCTION:

RNGFND2\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND2_MIN_CM:

RNGFND2\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND2_MAX_CM:

RNGFND2\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND2_STOP_PIN:

RNGFND2\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND2_RMETRIC:

RNGFND2\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND2_PWRRNG:

RNGFND2\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND2_GNDCLEAR:

RNGFND2\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND2_ADDR:

RNGFND2\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND2_POS_X:

RNGFND2\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND2_POS_Y:

RNGFND2\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND2_POS_Z:

RNGFND2\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND2_ORIENT:

RNGFND2\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND2_WSP_MAVG:

RNGFND2\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND2_WSP_MEDF:

RNGFND2\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND2_WSP_FRQ:

RNGFND2\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND2_WSP_AVG:

RNGFND2\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND2_WSP_THR:

RNGFND2\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND2_WSP_BAUD:

RNGFND2\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND2_RECV_ID:

RNGFND2\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND2_SNR_MIN:

RNGFND2\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND3_:

RNGFND3\_ Parameters
--------------------


.. _RNGFND3_TYPE:

RNGFND3\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND3_PIN:

RNGFND3\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND3_SCALING:

RNGFND3\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND3_OFFSET:

RNGFND3\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND3_FUNCTION:

RNGFND3\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND3_MIN_CM:

RNGFND3\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND3_MAX_CM:

RNGFND3\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND3_STOP_PIN:

RNGFND3\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND3_RMETRIC:

RNGFND3\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND3_PWRRNG:

RNGFND3\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND3_GNDCLEAR:

RNGFND3\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND3_ADDR:

RNGFND3\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND3_POS_X:

RNGFND3\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND3_POS_Y:

RNGFND3\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND3_POS_Z:

RNGFND3\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND3_ORIENT:

RNGFND3\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND3_WSP_MAVG:

RNGFND3\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND3_WSP_MEDF:

RNGFND3\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND3_WSP_FRQ:

RNGFND3\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND3_WSP_AVG:

RNGFND3\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND3_WSP_THR:

RNGFND3\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND3_WSP_BAUD:

RNGFND3\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND3_RECV_ID:

RNGFND3\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND3_SNR_MIN:

RNGFND3\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND4_:

RNGFND4\_ Parameters
--------------------


.. _RNGFND4_TYPE:

RNGFND4\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND4_PIN:

RNGFND4\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND4_SCALING:

RNGFND4\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND4_OFFSET:

RNGFND4\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND4_FUNCTION:

RNGFND4\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND4_MIN_CM:

RNGFND4\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND4_MAX_CM:

RNGFND4\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND4_STOP_PIN:

RNGFND4\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND4_RMETRIC:

RNGFND4\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND4_PWRRNG:

RNGFND4\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND4_GNDCLEAR:

RNGFND4\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND4_ADDR:

RNGFND4\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND4_POS_X:

RNGFND4\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND4_POS_Y:

RNGFND4\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND4_POS_Z:

RNGFND4\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND4_ORIENT:

RNGFND4\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND4_WSP_MAVG:

RNGFND4\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND4_WSP_MEDF:

RNGFND4\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND4_WSP_FRQ:

RNGFND4\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND4_WSP_AVG:

RNGFND4\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND4_WSP_THR:

RNGFND4\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND4_WSP_BAUD:

RNGFND4\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND4_RECV_ID:

RNGFND4\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND4_SNR_MIN:

RNGFND4\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND5_:

RNGFND5\_ Parameters
--------------------


.. _RNGFND5_TYPE:

RNGFND5\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND5_PIN:

RNGFND5\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND5_SCALING:

RNGFND5\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND5_OFFSET:

RNGFND5\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND5_FUNCTION:

RNGFND5\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND5_MIN_CM:

RNGFND5\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND5_MAX_CM:

RNGFND5\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND5_STOP_PIN:

RNGFND5\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND5_RMETRIC:

RNGFND5\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND5_PWRRNG:

RNGFND5\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND5_GNDCLEAR:

RNGFND5\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND5_ADDR:

RNGFND5\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND5_POS_X:

RNGFND5\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND5_POS_Y:

RNGFND5\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND5_POS_Z:

RNGFND5\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND5_ORIENT:

RNGFND5\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND5_WSP_MAVG:

RNGFND5\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND5_WSP_MEDF:

RNGFND5\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND5_WSP_FRQ:

RNGFND5\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND5_WSP_AVG:

RNGFND5\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND5_WSP_THR:

RNGFND5\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND5_WSP_BAUD:

RNGFND5\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND5_RECV_ID:

RNGFND5\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND5_SNR_MIN:

RNGFND5\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND6_:

RNGFND6\_ Parameters
--------------------


.. _RNGFND6_TYPE:

RNGFND6\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND6_PIN:

RNGFND6\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND6_SCALING:

RNGFND6\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND6_OFFSET:

RNGFND6\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND6_FUNCTION:

RNGFND6\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND6_MIN_CM:

RNGFND6\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND6_MAX_CM:

RNGFND6\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND6_STOP_PIN:

RNGFND6\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND6_RMETRIC:

RNGFND6\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND6_PWRRNG:

RNGFND6\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND6_GNDCLEAR:

RNGFND6\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND6_ADDR:

RNGFND6\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND6_POS_X:

RNGFND6\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND6_POS_Y:

RNGFND6\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND6_POS_Z:

RNGFND6\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND6_ORIENT:

RNGFND6\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND6_WSP_MAVG:

RNGFND6\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND6_WSP_MEDF:

RNGFND6\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND6_WSP_FRQ:

RNGFND6\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND6_WSP_AVG:

RNGFND6\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND6_WSP_THR:

RNGFND6\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND6_WSP_BAUD:

RNGFND6\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND6_RECV_ID:

RNGFND6\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND6_SNR_MIN:

RNGFND6\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND7_:

RNGFND7\_ Parameters
--------------------


.. _RNGFND7_TYPE:

RNGFND7\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND7_PIN:

RNGFND7\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND7_SCALING:

RNGFND7\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND7_OFFSET:

RNGFND7\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND7_FUNCTION:

RNGFND7\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND7_MIN_CM:

RNGFND7\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND7_MAX_CM:

RNGFND7\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND7_STOP_PIN:

RNGFND7\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND7_RMETRIC:

RNGFND7\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND7_PWRRNG:

RNGFND7\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND7_GNDCLEAR:

RNGFND7\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND7_ADDR:

RNGFND7\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND7_POS_X:

RNGFND7\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND7_POS_Y:

RNGFND7\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND7_POS_Z:

RNGFND7\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND7_ORIENT:

RNGFND7\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND7_WSP_MAVG:

RNGFND7\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND7_WSP_MEDF:

RNGFND7\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND7_WSP_FRQ:

RNGFND7\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND7_WSP_AVG:

RNGFND7\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND7_WSP_THR:

RNGFND7\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND7_WSP_BAUD:

RNGFND7\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND7_RECV_ID:

RNGFND7\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND7_SNR_MIN:

RNGFND7\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND8_:

RNGFND8\_ Parameters
--------------------


.. _RNGFND8_TYPE:

RNGFND8\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND8_PIN:

RNGFND8\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND8_SCALING:

RNGFND8\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND8_OFFSET:

RNGFND8\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND8_FUNCTION:

RNGFND8\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND8_MIN_CM:

RNGFND8\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND8_MAX_CM:

RNGFND8\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND8_STOP_PIN:

RNGFND8\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND8_RMETRIC:

RNGFND8\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND8_PWRRNG:

RNGFND8\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND8_GNDCLEAR:

RNGFND8\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND8_ADDR:

RNGFND8\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND8_POS_X:

RNGFND8\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND8_POS_Y:

RNGFND8\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND8_POS_Z:

RNGFND8\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND8_ORIENT:

RNGFND8\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND8_WSP_MAVG:

RNGFND8\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND8_WSP_MEDF:

RNGFND8\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND8_WSP_FRQ:

RNGFND8\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND8_WSP_AVG:

RNGFND8\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND8_WSP_THR:

RNGFND8\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND8_WSP_BAUD:

RNGFND8\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND8_RECV_ID:

RNGFND8\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND8_SNR_MIN:

RNGFND8\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFND9_:

RNGFND9\_ Parameters
--------------------


.. _RNGFND9_TYPE:

RNGFND9\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFND9_PIN:

RNGFND9\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFND9_SCALING:

RNGFND9\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFND9_OFFSET:

RNGFND9\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFND9_FUNCTION:

RNGFND9\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND9_MIN_CM:

RNGFND9\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND9_MAX_CM:

RNGFND9\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND9_STOP_PIN:

RNGFND9\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFND9_RMETRIC:

RNGFND9\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND9_PWRRNG:

RNGFND9\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND9_GNDCLEAR:

RNGFND9\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFND9_ADDR:

RNGFND9\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFND9_POS_X:

RNGFND9\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND9_POS_Y:

RNGFND9\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND9_POS_Z:

RNGFND9\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFND9_ORIENT:

RNGFND9\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFND9_WSP_MAVG:

RNGFND9\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND9_WSP_MEDF:

RNGFND9\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND9_WSP_FRQ:

RNGFND9\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFND9_WSP_AVG:

RNGFND9\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFND9_WSP_THR:

RNGFND9\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFND9_WSP_BAUD:

RNGFND9\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND9_RECV_ID:

RNGFND9\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFND9_SNR_MIN:

RNGFND9\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_RNGFNDA_:

RNGFNDA\_ Parameters
--------------------


.. _RNGFNDA_TYPE:

RNGFNDA\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of connected rangefinder


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | None                   | |
| +-------+------------------------+ |
| | 1     | Analog                 | |
| +-------+------------------------+ |
| | 2     | MaxbotixI2C            | |
| +-------+------------------------+ |
| | 3     | LidarLite-I2C          | |
| +-------+------------------------+ |
| | 5     | PWM                    | |
| +-------+------------------------+ |
| | 6     | BBB-PRU                | |
| +-------+------------------------+ |
| | 7     | LightWareI2C           | |
| +-------+------------------------+ |
| | 8     | LightWareSerial        | |
| +-------+------------------------+ |
| | 9     | Bebop                  | |
| +-------+------------------------+ |
| | 10    | MAVLink                | |
| +-------+------------------------+ |
| | 11    | USD1_Serial            | |
| +-------+------------------------+ |
| | 12    | LeddarOne              | |
| +-------+------------------------+ |
| | 13    | MaxbotixSerial         | |
| +-------+------------------------+ |
| | 14    | TeraRangerI2C          | |
| +-------+------------------------+ |
| | 15    | LidarLiteV3-I2C        | |
| +-------+------------------------+ |
| | 16    | VL53L0X or VL53L1X     | |
| +-------+------------------------+ |
| | 17    | NMEA                   | |
| +-------+------------------------+ |
| | 18    | WASP-LRF               | |
| +-------+------------------------+ |
| | 19    | BenewakeTF02           | |
| +-------+------------------------+ |
| | 20    | Benewake-Serial        | |
| +-------+------------------------+ |
| | 21    | LidarLightV3HP         | |
| +-------+------------------------+ |
| | 22    | PWM                    | |
| +-------+------------------------+ |
| | 23    | BlueRoboticsPing       | |
| +-------+------------------------+ |
| | 24    | DroneCAN               | |
| +-------+------------------------+ |
| | 25    | BenewakeTFminiPlus-I2C | |
| +-------+------------------------+ |
| | 26    | LanbaoPSK-CM8JL65-CC5  | |
| +-------+------------------------+ |
| | 27    | BenewakeTF03           | |
| +-------+------------------------+ |
| | 28    | VL53L1X-ShortRange     | |
| +-------+------------------------+ |
| | 29    | LeddarVu8-Serial       | |
| +-------+------------------------+ |
| | 30    | HC-SR04                | |
| +-------+------------------------+ |
| | 31    | GYUS42v2               | |
| +-------+------------------------+ |
| | 32    | MSP                    | |
| +-------+------------------------+ |
| | 33    | USD1_CAN               | |
| +-------+------------------------+ |
| | 34    | Benewake_CAN           | |
| +-------+------------------------+ |
| | 100   | SITL                   | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _RNGFNDA_PIN:

RNGFNDA\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog or PWM input pin that rangefinder is connected to\. Airspeed ports can be used for Analog input\, AUXOUT can be used for PWM input\. When using analog pin 103\, the maximum value of the input in 3\.3V\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | -1    | Not Used                  | |
| +-------+---------------------------+ |
| | 11    | Pixracer                  | |
| +-------+---------------------------+ |
| | 13    | Pixhawk ADC4              | |
| +-------+---------------------------+ |
| | 14    | Pixhawk ADC3              | |
| +-------+---------------------------+ |
| | 15    | Pixhawk ADC6/Pixhawk2 ADC | |
| +-------+---------------------------+ |
| | 50    | AUX1                      | |
| +-------+---------------------------+ |
| | 51    | AUX2                      | |
| +-------+---------------------------+ |
| | 52    | AUX3                      | |
| +-------+---------------------------+ |
| | 53    | AUX4                      | |
| +-------+---------------------------+ |
| | 54    | AUX5                      | |
| +-------+---------------------------+ |
| | 55    | AUX6                      | |
| +-------+---------------------------+ |
| | 103   | Pixhawk SBUS              | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _RNGFNDA_SCALING:

RNGFNDA\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\. For Maxbotix serial sonar this is unit conversion to meters\.


+-----------+-----------------+
| Increment | Units           |
+===========+=================+
| 0.001     | meters per volt |
+-----------+-----------------+




.. _RNGFNDA_OFFSET:

RNGFNDA\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | volt  |
+-----------+-------+




.. _RNGFNDA_FUNCTION:

RNGFNDA\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFNDA_MIN_CM:

RNGFNDA\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFNDA_MAX_CM:

RNGFNDA\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFNDA_STOP_PIN:

RNGFNDA\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for the pwm rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This is used to enable powersaving when out of range\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | -1    | Not Used       | |
| +-------+----------------+ |
| | 50    | AUX1           | |
| +-------+----------------+ |
| | 51    | AUX2           | |
| +-------+----------------+ |
| | 52    | AUX3           | |
| +-------+----------------+ |
| | 53    | AUX4           | |
| +-------+----------------+ |
| | 54    | AUX5           | |
| +-------+----------------+ |
| | 55    | AUX6           | |
| +-------+----------------+ |
| | 111   | PX4 FMU Relay1 | |
| +-------+----------------+ |
| | 112   | PX4 FMU Relay2 | |
| +-------+----------------+ |
| | 113   | PX4IO Relay1   | |
| +-------+----------------+ |
| | 114   | PX4IO Relay2   | |
| +-------+----------------+ |
| | 115   | PX4IO ACC1     | |
| +-------+----------------+ |
| | 116   | PX4IO ACC2     | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _RNGFNDA_RMETRIC:

RNGFNDA\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFNDA_PWRRNG:

RNGFNDA\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFNDA_GNDCLEAR:

RNGFNDA\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+-----------+---------+-------------+
| Increment | Range   | Units       |
+===========+=========+=============+
| 1         | 5 - 127 | centimeters |
+-----------+---------+-------------+




.. _RNGFNDA_ADDR:

RNGFNDA\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 - 127 |
+-----------+---------+




.. _RNGFNDA_POS_X:

RNGFNDA\_POS\_X:  X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the rangefinder in body frame\. Positive X is forward of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFNDA_POS_Y:

RNGFNDA\_POS\_Y: Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the rangefinder in body frame\. Positive Y is to the right of the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFNDA_POS_Z:

RNGFNDA\_POS\_Z: Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the rangefinder in body frame\. Positive Z is down from the origin\. Use the zero range datum point if supplied\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _RNGFNDA_ORIENT:

RNGFNDA\_ORIENT: Rangefinder orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Orientation of rangefinder


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Forward       | |
| +-------+---------------+ |
| | 1     | Forward-Right | |
| +-------+---------------+ |
| | 2     | Right         | |
| +-------+---------------+ |
| | 3     | Back-Right    | |
| +-------+---------------+ |
| | 4     | Back          | |
| +-------+---------------+ |
| | 5     | Back-Left     | |
| +-------+---------------+ |
| | 6     | Left          | |
| +-------+---------------+ |
| | 7     | Forward-Left  | |
| +-------+---------------+ |
| | 24    | Up            | |
| +-------+---------------+ |
| | 25    | Down          | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _RNGFNDA_WSP_MAVG:

RNGFNDA\_WSP\_MAVG: Moving Average Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result\. When MAVG is greater than 1\, the current range result will be the current measured value averaged with the N\-1 previous results


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFNDA_WSP_MEDF:

RNGFNDA\_WSP\_MEDF: Moving Median Filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the window size for the real\-time median filter\. When MEDF is greater than 0 the median filter is active


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFNDA_WSP_FRQ:

RNGFNDA\_WSP\_FRQ: Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz\. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers\.


+-----------+
| Range     |
+===========+
| 0 - 10000 |
+-----------+




.. _RNGFNDA_WSP_AVG:

RNGFNDA\_WSP\_AVG: Multi\-pulse averages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi\-pulse averaging mode\. In this mode\, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement


+---------+
| Range   |
+=========+
| 0 - 255 |
+---------+




.. _RNGFNDA_WSP_THR:

RNGFNDA\_WSP\_THR: Sensitivity threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the system sensitivity\. Larger values of THR represent higher sensitivity\. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory\. Set to \-1 for automatic threshold adjustments


+----------+
| Range    |
+==========+
| -1 - 255 |
+----------+




.. _RNGFNDA_WSP_BAUD:

RNGFNDA\_WSP\_BAUD: Baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired baud rate


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Low Speed  | |
| +-------+------------+ |
| | 1     | High Speed | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFNDA_RECV_ID:

RNGFNDA\_RECV\_ID: CAN receive ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The receive ID of the CAN frames\. A value of zero means all IDs are accepted\.


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+




.. _RNGFNDA_SNR_MIN:

RNGFNDA\_SNR\_MIN: Minimum signal strength
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum signal strength \(SNR\) to accept distance


+-----------+
| Range     |
+===========+
| 0 - 65535 |
+-----------+





.. _parameters_SCR_:

SCR\_ Parameters
----------------


.. _SCR_ENABLE:

SCR\_ENABLE: Enable Scripting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Controls if scripting is enabled


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | None        | |
| +-------+-------------+ |
| | 1     | Lua Scripts | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _SCR_VM_I_COUNT:

SCR\_VM\_I\_COUNT: Scripting Virtual Machine Instruction Count
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The number virtual machine instructions that can be run before considering a script to have taken an excessive amount of time


+-----------+----------------+
| Increment | Range          |
+===========+================+
| 10000     | 1000 - 1000000 |
+-----------+----------------+




.. _SCR_HEAP_SIZE:

SCR\_HEAP\_SIZE: Scripting Heap Size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Amount of memory available for scripting


+-----------+----------------+
| Increment | Range          |
+===========+================+
| 1024      | 1024 - 1048576 |
+-----------+----------------+




.. _SCR_DEBUG_OPTS:

SCR\_DEBUG\_OPTS: Scripting Debug Level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Debugging options


+-----------------------------------------------------------------+
| Bitmask                                                         |
+=================================================================+
| +-----+-------------------------------------------------------+ |
| | Bit | Meaning                                               | |
| +=====+=======================================================+ |
| | 0   | No Scripts to run message if all scripts have stopped | |
| +-----+-------------------------------------------------------+ |
| | 1   | Runtime messages for memory usage and execution time  | |
| +-----+-------------------------------------------------------+ |
| | 2   | Suppress logging scripts to dataflash                 | |
| +-----+-------------------------------------------------------+ |
| | 3   | log runtime memory usage and execution time           | |
| +-----+-------------------------------------------------------+ |
|                                                                 |
+-----------------------------------------------------------------+




.. _SCR_USER1:

SCR\_USER1: Scripting User Parameter1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER2:

SCR\_USER2: Scripting User Parameter2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER3:

SCR\_USER3: Scripting User Parameter3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER4:

SCR\_USER4: Scripting User Parameter4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER5:

SCR\_USER5: Scripting User Parameter5
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER6:

SCR\_USER6: Scripting User Parameter6
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_DIR_DISABLE:

SCR\_DIR\_DISABLE: Directory disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This will stop scripts being loaded from the given locations


+-----------------------+
| Bitmask               |
+=======================+
| +-----+-------------+ |
| | Bit | Meaning     | |
| +=====+=============+ |
| | 0   | ROMFS       | |
| +-----+-------------+ |
| | 1   | APM/scripts | |
| +-----+-------------+ |
|                       |
+-----------------------+





.. _parameters_SERIAL:

SERIAL Parameters
-----------------


.. _SERIAL0_BAUD:

SERIAL0\_BAUD: Serial0 baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used on the USB console\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL0_PROTOCOL:

SERIAL0\_PROTOCOL: Console protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol to use on the console\. 


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 1     | MAVlink1 | |
| +-------+----------+ |
| | 2     | MAVLink2 | |
| +-------+----------+ |
|                      |
+----------------------+




.. _SERIAL1_PROTOCOL:

SERIAL1\_PROTOCOL: Telem1 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol to use on the Telem1 port\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL1_BAUD:

SERIAL1\_BAUD: Telem1 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used on the Telem1 port\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL2_PROTOCOL:

SERIAL2\_PROTOCOL: Telemetry 2 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol to use on the Telem2 port\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL2_BAUD:

SERIAL2\_BAUD: Telemetry 2 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate of the Telem2 port\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL3_PROTOCOL:

SERIAL3\_PROTOCOL: Serial 3 \(GPS\) protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial 3 \(GPS\) should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL3_BAUD:

SERIAL3\_BAUD: Serial 3 \(GPS\) Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for the Serial 3 \(GPS\)\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL4_PROTOCOL:

SERIAL4\_PROTOCOL: Serial4 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial4 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL4_BAUD:

SERIAL4\_BAUD: Serial 4 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial4\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL5_PROTOCOL:

SERIAL5\_PROTOCOL: Serial5 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial5 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL5_BAUD:

SERIAL5\_BAUD: Serial 5 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial5\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL6_PROTOCOL:

SERIAL6\_PROTOCOL: Serial6 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial6 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL6_BAUD:

SERIAL6\_BAUD: Serial 6 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial6\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL1_OPTIONS:

SERIAL1\_OPTIONS: Telem1 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL2_OPTIONS:

SERIAL2\_OPTIONS: Telem2 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL3_OPTIONS:

SERIAL3\_OPTIONS: Serial3 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL4_OPTIONS:

SERIAL4\_OPTIONS: Serial4 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL5_OPTIONS:

SERIAL5\_OPTIONS: Serial5 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL6_OPTIONS:

SERIAL6\_OPTIONS: Serial6 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL_PASS1:

SERIAL\_PASS1: Serial passthru first port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets one side of pass\-through between two serial ports\. Once both sides are set then all data received on either port will be passed to the other port


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 0     | Serial0  | |
| +-------+----------+ |
| | 1     | Serial1  | |
| +-------+----------+ |
| | 2     | Serial2  | |
| +-------+----------+ |
| | 3     | Serial3  | |
| +-------+----------+ |
| | 4     | Serial4  | |
| +-------+----------+ |
| | 5     | Serial5  | |
| +-------+----------+ |
| | 6     | Serial6  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _SERIAL_PASS2:

SERIAL\_PASS2: Serial passthru second port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets one side of pass\-through between two serial ports\. Once both sides are set then all data received on either port will be passed to the other port


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 0     | Serial0  | |
| +-------+----------+ |
| | 1     | Serial1  | |
| +-------+----------+ |
| | 2     | Serial2  | |
| +-------+----------+ |
| | 3     | Serial3  | |
| +-------+----------+ |
| | 4     | Serial4  | |
| +-------+----------+ |
| | 5     | Serial5  | |
| +-------+----------+ |
| | 6     | Serial6  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _SERIAL_PASSTIMO:

SERIAL\_PASSTIMO: Serial passthru timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets a timeout for serial pass\-through in seconds\. When the pass\-through is enabled by setting the SERIAL\_PASS1 and SERIAL\_PASS2 parameters then it remains in effect until no data comes from the first port for SERIAL\_PASSTIMO seconds\. This allows the port to revent to its normal usage \(such as MAVLink connection to a GCS\) when it is no longer needed\. A value of 0 means no timeout\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 120 | seconds |
+---------+---------+




.. _SERIAL7_PROTOCOL:

SERIAL7\_PROTOCOL: Serial7 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial7 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL7_BAUD:

SERIAL7\_BAUD: Serial 7 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial7\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL7_OPTIONS:

SERIAL7\_OPTIONS: Serial7 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL8_PROTOCOL:

SERIAL8\_PROTOCOL: Serial8 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial8 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL8_BAUD:

SERIAL8\_BAUD: Serial 8 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial8\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL8_OPTIONS:

SERIAL8\_OPTIONS: Serial8 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+




.. _SERIAL9_PROTOCOL:

SERIAL9\_PROTOCOL: Serial9 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial9 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | -1    | None                             | |
| +-------+----------------------------------+ |
| | 1     | MAVLink1                         | |
| +-------+----------------------------------+ |
| | 2     | MAVLink2                         | |
| +-------+----------------------------------+ |
| | 3     | Frsky D                          | |
| +-------+----------------------------------+ |
| | 4     | Frsky SPort                      | |
| +-------+----------------------------------+ |
| | 5     | GPS                              | |
| +-------+----------------------------------+ |
| | 7     | Alexmos Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 8     | SToRM32 Gimbal Serial            | |
| +-------+----------------------------------+ |
| | 9     | Rangefinder                      | |
| +-------+----------------------------------+ |
| | 10    | FrSky SPort Passthrough (OpenTX) | |
| +-------+----------------------------------+ |
| | 11    | Lidar360                         | |
| +-------+----------------------------------+ |
| | 13    | Beacon                           | |
| +-------+----------------------------------+ |
| | 14    | Volz servo out                   | |
| +-------+----------------------------------+ |
| | 15    | SBus servo out                   | |
| +-------+----------------------------------+ |
| | 16    | ESC Telemetry                    | |
| +-------+----------------------------------+ |
| | 17    | Devo Telemetry                   | |
| +-------+----------------------------------+ |
| | 18    | OpticalFlow                      | |
| +-------+----------------------------------+ |
| | 19    | RobotisServo                     | |
| +-------+----------------------------------+ |
| | 20    | NMEA Output                      | |
| +-------+----------------------------------+ |
| | 21    | WindVane                         | |
| +-------+----------------------------------+ |
| | 22    | SLCAN                            | |
| +-------+----------------------------------+ |
| | 23    | RCIN                             | |
| +-------+----------------------------------+ |
| | 24    | MegaSquirt EFI                   | |
| +-------+----------------------------------+ |
| | 25    | LTM                              | |
| +-------+----------------------------------+ |
| | 26    | RunCam                           | |
| +-------+----------------------------------+ |
| | 27    | HottTelem                        | |
| +-------+----------------------------------+ |
| | 28    | Scripting                        | |
| +-------+----------------------------------+ |
| | 29    | Crossfire VTX                    | |
| +-------+----------------------------------+ |
| | 30    | Generator                        | |
| +-------+----------------------------------+ |
| | 31    | Winch                            | |
| +-------+----------------------------------+ |
| | 32    | MSP                              | |
| +-------+----------------------------------+ |
| | 33    | DJI FPV                          | |
| +-------+----------------------------------+ |
| | 34    | AirSpeed                         | |
| +-------+----------------------------------+ |
| | 35    | ADSB                             | |
| +-------+----------------------------------+ |
| | 36    | AHRS                             | |
| +-------+----------------------------------+ |
| | 37    | SmartAudio                       | |
| +-------+----------------------------------+ |
| | 38    | FETtecOneWire                    | |
| +-------+----------------------------------+ |
| | 39    | Torqeedo                         | |
| +-------+----------------------------------+ |
| | 40    | AIS                              | |
| +-------+----------------------------------+ |
| | 41    | CoDevESC                         | |
| +-------+----------------------------------+ |
| | 42    | DisplayPort                      | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




.. _SERIAL9_BAUD:

SERIAL9\_BAUD: Serial 9 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial8\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 230   | 230400  | |
| +-------+---------+ |
| | 256   | 256000  | |
| +-------+---------+ |
| | 460   | 460800  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL9_OPTIONS:

SERIAL9\_OPTIONS: Serial9 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\.


+-----------------------------------------+
| Bitmask                                 |
+=========================================+
| +-----+-------------------------------+ |
| | Bit | Meaning                       | |
| +=====+===============================+ |
| | 0   | InvertRX                      | |
| +-----+-------------------------------+ |
| | 1   | InvertTX                      | |
| +-----+-------------------------------+ |
| | 2   | HalfDuplex                    | |
| +-----+-------------------------------+ |
| | 3   | Swap                          | |
| +-----+-------------------------------+ |
| | 4   | RX_PullDown                   | |
| +-----+-------------------------------+ |
| | 5   | RX_PullUp                     | |
| +-----+-------------------------------+ |
| | 6   | TX_PullDown                   | |
| +-----+-------------------------------+ |
| | 7   | TX_PullUp                     | |
| +-----+-------------------------------+ |
| | 8   | RX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 9   | TX_NoDMA                      | |
| +-----+-------------------------------+ |
| | 10  | Don't forward mavlink to/from | |
| +-----+-------------------------------+ |
| | 11  | DisableFIFO                   | |
| +-----+-------------------------------+ |
| | 12  | Ignore Streamrate             | |
| +-----+-------------------------------+ |
|                                         |
+-----------------------------------------+





.. _parameters_VISO:

VISO Parameters
---------------


.. _VISO_TYPE:

VISO\_TYPE: Visual odometry camera connection type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Visual odometry camera connection type


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | None          | |
| +-------+---------------+ |
| | 1     | MAVLink       | |
| +-------+---------------+ |
| | 2     | IntelT265     | |
| +-------+---------------+ |
| | 3     | VOXL(ModalAI) | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _VISO_POS_X:

VISO\_POS\_X: Visual odometry camera X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the camera in body frame\. Positive X is forward of the origin\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _VISO_POS_Y:

VISO\_POS\_Y: Visual odometry camera Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the camera in body frame\. Positive Y is to the right of the origin\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _VISO_POS_Z:

VISO\_POS\_Z: Visual odometry camera Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the camera in body frame\. Positive Z is down from the origin\.


+-----------+--------+--------+
| Increment | Range  | Units  |
+===========+========+========+
| 0.01      | -5 - 5 | meters |
+-----------+--------+--------+




.. _VISO_ORIENT:

VISO\_ORIENT: Visual odometery camera orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometery camera orientation


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Forward | |
| +-------+---------+ |
| | 2     | Right   | |
| +-------+---------+ |
| | 4     | Back    | |
| +-------+---------+ |
| | 6     | Left    | |
| +-------+---------+ |
| | 24    | Up      | |
| +-------+---------+ |
| | 25    | Down    | |
| +-------+---------+ |
|                     |
+---------------------+




.. _VISO_SCALE:

VISO\_SCALE: Visual odometry scaling factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry scaling factor applied to position estimates from sensor


.. _VISO_DELAY_MS:

VISO\_DELAY\_MS: Visual odometry sensor delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry sensor delay relative to inertial measurements


+---------+--------------+
| Range   | Units        |
+=========+==============+
| 0 - 250 | milliseconds |
+---------+--------------+




.. _VISO_VEL_M_NSE:

VISO\_VEL\_M\_NSE: Visual odometry velocity measurement noise
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry velocity measurement noise in m\/s


+------------+-------------------+
| Range      | Units             |
+============+===================+
| 0.05 - 5.0 | meters per second |
+------------+-------------------+




.. _VISO_POS_M_NSE:

VISO\_POS\_M\_NSE: Visual odometry position measurement noise
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry position measurement noise minimum \(meters\)\. This value will be used if the sensor provides a lower noise value \(or no noise value\)


+------------+--------+
| Range      | Units  |
+============+========+
| 0.1 - 10.0 | meters |
+------------+--------+




.. _VISO_YAW_M_NSE:

VISO\_YAW\_M\_NSE: Visual odometry yaw measurement noise
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry yaw measurement noise minimum \(radians\)\, This value will be used if the sensor provides a lower noise value \(or no noise value\)


+------------+---------+
| Range      | Units   |
+============+=========+
| 0.05 - 1.0 | radians |
+------------+---------+





.. _parameters_VTX_:

VTX\_ Parameters
----------------


.. _VTX_ENABLE:

VTX\_ENABLE: Is the Video Transmitter enabled or not
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Toggles the Video Transmitter on and off


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _VTX_POWER:

VTX\_POWER: Video Transmitter Power Level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Power Level\. Different VTXs support different power levels\, the power level chosen will be rounded down to the nearest supported power level


+----------+
| Range    |
+==========+
| 1 - 1000 |
+----------+




.. _VTX_CHANNEL:

VTX\_CHANNEL: Video Transmitter Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Channel


+-------+
| Range |
+=======+
| 0 - 7 |
+-------+




.. _VTX_BAND:

VTX\_BAND: Video Transmitter Band
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Band


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | Band A       | |
| +-------+--------------+ |
| | 1     | Band B       | |
| +-------+--------------+ |
| | 2     | Band E       | |
| +-------+--------------+ |
| | 3     | Airwave      | |
| +-------+--------------+ |
| | 4     | RaceBand     | |
| +-------+--------------+ |
| | 5     | Low RaceBand | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _VTX_FREQ:

VTX\_FREQ: Video Transmitter Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Frequency\. The frequency is derived from the setting of BAND and CHANNEL


+-------------+----------+
| Range       | ReadOnly |
+=============+==========+
| 5000 - 6000 | True     |
+-------------+----------+




.. _VTX_OPTIONS:

VTX\_OPTIONS: Video Transmitter Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Video Transmitter Options\. Pitmode puts the VTX in a low power state\. Unlocked enables certain restricted frequencies and power levels\. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels\.


+---------------------------------------------+
| Bitmask                                     |
+=============================================+
| +-----+-----------------------------------+ |
| | Bit | Meaning                           | |
| +=====+===================================+ |
| | 0   | Pitmode                           | |
| +-----+-----------------------------------+ |
| | 1   | Pitmode until armed               | |
| +-----+-----------------------------------+ |
| | 2   | Pitmode when disarmed             | |
| +-----+-----------------------------------+ |
| | 3   | Unlocked                          | |
| +-----+-----------------------------------+ |
| | 4   | Add leading zero byte to requests | |
| +-----+-----------------------------------+ |
|                                             |
+---------------------------------------------+




.. _VTX_MAX_POWER:

VTX\_MAX\_POWER: Video Transmitter Max Power Level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Maximum Power Level\. Different VTXs support different power levels\, this prevents the power aux switch from requesting too high a power level\. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting\.


+-----------+
| Range     |
+===========+
| 25 - 1000 |
+-----------+



