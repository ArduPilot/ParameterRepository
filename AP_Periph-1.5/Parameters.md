
This is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your autopilot to control vehicle behaviour. This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code. Some parameters may only be available for developers, and are enabled at compile-time.

# APPeriph Parameters

## FORMAT_VERSION: Eeprom format version number

*Note: This parameter is for advanced users*

This value is incremented when changes are made to the eeprom format

## CAN_NODE: UAVCAN node that is used for this network

*Note: This parameter is for advanced users*

UAVCAN node should be set implicitly or 0 for dynamic node allocation

- Range: 0 250

- RebootRequired: True

## CAN_BAUDRATE: Bitrate of CAN interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 10000 to 1000000

- Range: 10000 1000000

- RebootRequired: True

## CAN_SLCAN_CPORT: SLCAN Route

CAN Interface ID to be routed to SLCAN, 0 means no routing

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|First interface|
|2|Second interface|

- RebootRequired: True

## CAN_PROTOCOL: Enable use of specific protocol to be used on this port

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UAVCAN|
|4|PiccoloCAN|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|

- RebootRequired: True

## CAN2_BAUDRATE: Bitrate of CAN2 interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 10000 to 1000000

- Range: 10000 1000000

- RebootRequired: True

## CAN2_PROTOCOL: Enable use of specific protocol to be used on this port

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UAVCAN|
|4|PiccoloCAN|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|

## CAN3_BAUDRATE: Bitrate of CAN3 interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 10000 to 1000000

- Range: 10000 1000000

- RebootRequired: True

## CAN3_PROTOCOL: Enable use of specific protocol to be used on this port

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UAVCAN|
|4|PiccoloCAN|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|

## CAN_FDMODE: Enable CANFD mode

*Note: This parameter is for advanced users*

Enabling this option sets the CAN bus to be in CANFD mode with BRS.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## CAN_FDBAUDRATE: Set up bitrate for data section on CAN1

*Note: This parameter is for advanced users*

This sets the bitrate for the data section of CAN1.

|Value|Meaning|
|:---:|:---:|
|1|1M|
|2|2M|
|4|4M|
|5|5M|
|8|8M|

- RebootRequired: True

## CAN2_FDBAUDRATE: Set up bitrate for data section on CAN2

*Note: This parameter is for advanced users*

This sets the bitrate for the data section of CAN2.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|1|1M|
|2|2M|
|4|4M|
|5|5M|
|8|8M|

## FLASH_BOOTLOADER: Trigger bootloader update

*Note: This parameter is for advanced users*

DANGER! When enabled, the App will perform a bootloader update by copying the embedded bootloader over the existing bootloader. This may take a few seconds to perform and should only be done if you know what you're doing.

- Range: 0 1

## DEBUG: Debug

*Note: This parameter is for advanced users*

Debug

- Bitmask: 0:Disabled, 1:Show free stack space, 2:Auto Reboot after 15sec, 3:Enable sending stats

## BRD_SERIAL_NUM: Serial number of device

*Note: This parameter is for advanced users*

Non-zero positive values will be shown on the CAN App Name string

- Range: 0 2147483648

## BUZZER_VOLUME: Buzzer volume

*Note: This parameter is for advanced users*

Control the volume of the buzzer

- Range: 0 100

- Units: %

- Increment: 1

## GPS_PORT: GPS Serial Port

*Note: This parameter is for advanced users*

This is the serial port number where SERIALx_PROTOCOL will be set to GPS.

- Range: 0 10

- Increment: 1

- RebootRequired: True

## MB_CAN_PORT: Moving Baseline CAN Port option

*Note: This parameter is for advanced users*

Autoselect dedicated CAN port on which moving baseline data will be transmitted.

|Value|Meaning|
|:---:|:---:|
|0|Sends moving baseline data on all ports|
|1|auto select remaining port for transmitting Moving baseline Data|

- RebootRequired: True

## BARO_ENABLE: Barometer Enable

Barometer Enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## LED_BRIGHTNESS: LED Brightness

Select the RGB LED brightness level.

- Range: 0 100

- Units: %

- Increment: 1

## RNGFND_BAUDRATE: Rangefinder serial baudrate

Rangefinder serial baudrate.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|

- Increment: 1

- RebootRequired: True

## RNGFND_PORT: Rangefinder Serial Port

*Note: This parameter is for advanced users*

This is the serial port number where SERIALx_PROTOCOL will be set to Rangefinder.

- Range: 0 10

- Increment: 1

- RebootRequired: True

## RNGFND_MAX_RATE: Rangefinder max rate

*Note: This parameter is for advanced users*

This is the maximum rate we send rangefinder data in Hz. Zero means no limit

- Units: Hz

- Range: 0 200

- Increment: 1

## ADSB_BAUDRATE: ADSB serial baudrate

ADSB serial baudrate.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|

- Increment: 1

- RebootRequired: True

## ADSB_PORT: ADSB Serial Port

*Note: This parameter is for advanced users*

This is the serial port number where SERIALx_PROTOCOL will be set to ADSB.

- Range: 0 10

- Increment: 1

- RebootRequired: True

## HARDPOINT_ID: Hardpoint ID

*Note: This parameter is for advanced users*

Hardpoint ID

## HARDPOINT_RATE: Hardpoint PWM rate

*Note: This parameter is for advanced users*

Hardpoint PWM rate

- Range: 10 100

- Units: Hz

- Increment: 1

## ESC_NUMBER: ESC number

*Note: This parameter is for advanced users*

This is the ESC number to report as in UAVCAN ESC telemetry feedback packets.

- Increment: 1

## ESC_PWM_TYPE: Output PWM type

*Note: This parameter is for advanced users*

This selects the output PWM type, allowing for normal PWM continuous output, OneShot, brushed or DShot motor output

|Value|Meaning|
|:---:|:---:|
|1|Normal|
|2|OneShot|
|3|OneShot125|
|4|Brushed|
|5|DShot150|
|6|DShot300|
|7|DShot600|
|8|DShot1200|

- RebootRequired: True

## ESC_CMD_TIMO: ESC Command Timeout

*Note: This parameter is for advanced users*

This is the duration (ms) with which to hold the last driven ESC command before timing out and zeroing the ESC outputs. To disable zeroing of outputs in event of CAN loss, use 0. Use values greater than the expected duration between two CAN frames to ensure Periph is not starved of ESC Raw Commands.

- Range: 0 10000

- Units: ms

## ESC_TELEM_PORT: ESC Telemetry Serial Port

*Note: This parameter is for advanced users*

This is the serial port number where SERIALx_PROTOCOL will be set to ESC Telemetry

- Range: 0 10

- Increment: 1

- RebootRequired: True

## ESC_TELEM_RATE: ESC Telemetry update rate

*Note: This parameter is for advanced users*

This is the rate at which ESC Telemetry will be sent across the CAN bus

- Range: 0 1000

- Increment: 1

- RebootRequired: True

## MSP_PORT: MSP Serial Port

*Note: This parameter is for advanced users*

This is the serial port number where SERIALx_PROTOCOL will be set to MSP

- Range: 0 10

- Increment: 1

- RebootRequired: True

## LOG_BITMASK: Log bitmask

4 byte bitmap of log types to enable

- Bitmask: 2:GPS

## SYSID_THISMAV: MAVLink system ID of this vehicle

*Note: This parameter is for advanced users*

Allows setting an individual system id for this vehicle to distinguish it from others on the same network

- Range: 1 255

## EFI_BAUDRATE: EFI serial baudrate

EFI  serial baudrate.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|

- Increment: 1

- RebootRequired: True

## EFI_PORT: EFI Serial Port

*Note: This parameter is for advanced users*

This is the serial port number where SERIALx_PROTOCOL will be set to EFI.

- Range: 0 10

- Increment: 1

- RebootRequired: True

## PRX_BAUDRATE: Proximity Sensor serial baudrate

Proximity Sensor serial baudrate.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|

- Increment: 1

- RebootRequired: True

## PRX_PORT: Proximity Sensor Serial Port

*Note: This parameter is for advanced users*

This is the serial port number where SERIALx_PROTOCOL will be set to Proximity Sensor.

- Range: 0 10

- Increment: 1

- RebootRequired: True

## PRX_MAX_RATE: Proximity Sensor max rate

*Note: This parameter is for advanced users*

This is the maximum rate we send Proximity Sensor data in Hz. Zero means no limit

- Units: Hz

- Range: 0 200

- Increment: 1

## ESC_APD_SERIAL_1: ESC APD Serial 1

*Note: This parameter is for advanced users*

Which serial port to use for APD ESC data

- Range: 0 6

- Increment: 1

- RebootRequired: True

## ESC_APD_SERIAL_2: ESC APD Serial 2

*Note: This parameter is for advanced users*

Which serial port to use for APD ESC data

- Range: 0 6

- Increment: 1

- RebootRequired: True

# Lua Script Parameters

## POI_DIST_MAX: Mount POI distance max

POI's max distance (in meters) from the vehicle

- Range: 0 10000

## SHIP_ENABLE: Ship landing enable

Enable ship landing system

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## SHIP_LAND_ANGLE: Ship landing angle

Angle from the stern of the ship for landing approach. Use this to ensure that on a go-around that ship superstructure and cables are avoided. A value of zero means to approach from the rear of the ship. A value of 90 means the landing will approach from the port (left) side of the ship. A value of -90 will mean approaching from the starboard (right) side of the ship. A value of 180 will approach from the bow of the ship. This parameter is combined with the sign of the RTL_RADIUS parameter to determine the holdoff pattern. If RTL_RADIUS is positive then a clockwise loiter is performed, if RTL_RADIUS is negative then a counter-clockwise loiter is used.

- Range: -180 180

- Units: deg

## SHIP_AUTO_OFS: Ship automatic offset trigger

Settings this parameter to one triggers an automatic follow offset calculation based on current position of the vehicle and the landing target. NOTE: This parameter will auto-reset to zero once the offset has been calculated.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Trigger|

## PREV_ENABLE: parameter reversion enable

Enable parameter reversion system

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## PREV_RC_FUNC: param reversion RC function

RCn_OPTION number to used to trigger parameter reversion

## QUIK_ENABLE: Quicktune enable

Enable quicktune system

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## QUIK_AXES: Quicktune axes

axes to tune

- Bitmask: 0:Roll,1:Pitch,2:Yaw

## QUIK_DOUBLE_TIME: Quicktune doubling time

Time to double a tuning parameter. Raise this for a slower tune.

- Range: 5 20

- Units: s

## QUIK_GAIN_MARGIN: Quicktune gain margin

Reduction in gain after oscillation detected. Raise this number to get a more conservative tune

- Range: 20 80

- Units: %

## QUIK_OSC_SMAX: Quicktune oscillation rate threshold

Threshold for oscillation detection. A lower value will lead to a more conservative tune.

- Range: 1 10

## QUIK_YAW_P_MAX: Quicktune Yaw P max

Maximum value for yaw P gain

- Range: 0.1 3

## QUIK_YAW_D_MAX: Quicktune Yaw D max

Maximum value for yaw D gain

- Range: 0.001 1

## QUIK_RP_PI_RATIO: Quicktune roll/pitch PI ratio

Ratio between P and I gains for roll and pitch. Raise this to get a lower I gain

- Range: 0.5 1.0

## QUIK_Y_PI_RATIO: Quicktune Yaw PI ratio

Ratio between P and I gains for yaw. Raise this to get a lower I gain

- Range: 0.5 20

## QUIK_AUTO_FILTER: Quicktune auto filter enable

When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## QUIK_AUTO_SAVE: Quicktune auto save

Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune

- Units: s

## QUIK_RC_FUNC: Quicktune RC function

RCn_OPTION number to use to control tuning stop/start/save

## QUIK_MAX_REDUCE: Quicktune maximum gain reduction

This controls how much quicktune is allowed to lower gains from the original gains. If the vehicle already has a reasonable tune and is not oscillating then you can set this to zero to prevent gain reductions. The default of 20% is reasonable for most vehicles. Using a maximum gain reduction lowers the chance of an angle P oscillation happening if quicktune gets a false positive oscillation at a low gain, which can result in very low rate gains and a dangerous angle P oscillation.

- Units: %

- Range: 0 100

## DR_ENABLE: Deadreckoning Enable

Deadreckoning Enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## DR_ENABLE_DIST: Deadreckoning Enable Distance

Distance from home (in meters) beyond which the dead reckoning will be enabled

- Units: m

## DR_GPS_SACC_MAX: Deadreckoning GPS speed accuracy maximum threshold

GPS speed accuracy maximum, above which deadreckoning home will begin (default is 0.8).  Lower values trigger with good GPS quality, higher values will allow poorer GPS before triggering. Set to 0 to disable use of GPS speed accuracy

- Range: 0 10

## DR_GPS_SAT_MIN: Deadreckoning GPS satellite count min threshold

GPS satellite count threshold below which deadreckoning home will begin (default is 6).  Higher values trigger with good GPS quality, Lower values trigger with worse GPS quality. Set to 0 to disable use of GPS satellite count

- Range: 0 30

## DR_GPS_TRIGG_SEC: Deadreckoning GPS check trigger seconds

GPS checks must fail for this many seconds before dead reckoning will be triggered

- Units: s

## DR_FLY_ANGLE: Deadreckoning Lean Angle

lean angle (in degrees) during deadreckoning

- Units: deg

- Range: 0 45

## DR_FLY_ALT_MIN: Deadreckoning Altitude Min

Copter will fly at at least this altitude (in meters) above home during deadreckoning

- Units: m

- Range: 0 1000

## DR_FLY_TIMEOUT: Deadreckoning flight timeout

Copter will attempt to switch to NEXT_MODE after this many seconds of deadreckoning.  If it cannot switch modes it will continue in Guided_NoGPS.  Set to 0 to disable timeout

- Units: s

## DR_NEXT_MODE: Deadreckoning Next Mode

Copter switch to this mode after GPS recovers or DR_FLY_TIMEOUT has elapsed.  Default is 6/RTL.  Set to -1 to return to mode used before deadreckoning was triggered

|Value|Meaning|
|:---:|:---:|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|16|PosHold|
|17|Brake|
|20|Guided_NoGPS|
|21|Smart_RTL|
|27|Auto RTL|

## RCK_FORCEHL: Force enable High Latency mode

Automatically enables High Latency mode if not already enabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## RCK_PERIOD: Update rate

When in High Latency mode, send Rockblock updates every N seconds

- Range: 0 600

- Units: s

## RCK_DEBUG: Display Rockblock debugging text

Sends Rockblock debug text to GCS via statustexts

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## RCK_ENABLE: Enable Message transmission

Enables the Rockblock sending and recieving

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## AEROM_ANG_ACCEL: Angular acceleration limit

Maximum angular acceleration in maneuvers

- Units: deg/s/s

## AEROM_ANG_TC: Roll control filtertime constant

This is the time over which we filter the desired roll to smooth it

- Units: s

## AEROM_THR_PIT_FF: Throttle feed forward from pitch

This controls how much extra throttle to add based on pitch ange. The value is for 90 degrees and is applied in proportion to pitch

- Units: %

## AEROM_SPD_P: P gain for speed controller

This controls how rapidly the throttle is raised to compensate for a speed error

- Units: %

## AEROM_SPD_I: I gain for speed controller

This controls how rapidly the throttle is raised to compensate for a speed error

- Units: %

## AEROM_ROL_COR_TC: Roll control time constant

This is the time constant for correcting roll errors. A smaller value leads to faster roll corrections

- Units: s

## AEROM_TIME_COR_P: Time constant for correction of our distance along the path

This is the time constant for correcting path position errors

- Units: s

## AEROM_ERR_COR_P: P gain for path error corrections

This controls how rapidly we correct back onto the desired path

## AEROM_ERR_COR_D: D gain for path error corrections

This controls how rapidly we correct back onto the desired path

## AEROM_ENTRY_RATE: The roll rate to use when entering a roll maneuver

This controls how rapidly we roll into a new orientation

- Units: deg/s

## AEROM_THR_LKAHD: The lookahead for throttle control

This controls how far ahead we look in time along the path for the target throttle

- Units: s

## AEROM_DEBUG: Debug control

This controls the printing of extra debug information on paths

## AEROM_THR_MIN: Minimum Throttle

Lowest throttle used during maneuvers

- Units: %

## AEROM_THR_BOOST: Throttle boost

This is the extra throttle added in schedule elements marked as needing a throttle boost

- Units: %

## AEROM_YAW_ACCEL: Yaw acceleration

This is maximum yaw acceleration to use

- Units: deg/s/s

## AEROM_LKAHD: Lookahead

This is how much time to look ahead in the path for calculating path rates

- Units: s

## AEROM_PATH_SCALE: Path Scale

Scale factor for Path/Box size. 0.5 would half the distances in maneuvers. Radii are unaffected.

- Range: 0.1 100

## AEROM_BOX_WIDTH: Box Width

Length of aerobatic "box" 

- Units: m

## AEROM_STALL_THR: Stall turn throttle

Amount of throttle to reduce to for a stall turn

- Units: %

## AEROM_STALL_PIT: Stall turn pitch threshold

Pitch threashold for moving to final stage of stall turn

- Units: deg

## AEROM_KE_RUDD: KnifeEdge Rudder

Percent of rudder normally uses to sustain knife-edge at trick speed

- Units: %

## AEROM_KE_RUDD_LK: KnifeEdge Rudder lookahead

Time to look ahead in the path to calculate rudder correction for bank angle

- Units: s

## AEROM_ALT_ABORT: Altitude Abort

Maximum allowable loss in altitude during a trick or sequence from its starting altitude.

- Units: m

## AEROM_TS_P: Timesync P gain

This controls how rapidly two aircraft are brought back into time sync

## AEROM_TS_I: Timesync I gain

This controls how rapidly two aircraft are brought back into time sync

## AEROM_TS_SPDMAX: Timesync speed max

This sets the maximum speed adjustment for time sync between aircraft

- Units: m/s

## AEROM_TS_RATE: Timesync rate of send of NAMED_VALUE_FLOAT data

This sets the rate we send data for time sync between aircraft

- Units: Hz

## AEROM_MIS_ANGLE: Mission angle

When set to a non-zero value, this is the assumed direction of the mission. Otherwise the waypoint angle is used

- Units: deg

## AEROM_OPTIONS: Aerobatic options

Options to control aerobatic behavior

- Bitmask: 0:UseRTLOnAbort, 1:AddAtToMessages, 2:DualAircraftSynchronised

- Units: deg

## TRIK_ENABLE: Tricks on Switch Enable

Enables Tricks on Switch. TRIK params hidden until enabled

## TRIK_SEL_FN: Trik Selection Scripting Function

Setting an RC channel's _OPTION to this value will use it for trick selection

- Range: 301 307

## TRIK_ACT_FN: Trik Action Scripting Function

Setting an RC channel's _OPTION to this value will use it for trick action (abort,announce,execute)

- Range: 301 307

## TRIK_COUNT: Trik Count

Number of tricks which can be selected over the range of the trik selection RC channel

- Range: 1 11

## DJIR_DEBUG: DJIRS2 debug

*Note: This parameter is for advanced users*

Enable DJIRS2 debug

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Enabled with attitude reporting|

## DJIR_UPSIDEDOWN: DJIRS2 upside down

DJIRS2 upside down

|Value|Meaning|
|:---:|:---:|
|0|Right side up|
|1|Upside down|

## VIEP_DEBUG: ViewPro debug

*Note: This parameter is for advanced users*

ViewPro debug

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Enabled including attitude reporting|

## VIEP_CAM_SWLOW: ViewPro Camera For Switch Low

Camera selection when switch is in low position

|Value|Meaning|
|:---:|:---:|
|0|No change in camera selection|
|1|EO1|
|2|IR thermal|
|3|EO1 + IR Picture-in-picture|
|4|IR + EO1 Picture-in-picture|
|5|Fusion|
|6|IR1 13mm|
|7|IR2 52mm|

## VIEP_CAM_SWMID: ViewPro Camera For Switch Mid

Camera selection when switch is in middle position

|Value|Meaning|
|:---:|:---:|
|0|No change in camera selection|
|1|EO1|
|2|IR thermal|
|3|EO1 + IR Picture-in-picture|
|4|IR + EO1 Picture-in-picture|
|5|Fusion|
|6|IR1 13mm|
|7|IR2 52mm|

## VIEP_CAM_SWHIGH: ViewPro Camera For Switch High

Camera selection when switch is in high position

|Value|Meaning|
|:---:|:---:|
|0|No change in camera selection|
|1|EO1|
|2|IR thermal|
|3|EO1 + IR Picture-in-picture|
|4|IR + EO1 Picture-in-picture|
|5|Fusion|
|6|IR1 13mm|
|7|IR2 52mm|

## VIEP_ZOOM_SPEED: ViewPro Zoom Speed

ViewPro Zoom Speed.  Higher numbers result in faster zooming

- Range: 0 7

## VIEP_ZOOM_MAX: ViewPro Zoom Times Max

ViewPro Zoom Times Max

- Range: 0 30

## EFI_SP_ENABLE: Enable SkyPower EFI support

Enable SkyPower EFI support

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_SP_CANDRV: Set SkyPower EFI CAN driver

Set SkyPower EFI CAN driver

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|1stCANDriver|
|2|2ndCanDriver|

## EFI_SP_UPDATE_HZ: SkyPower EFI update rate

*Note: This parameter is for advanced users*

SkyPower EFI update rate

- Range: 10 200

- Units: Hz

## EFI_SP_THR_FN: SkyPower EFI throttle function

SkyPower EFI throttle function. This sets which SERVOn_FUNCTION to use for the target throttle. This should be 70 for fixed wing aircraft and 31 for helicopter rotor speed control

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|70|FixedWing|
|31|HeliRSC|

## EFI_SP_THR_RATE: SkyPower EFI throttle rate

*Note: This parameter is for advanced users*

SkyPower EFI throttle rate. This sets rate at which throttle updates are sent to the engine

- Range: 10 100

- Units: Hz

## EFI_SP_START_FN: SkyPower EFI start function

SkyPower EFI start function. This is the RCn_OPTION value to use to find the R/C channel used for controlling engine start

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|300|300|
|301|301|
|302|302|
|303|303|
|304|304|
|305|305|
|306|306|
|307|307|

## EFI_SP_GEN_FN: SkyPower EFI generator control function

SkyPower EFI generator control function. This is the RCn_OPTION value to use to find the R/C channel used for controlling generator start/stop

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|300|300|
|301|301|
|302|302|
|303|303|
|304|304|
|305|305|
|306|306|
|307|307|

## EFI_SP_MIN_RPM: SkyPower EFI minimum RPM

*Note: This parameter is for advanced users*

SkyPower EFI minimum RPM. This is the RPM below which the engine is considered to be stopped

- Range: 1 1000

## EFI_SP_TLM_RT: SkyPower EFI telemetry rate

*Note: This parameter is for advanced users*

SkyPower EFI telemetry rate. This is the rate at which extra telemetry values are sent to the GCS

- Range: 1 10

- Units: Hz

## EFI_SP_LOG_RT: SkyPower EFI log rate

*Note: This parameter is for advanced users*

SkyPower EFI log rate. This is the rate at which extra logging of the SkyPower EFI is performed

- Range: 1 50

- Units: Hz

## EFI_SP_ST_DISARM: SkyPower EFI allow start disarmed

SkyPower EFI allow start disarmed. This controls if starting the engine while disarmed is allowed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_H6K_ENABLE: Enable Halo6000 EFI driver

Enable Halo6000 EFI driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_H6K_CANDRV: Halo6000 CAN driver

Halo6000 CAN driver. Use 1 for first CAN scripting driver, 2 for 2nd driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|FirstCAN|
|2|SecondCAN|

## EFI_H6K_START_FN: Halo6000 start auxilliary function

The RC auxilliary function number for start/stop of the generator. Zero to disable start function

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|300|300|
|301|301|
|302|302|
|303|303|
|304|304|
|305|305|
|306|306|
|307|307|

## EFI_H6K_TELEM_RT: Halo6000 telemetry rate

The rate that additional generator telemetry is sent

- Units: Hz

## EFI_H6K_FUELTOT: Halo6000 total fuel capacity

The capacity of the tank in litres

- Units: litres

# AIS Parameters

## AIS_TYPE: AIS receiver type

AIS receiver type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|NMEA AIVDM message|

- RebootRequired: True

## AIS_LIST_MAX: AIS vessel list size

*Note: This parameter is for advanced users*

AIS list size of nearest vessels. Longer lists take longer to refresh with lower SRx_ADSB values.

- Range: 1 100

## AIS_TIME_OUT: AIS vessel time out

*Note: This parameter is for advanced users*

if no updates are received in this time a vessel will be removed from the list

- Units: s

- Range: 1 2000

## AIS_LOGGING: AIS logging options

*Note: This parameter is for advanced users*

Bitmask of AIS logging options

- Bitmask: 0:Log all AIVDM messages,1:Log only unsupported AIVDM messages,2:Log decoded messages

# ARSPD Parameters

## ARSPD_ENABLE: Airspeed Enable

Enable airspeed sensor support

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## ARSPD_TUBE_ORDER: Control pitot tube order

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD_PRIMARY: Primary airspeed sensor

*Note: This parameter is for advanced users*

This selects which airspeed sensor will be the primary if multiple sensors are found

|Value|Meaning|
|:---:|:---:|
|0|FirstSensor|
|1|2ndSensor|

## ARSPD_OPTIONS: Airspeed options bitmask

*Note: This parameter is for advanced users*

Bitmask of options to use with airspeed. 0:Disable use based on airspeed/groundspeed mismatch (see ARSPD_WIND_MAX), 1:Automatically reenable use based on airspeed/groundspeed mismatch recovery (see ARSPD_WIND_MAX) 2:Disable voltage correction, 3:Check that the airspeed is statistically consistent with the navigation EKF vehicle and wind velocity estimates using EKF3 (requires AHRS_EKF_TYPE = 3)

- Bitmask: 0:SpeedMismatchDisable, 1:AllowSpeedMismatchRecovery, 2:DisableVoltageCorrection, 3:UseEkf3Consistency

## ARSPD_WIND_MAX: Maximum airspeed and ground speed difference

*Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy. Using ARSPD_OPTION this health value can be used to disable the sensor.

- Units: m/s

## ARSPD_WIND_WARN: Airspeed and GPS speed difference that gives a warning

*Note: This parameter is for advanced users*

If the difference between airspeed and GPS speed is greater than this value the sensor will issue a warning. If 0 ARSPD_WIND_MAX is used.

- Units: m/s

## ARSPD_WIND_GATE: Re-enable Consistency Check Gate Size

*Note: This parameter is for advanced users*

Number of standard deviations applied to the re-enable EKF consistency check that is used when ARSPD_OPTIONS bit position 3 is set. Larger values will make the re-enabling of the airspeed sensor faster, but increase the likelihood of re-enabling a degraded sensor. The value can be tuned by using the ARSP.TR log message by setting ARSPD_WIND_GATE to a value that is higher than the value for ARSP.TR observed with a healthy airspeed sensor. Occasional transients in ARSP.TR above the value set by ARSPD_WIND_GATE can be tolerated provided they are less than 5 seconds in duration and less than 10% duty cycle.

- Range: 0.0 10.0

## ARSPD_OFF_PCNT: Maximum offset cal speed error 

*Note: This parameter is for advanced users*

The maximum percentage speed change in airspeed reports that is allowed due to offset changes between calibraions before a warning is issued. This potential speed error is in percent of ASPD_FBW_MIN. 0 disables. Helps warn of calibrations without pitot being covered.

- Range: 0.0 10.0

- Units: %

# ARSPD2 Parameters

## ARSPD2_TYPE: Airspeed type

Type of airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|I2C-MS4525D0|
|2|Analog|
|3|I2C-MS5525|
|4|I2C-MS5525 (0x76)|
|5|I2C-MS5525 (0x77)|
|6|I2C-SDP3X|
|7|I2C-DLVR-5in|
|8|DroneCAN|
|9|I2C-DLVR-10in|
|10|I2C-DLVR-20in|
|11|I2C-DLVR-30in|
|12|I2C-DLVR-60in|
|13|NMEA water speed|
|14|MSP|
|15|ASP5033|
|100|SITL|

## ARSPD2_USE: Airspeed use

Enables airspeed use for automatic throttle modes and replaces control from THR_TRIM. Continues to display and log airspeed if set to 0. Uses airspeed for control if set to 1. Only uses airspeed when throttle = 0 if set to 2 (useful for gliders with airspeed sensors behind propellers).

|Value|Meaning|
|:---:|:---:|
|0|DoNotUse|
|1|Use|
|2|UseWhenZeroThrottle|

## ARSPD2_OFFSET: Airspeed offset

*Note: This parameter is for advanced users*

Airspeed calibration offset

- Increment: 0.1

## ARSPD2_RATIO: Airspeed ratio

*Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.

- Increment: 0.1

## ARSPD2_PIN: Airspeed pin

*Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors. Set to 15 on the Pixhawk for the analog airspeed port. 

## ARSPD2_AUTOCAL: Automatic airspeed ratio calibration

*Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD2_TUBE_ORDR: Control pitot tube order

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD2_SKIP_CAL: Skip airspeed offset calibration on startup

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## ARSPD2_PSI_RANGE: The PSI range of the device

*Note: This parameter is for advanced users*

This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD2_BUS: Airspeed I2C bus

*Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected. May not correspond to board's I2C bus number labels. Retry another bus and reboot if airspeed sensor fails to initialize.

|Value|Meaning|
|:---:|:---:|
|0|Bus0|
|1|Bus1|
|2|Bus2|

- RebootRequired: True

## ARSPD2_DEVID: Airspeed ID

*Note: This parameter is for advanced users*

Airspeed sensor ID, taking into account its type, bus and instance

- ReadOnly: True

# ARSPD Parameters

## ARSPD_TYPE: Airspeed type

Type of airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|I2C-MS4525D0|
|2|Analog|
|3|I2C-MS5525|
|4|I2C-MS5525 (0x76)|
|5|I2C-MS5525 (0x77)|
|6|I2C-SDP3X|
|7|I2C-DLVR-5in|
|8|DroneCAN|
|9|I2C-DLVR-10in|
|10|I2C-DLVR-20in|
|11|I2C-DLVR-30in|
|12|I2C-DLVR-60in|
|13|NMEA water speed|
|14|MSP|
|15|ASP5033|
|100|SITL|

## ARSPD_USE: Airspeed use

Enables airspeed use for automatic throttle modes and replaces control from THR_TRIM. Continues to display and log airspeed if set to 0. Uses airspeed for control if set to 1. Only uses airspeed when throttle = 0 if set to 2 (useful for gliders with airspeed sensors behind propellers).

|Value|Meaning|
|:---:|:---:|
|0|DoNotUse|
|1|Use|
|2|UseWhenZeroThrottle|

## ARSPD_OFFSET: Airspeed offset

*Note: This parameter is for advanced users*

Airspeed calibration offset

- Increment: 0.1

## ARSPD_RATIO: Airspeed ratio

*Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.

- Increment: 0.1

## ARSPD_PIN: Airspeed pin

*Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors. Set to 15 on the Pixhawk for the analog airspeed port. 

## ARSPD_AUTOCAL: Automatic airspeed ratio calibration

*Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD_TUBE_ORDR: Control pitot tube order

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD_SKIP_CAL: Skip airspeed offset calibration on startup

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## ARSPD_PSI_RANGE: The PSI range of the device

*Note: This parameter is for advanced users*

This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD_BUS: Airspeed I2C bus

*Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected. May not correspond to board's I2C bus number labels. Retry another bus and reboot if airspeed sensor fails to initialize.

|Value|Meaning|
|:---:|:---:|
|0|Bus0|
|1|Bus1|
|2|Bus2|

- RebootRequired: True

## ARSPD_DEVID: Airspeed ID

*Note: This parameter is for advanced users*

Airspeed sensor ID, taking into account its type, bus and instance

- ReadOnly: True

# BARO Parameters

## BARO1_GND_PRESS: Ground Pressure

*Note: This parameter is for advanced users*

calibrated ground pressure in Pascals

- Units: Pa

- Increment: 1

- ReadOnly: True

- Volatile: True

## BARO_GND_TEMP: ground temperature

*Note: This parameter is for advanced users*

User provided ambient ground temperature in degrees Celsius. This is used to improve the calculation of the altitude the vehicle is at. This parameter is not persistent and will be reset to 0 every time the vehicle is rebooted. A value of 0 means use the internal measurement ambient temperature.

- Units: degC

- Increment: 1

- Volatile: True

## BARO_ALT_OFFSET: altitude offset

*Note: This parameter is for advanced users*

altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.

- Units: m

- Increment: 0.1

## BARO_PRIMARY: Primary barometer

*Note: This parameter is for advanced users*

This selects which barometer will be the primary if multiple barometers are found

|Value|Meaning|
|:---:|:---:|
|0|FirstBaro|
|1|2ndBaro|
|2|3rdBaro|

## BARO_EXT_BUS: External baro bus

*Note: This parameter is for advanced users*

This selects the bus number for looking for an I2C barometer. When set to -1 it will probe all external i2c buses based on the BARO_PROBE_EXT parameter.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Bus0|
|1|Bus1|
|6|Bus6|

## BARO2_GND_PRESS: Ground Pressure

*Note: This parameter is for advanced users*

calibrated ground pressure in Pascals

- Units: Pa

- Increment: 1

- ReadOnly: True

- Volatile: True

## BARO3_GND_PRESS: Absolute Pressure

*Note: This parameter is for advanced users*

calibrated ground pressure in Pascals

- Units: Pa

- Increment: 1

- ReadOnly: True

- Volatile: True

## BARO_FLTR_RNG: Range in which sample is accepted

This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables. The value is a percentage from the average value. A value of zero disables this filter.

- Units: %

- Range: 0 100

- Increment: 1

## BARO_PROBE_EXT: External barometers to probe

*Note: This parameter is for advanced users*

This sets which types of external i2c barometer to look for. It is a bitmask of barometer types. The I2C buses to probe is based on BARO_EXT_BUS. If BARO_EXT_BUS is -1 then it will probe all external buses, otherwise it will probe just the bus number given in BARO_EXT_BUS.

- Bitmask: 0:BMP085,1:BMP280,2:MS5611,3:MS5607,4:MS5637,5:FBM320,6:DPS280,7:LPS25H,8:Keller,9:MS5837,10:BMP388,11:SPL06,12:MSP

## BARO1_DEVID: Baro ID

*Note: This parameter is for advanced users*

Barometer sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## BARO2_DEVID: Baro ID2

*Note: This parameter is for advanced users*

Barometer2 sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## BARO3_DEVID: Baro ID3

*Note: This parameter is for advanced users*

Barometer3 sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## BARO_FIELD_ELV: field elevation

*Note: This parameter is for advanced users*

User provided field elevation in meters. This is used to improve the calculation of the altitude the vehicle is at. This parameter is not persistent and will be reset to 0 every time the vehicle is rebooted. Changes to this parameter will only be used when disarmed. A value of 0 means the EKF origin height is used for takeoff height above sea level.

- Units: m

- Increment: 0.1

- Volatile: True

## BARO_ALTERR_MAX: Altitude error maximum

*Note: This parameter is for advanced users*

This is the maximum acceptable altitude discrepancy between GPS altitude and barometric presssure altitude calculated against a standard atmosphere for arming checks to pass. If you are getting an arming error due to this parameter then you may have a faulty or substituted barometer. A common issue is vendors replacing a MS5611 in a "Pixhawk" with a MS5607. If you have that issue then please see BARO_OPTIONS parameter to force the MS5611 to be treated as a MS5607. This check is disabled if the value is zero.

- Units: m

- Increment: 1

- Range: 0 5000

## BARO_OPTIONS: Barometer options

*Note: This parameter is for advanced users*

Barometer options

- Bitmask: 0:Treat MS5611 as MS5607

# BARO1WCF Parameters

## BARO1_WCF_ENABLE: Wind coefficient enable

*Note: This parameter is for advanced users*

This enables the use of wind coefficients for barometer compensation

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BARO1_WCF_FWD: Pressure error coefficient in positive X direction (forward)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis. If the baro height estimate rises during forwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO1_WCF_BCK: Pressure error coefficient in negative X direction (backwards)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis. If the baro height estimate rises during backwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO1_WCF_RGT: Pressure error coefficient in positive Y direction (right)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the right, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO1_WCF_LFT: Pressure error coefficient in negative Y direction (left)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the left, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO1_WCF_UP: Pressure error coefficient in positive Z direction (up)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Z body axis. If the baro height estimate rises above truth height during climbing flight (or forward flight with a high forwards lean angle), then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO1_WCF_DN: Pressure error coefficient in negative Z direction (down)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Z body axis. If the baro height estimate rises above truth height during descending flight (or forward flight with a high backwards lean angle, eg braking manoeuvre), then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

# BARO2WCF Parameters

## BARO2_WCF_ENABLE: Wind coefficient enable

*Note: This parameter is for advanced users*

This enables the use of wind coefficients for barometer compensation

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BARO2_WCF_FWD: Pressure error coefficient in positive X direction (forward)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis. If the baro height estimate rises during forwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO2_WCF_BCK: Pressure error coefficient in negative X direction (backwards)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis. If the baro height estimate rises during backwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO2_WCF_RGT: Pressure error coefficient in positive Y direction (right)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the right, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO2_WCF_LFT: Pressure error coefficient in negative Y direction (left)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the left, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO2_WCF_UP: Pressure error coefficient in positive Z direction (up)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Z body axis. If the baro height estimate rises above truth height during climbing flight (or forward flight with a high forwards lean angle), then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO2_WCF_DN: Pressure error coefficient in negative Z direction (down)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Z body axis. If the baro height estimate rises above truth height during descending flight (or forward flight with a high backwards lean angle, eg braking manoeuvre), then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

# BARO3WCF Parameters

## BARO3_WCF_ENABLE: Wind coefficient enable

*Note: This parameter is for advanced users*

This enables the use of wind coefficients for barometer compensation

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BARO3_WCF_FWD: Pressure error coefficient in positive X direction (forward)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis. If the baro height estimate rises during forwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO3_WCF_BCK: Pressure error coefficient in negative X direction (backwards)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis. If the baro height estimate rises during backwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO3_WCF_RGT: Pressure error coefficient in positive Y direction (right)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the right, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO3_WCF_LFT: Pressure error coefficient in negative Y direction (left)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the left, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO3_WCF_UP: Pressure error coefficient in positive Z direction (up)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Z body axis. If the baro height estimate rises above truth height during climbing flight (or forward flight with a high forwards lean angle), then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BARO3_WCF_DN: Pressure error coefficient in negative Z direction (down)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Z body axis. If the baro height estimate rises above truth height during descending flight (or forward flight with a high backwards lean angle, eg braking manoeuvre), then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

# BATT2 Parameters

## BATT2_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT2_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT2_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT2_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT2_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT2_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT2_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT2_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT2_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT2_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT2_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT2_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT2_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT2_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT2__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT2_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT2_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT2__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT2_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT2_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT2_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT2_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT2_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT2_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT2_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT2_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT2_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT2_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT2_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT2_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT2_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT2_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT2_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT2_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT2_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT2_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT2_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT2_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT3 Parameters

## BATT3_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT3_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT3_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT3_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT3_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT3_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT3_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT3_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT3_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT3_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT3_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT3_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT3_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT3_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT3__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT3_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT3_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT3__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT3_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT3_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT3_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT3_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT3_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT3_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT3_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT3_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT3_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT3_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT3_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT3_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT3_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT3_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT3_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT3_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT3_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT3_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT3_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT3_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT4 Parameters

## BATT4_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT4_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT4_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT4_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT4_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT4_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT4_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT4_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT4_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT4_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT4_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT4_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT4_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT4_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT4__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT4_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT4_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT4__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT4_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT4_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT4_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT4_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT4_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT4_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT4_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT4_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT4_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT4_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT4_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT4_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT4_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT4_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT4_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT4_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT4_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT4_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT4_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT4_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT5 Parameters

## BATT5_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT5_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT5_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT5_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT5_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT5_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT5_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT5_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT5_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT5_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT5_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT5_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT5_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT5_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT5__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT5_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT5_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT5__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT5_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT5_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT5_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT5_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT5_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT5_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT5_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT5_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT5_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT5_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT5_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT5_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT5_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT5_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT5_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT5_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT5_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT5_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT5_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT5_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT6 Parameters

## BATT6_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT6_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT6_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT6_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT6_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT6_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT6_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT6_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT6_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT6_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT6_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT6_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT6_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT6_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT6__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT6_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT6_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT6__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT6_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT6_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT6_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT6_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT6_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT6_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT6_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT6_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT6_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT6_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT6_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT6_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT6_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT6_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT6_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT6_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT6_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT6_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT6_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT6_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT7 Parameters

## BATT7_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT7_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT7_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT7_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT7_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT7_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT7_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT7_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT7_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT7_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT7_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT7_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT7_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT7_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT7__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT7_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT7_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT7__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT7_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT7_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT7_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT7_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT7_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT7_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT7_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT7_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT7_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT7_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT7_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT7_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT7_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT7_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT7_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT7_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT7_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT7_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT7_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT7_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT8 Parameters

## BATT8_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT8_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT8_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT8_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT8_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT8_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT8_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT8_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT8_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT8_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT8_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT8_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT8_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT8_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT8__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT8_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT8_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT8__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT8_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT8_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT8_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT8_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT8_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT8_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT8_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT8_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT8_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT8_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT8_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT8_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT8_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT8_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT8_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT8_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT8_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT8_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT8_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT8_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT9 Parameters

## BATT9_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT9_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT9_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT9_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT9_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT9_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT9_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT9_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT9_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT9_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT9_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT9_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT9_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT9_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT9__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT9_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT9_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT9__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT9_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT9_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT9_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT9_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT9_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT9_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT9_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT9_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT9_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT9_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT9_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT9_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT9_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT9_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT9_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT9_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT9_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT9_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT9_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT9_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# BATT Parameters

## BATT_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|DroneCAN-BatteryInfo|
|9|ESC|
|10|Sum Of Selected Monitors|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|
|20|MPPT|
|21|INA2XX|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|

- RebootRequired: True

## BATT_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATT_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATT_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATT_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATT_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATT_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATT_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS

## BATT_VOLT_PIN: Battery Voltage sensing pin

Sets the analog input pin that should be used for voltage monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- RebootRequired: True

## BATT_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for current monitoring.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|3|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|4|CubeOrange_PM2/Navigator|
|14|Pixhawk2_PM2|
|15|CubeOrange|
|17|Durandal|
|101|PX4-v1|

- RebootRequired: True

## BATT_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATT_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATT_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATT_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATT_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATT_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATT_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: .1 10

## BATT_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATT_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATT_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATT_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to. Airspeed ports can be used for Analog input. When using analog pin 103, the maximum value of the input in 3.3V.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|103|Pixhawk SBUS|

## BATT_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATT_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATT_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATT_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

- RebootRequired: True

# CAMRC Parameters

## CAM_RC_TYPE: RunCam device type

RunCam deviee type used to determine OSD menu structure and shutter options.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|RunCam Split Micro/RunCam with UART|
|2|RunCam Split|
|3|RunCam Split4 4k|
|4|RunCam Hybrid/RunCam Thumb Pro|
|5|Runcam 2 4k|

## CAM_RC_FEATURES: RunCam features available

*Note: This parameter is for advanced users*

The available features of the attached RunCam device. If 0 then the RunCam device will be queried for the features it supports, otherwise this setting is used.

- Bitmask: 0:Power Button,1:WiFi Button,2:Change Mode,3:5-Key OSD,4:Settings Access,5:DisplayPort,6:Start Recording,7:Stop Recording

## CAM_RC_BT_DELAY: RunCam boot delay before allowing updates

*Note: This parameter is for advanced users*

Time it takes for the RunCam to become fully ready in ms. If this is too short then commands can get out of sync.

## CAM_RC_BTN_DELAY: RunCam button delay before allowing further button presses

*Note: This parameter is for advanced users*

Time it takes for the a RunCam button press to be actived in ms. If this is too short then commands can get out of sync.

## CAM_RC_MDE_DELAY: RunCam mode delay before allowing further button presses

*Note: This parameter is for advanced users*

Time it takes for the a RunCam mode button press to be actived in ms. If a mode change first requires a video recording change then double this value is used. If this is too short then commands can get out of sync.

## CAM_RC_CONTROL: RunCam control option

*Note: This parameter is for advanced users*

Specifies the allowed actions required to enter the OSD menu and other option like autorecording

- Bitmask: 0:Stick yaw right,1:Stick roll right,2:3-position switch,3:2-position switch,4:Autorecording enabled

# COMPASS Parameters

## COMPASS_OFS_X: Compass offsets in milligauss on the X axis

*Note: This parameter is for advanced users*

Offset to be added to the compass x-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

- Calibration: 1

## COMPASS_OFS_Y: Compass offsets in milligauss on the Y axis

*Note: This parameter is for advanced users*

Offset to be added to the compass y-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

- Calibration: 1

## COMPASS_OFS_Z: Compass offsets in milligauss on the Z axis

*Note: This parameter is for advanced users*

Offset to be added to the compass z-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

## COMPASS_DEC: Compass declination

An angle to compensate between the true north and magnetic north

- Range: -3.142 3.142

- Units: rad

- Increment: 0.01

## COMPASS_LEARN: Learn compass offsets automatically

*Note: This parameter is for advanced users*

Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle. If InFlight learning is enabled then the compass with automatically start learning once a flight starts (must be armed). While InFlight learning is running you cannot use position control modes.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Internal-Learning|
|2|EKF-Learning|
|3|InFlight-Learning|

## COMPASS_USE: Use compass for yaw

*Note: This parameter is for advanced users*

Enable or disable the use of the compass (instead of the GPS) for determining heading

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## COMPASS_AUTODEC: Auto Declination

*Note: This parameter is for advanced users*

Enable or disable the automatic calculation of the declination based on gps location

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## COMPASS_MOTCT: Motor interference compensation type

*Note: This parameter is for advanced users*

Set motor interference compensation type to disabled, throttle or current.  Do not change manually.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Use Throttle|
|2|Use Current|

- Calibration: 1

## COMPASS_MOT_X: Motor interference compensation for body frame X axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

- Calibration: 1

## COMPASS_MOT_Y: Motor interference compensation for body frame Y axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

- Calibration: 1

## COMPASS_MOT_Z: Motor interference compensation for body frame Z axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

## COMPASS_ORIENT: Compass orientation

*Note: This parameter is for advanced users*

The orientation of the first external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used. The label for each option is specified in the order of rotations for that orientation. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the COMPASS_CUS_ROLL/PIT/YAW angles for Compass orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_1_ROLL/PIT/YAW or CUST_2_ROLL/PIT/YAW angles.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Yaw45|
|2|Yaw90|
|3|Yaw135|
|4|Yaw180|
|5|Yaw225|
|6|Yaw270|
|7|Yaw315|
|8|Roll180|
|9|Yaw45Roll180|
|10|Yaw90Roll180|
|11|Yaw135Roll180|
|12|Pitch180|
|13|Yaw225Roll180|
|14|Yaw270Roll180|
|15|Yaw315Roll180|
|16|Roll90|
|17|Yaw45Roll90|
|18|Yaw90Roll90|
|19|Yaw135Roll90|
|20|Roll270|
|21|Yaw45Roll270|
|22|Yaw90Roll270|
|23|Yaw135Roll270|
|24|Pitch90|
|25|Pitch270|
|26|Yaw90Pitch180|
|27|Yaw270Pitch180|
|28|Pitch90Roll90|
|29|Pitch90Roll180|
|30|Pitch90Roll270|
|31|Pitch180Roll90|
|32|Pitch180Roll270|
|33|Pitch270Roll90|
|34|Pitch270Roll180|
|35|Pitch270Roll270|
|36|Yaw90Pitch180Roll90|
|37|Yaw270Roll90|
|38|Yaw293Pitch68Roll180|
|39|Pitch315|
|40|Pitch315Roll90|
|42|Roll45|
|43|Roll315|
|100|Custom 4.1 and older|
|101|Custom 1|
|102|Custom 2|

## COMPASS_EXTERNAL: Compass is attached via an external cable

*Note: This parameter is for advanced users*

Configure compass so it is attached externally. This is auto-detected on most boards. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.

|Value|Meaning|
|:---:|:---:|
|0|Internal|
|1|External|
|2|ForcedExternal|

## COMPASS_OFS2_X: Compass2 offsets in milligauss on the X axis

*Note: This parameter is for advanced users*

Offset to be added to compass2's x-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

- Calibration: 1

## COMPASS_OFS2_Y: Compass2 offsets in milligauss on the Y axis

*Note: This parameter is for advanced users*

Offset to be added to compass2's y-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

- Calibration: 1

## COMPASS_OFS2_Z: Compass2 offsets in milligauss on the Z axis

*Note: This parameter is for advanced users*

Offset to be added to compass2's z-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

## COMPASS_MOT2_X: Motor interference compensation to compass2 for body frame X axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

- Calibration: 1

## COMPASS_MOT2_Y: Motor interference compensation to compass2 for body frame Y axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

- Calibration: 1

## COMPASS_MOT2_Z: Motor interference compensation to compass2 for body frame Z axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

## COMPASS_OFS3_X: Compass3 offsets in milligauss on the X axis

*Note: This parameter is for advanced users*

Offset to be added to compass3's x-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

- Calibration: 1

## COMPASS_OFS3_Y: Compass3 offsets in milligauss on the Y axis

*Note: This parameter is for advanced users*

Offset to be added to compass3's y-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

- Calibration: 1

## COMPASS_OFS3_Z: Compass3 offsets in milligauss on the Z axis

*Note: This parameter is for advanced users*

Offset to be added to compass3's z-axis values to compensate for metal in the frame

- Range: -400 400

- Units: mGauss

- Increment: 1

## COMPASS_MOT3_X: Motor interference compensation to compass3 for body frame X axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

- Calibration: 1

## COMPASS_MOT3_Y: Motor interference compensation to compass3 for body frame Y axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

- Calibration: 1

## COMPASS_MOT3_Z: Motor interference compensation to compass3 for body frame Z axis

*Note: This parameter is for advanced users*

Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)

- Range: -1000 1000

- Units: mGauss/A

- Increment: 1

## COMPASS_DEV_ID: Compass device id

*Note: This parameter is for advanced users*

Compass device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_DEV_ID2: Compass2 device id

*Note: This parameter is for advanced users*

Second compass's device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_DEV_ID3: Compass3 device id

*Note: This parameter is for advanced users*

Third compass's device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_USE2: Compass2 used for yaw

*Note: This parameter is for advanced users*

Enable or disable the secondary compass for determining heading.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## COMPASS_ORIENT2: Compass2 orientation

*Note: This parameter is for advanced users*

The orientation of a second external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used. The label for each option is specified in the order of rotations for that orientation. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the COMPASS_CUS_ROLL/PIT/YAW angles for Compass orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_1_ROLL/PIT/YAW or CUST_2_ROLL/PIT/YAW angles.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Yaw45|
|2|Yaw90|
|3|Yaw135|
|4|Yaw180|
|5|Yaw225|
|6|Yaw270|
|7|Yaw315|
|8|Roll180|
|9|Yaw45Roll180|
|10|Yaw90Roll180|
|11|Yaw135Roll180|
|12|Pitch180|
|13|Yaw225Roll180|
|14|Yaw270Roll180|
|15|Yaw315Roll180|
|16|Roll90|
|17|Yaw45Roll90|
|18|Yaw90Roll90|
|19|Yaw135Roll90|
|20|Roll270|
|21|Yaw45Roll270|
|22|Yaw90Roll270|
|23|Yaw135Roll270|
|24|Pitch90|
|25|Pitch270|
|26|Yaw90Pitch180|
|27|Yaw270Pitch180|
|28|Pitch90Roll90|
|29|Pitch90Roll180|
|30|Pitch90Roll270|
|31|Pitch180Roll90|
|32|Pitch180Roll270|
|33|Pitch270Roll90|
|34|Pitch270Roll180|
|35|Pitch270Roll270|
|36|Yaw90Pitch180Roll90|
|37|Yaw270Roll90|
|38|Yaw293Pitch68Roll180|
|39|Pitch315|
|40|Pitch315Roll90|
|42|Roll45|
|43|Roll315|
|100|Custom 4.1 and older|
|101|Custom 1|
|102|Custom 2|

## COMPASS_EXTERN2: Compass2 is attached via an external cable

*Note: This parameter is for advanced users*

Configure second compass so it is attached externally. This is auto-detected on most boards. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.

|Value|Meaning|
|:---:|:---:|
|0|Internal|
|1|External|
|2|ForcedExternal|

## COMPASS_USE3: Compass3 used for yaw

*Note: This parameter is for advanced users*

Enable or disable the tertiary compass for determining heading.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## COMPASS_ORIENT3: Compass3 orientation

*Note: This parameter is for advanced users*

The orientation of a third external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used. The label for each option is specified in the order of rotations for that orientation. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the COMPASS_CUS_ROLL/PIT/YAW angles for Compass orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_1_ROLL/PIT/YAW or CUST_2_ROLL/PIT/YAW angles.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Yaw45|
|2|Yaw90|
|3|Yaw135|
|4|Yaw180|
|5|Yaw225|
|6|Yaw270|
|7|Yaw315|
|8|Roll180|
|9|Yaw45Roll180|
|10|Yaw90Roll180|
|11|Yaw135Roll180|
|12|Pitch180|
|13|Yaw225Roll180|
|14|Yaw270Roll180|
|15|Yaw315Roll180|
|16|Roll90|
|17|Yaw45Roll90|
|18|Yaw90Roll90|
|19|Yaw135Roll90|
|20|Roll270|
|21|Yaw45Roll270|
|22|Yaw90Roll270|
|23|Yaw135Roll270|
|24|Pitch90|
|25|Pitch270|
|26|Yaw90Pitch180|
|27|Yaw270Pitch180|
|28|Pitch90Roll90|
|29|Pitch90Roll180|
|30|Pitch90Roll270|
|31|Pitch180Roll90|
|32|Pitch180Roll270|
|33|Pitch270Roll90|
|34|Pitch270Roll180|
|35|Pitch270Roll270|
|36|Yaw90Pitch180Roll90|
|37|Yaw270Roll90|
|38|Yaw293Pitch68Roll180|
|39|Pitch315|
|40|Pitch315Roll90|
|42|Roll45|
|43|Roll315|
|100|Custom 4.1 and older|
|101|Custom 1|
|102|Custom 2|

## COMPASS_EXTERN3: Compass3 is attached via an external cable

*Note: This parameter is for advanced users*

Configure third compass so it is attached externally. This is auto-detected on most boards. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.

|Value|Meaning|
|:---:|:---:|
|0|Internal|
|1|External|
|2|ForcedExternal|

## COMPASS_DIA_X: Compass soft-iron diagonal X component

*Note: This parameter is for advanced users*

DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_DIA_Y: Compass soft-iron diagonal Y component

*Note: This parameter is for advanced users*

DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_DIA_Z: Compass soft-iron diagonal Z component

*Note: This parameter is for advanced users*

DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

## COMPASS_ODI_X: Compass soft-iron off-diagonal X component

*Note: This parameter is for advanced users*

ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_ODI_Y: Compass soft-iron off-diagonal Y component

*Note: This parameter is for advanced users*

ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_ODI_Z: Compass soft-iron off-diagonal Z component

*Note: This parameter is for advanced users*

ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

## COMPASS_DIA2_X: Compass2 soft-iron diagonal X component

*Note: This parameter is for advanced users*

DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_DIA2_Y: Compass2 soft-iron diagonal Y component

*Note: This parameter is for advanced users*

DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_DIA2_Z: Compass2 soft-iron diagonal Z component

*Note: This parameter is for advanced users*

DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

## COMPASS_ODI2_X: Compass2 soft-iron off-diagonal X component

*Note: This parameter is for advanced users*

ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_ODI2_Y: Compass2 soft-iron off-diagonal Y component

*Note: This parameter is for advanced users*

ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_ODI2_Z: Compass2 soft-iron off-diagonal Z component

*Note: This parameter is for advanced users*

ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

## COMPASS_DIA3_X: Compass3 soft-iron diagonal X component

*Note: This parameter is for advanced users*

DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_DIA3_Y: Compass3 soft-iron diagonal Y component

*Note: This parameter is for advanced users*

DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_DIA3_Z: Compass3 soft-iron diagonal Z component

*Note: This parameter is for advanced users*

DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

## COMPASS_ODI3_X: Compass3 soft-iron off-diagonal X component

*Note: This parameter is for advanced users*

ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_ODI3_Y: Compass3 soft-iron off-diagonal Y component

*Note: This parameter is for advanced users*

ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

- Calibration: 1

## COMPASS_ODI3_Z: Compass3 soft-iron off-diagonal Z component

*Note: This parameter is for advanced users*

ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]

## COMPASS_CAL_FIT: Compass calibration fitness

*Note: This parameter is for advanced users*

This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass). This is the value used for the primary magnetometer. Other magnetometers get double the value.

- Range: 4 32

|Value|Meaning|
|:---:|:---:|
|4|Very Strict|
|8|Strict|
|16|Default|
|32|Relaxed|

- Increment: 0.1

## COMPASS_OFFS_MAX: Compass maximum offset

*Note: This parameter is for advanced users*

This sets the maximum allowed compass offset in calibration and arming checks

- Range: 500 3000

- Increment: 1

## COMPASS_TYPEMASK: Compass disable driver type mask

*Note: This parameter is for advanced users*

This is a bitmask of driver types to disable. If a driver type is set in this mask then that driver will not try to find a sensor at startup

- Bitmask: 0:HMC5883,1:LSM303D,2:AK8963,3:BMM150,4:LSM9DS1,5:LIS3MDL,6:AK09916,7:IST8310,8:ICM20948,9:MMC3416,11:DroneCAN,12:QMC5883,14:MAG3110,15:IST8308,16:RM3100,17:MSP,18:ExternalAHRS

## COMPASS_FLTR_RNG: Range in which sample is accepted

This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables. The value is a percentage from the average value. A value of zero disables this filter.

- Units: %

- Range: 0 100

- Increment: 1

## COMPASS_AUTO_ROT: Automatically check orientation

When enabled this will automatically check the orientation of compasses on successful completion of compass calibration. If set to 2 then external compasses will have their orientation automatically corrected.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|CheckOnly|
|2|CheckAndFix|
|3|use same tolerance to auto rotate 45 deg rotations|

## COMPASS_PRIO1_ID: Compass device id with 1st order priority

*Note: This parameter is for advanced users*

Compass device id with 1st order priority, set automatically if 0. Reboot required after change.

- RebootRequired: True

## COMPASS_PRIO2_ID: Compass device id with 2nd order priority

*Note: This parameter is for advanced users*

Compass device id with 2nd order priority, set automatically if 0. Reboot required after change.

- RebootRequired: True

## COMPASS_PRIO3_ID: Compass device id with 3rd order priority

*Note: This parameter is for advanced users*

Compass device id with 3rd order priority, set automatically if 0. Reboot required after change.

- RebootRequired: True

## COMPASS_ENABLE: Enable Compass

Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass. Note that this is separate from COMPASS_USE. This will enable the low level senor, and will enable logging of magnetometer data. To use the compass for navigation you must also set COMPASS_USE to 1.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## COMPASS_SCALE: Compass1 scale factor

Scaling factor for first compass to compensate for sensor scaling errors. If this is 0 then no scaling is done

- Range: 0 1.3

## COMPASS_SCALE2: Compass2 scale factor

Scaling factor for 2nd compass to compensate for sensor scaling errors. If this is 0 then no scaling is done

- Range: 0 1.3

## COMPASS_SCALE3: Compass3 scale factor

Scaling factor for 3rd compass to compensate for sensor scaling errors. If this is 0 then no scaling is done

- Range: 0 1.3

## COMPASS_OPTIONS: Compass options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the compass

- Bitmask: 0:CalRequireGPS, 1: Allow missing DroneCAN compasses to be automaticaly replaced (calibration still required)

## COMPASS_DEV_ID4: Compass4 device id

*Note: This parameter is for advanced users*

Extra 4th compass's device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_DEV_ID5: Compass5 device id

*Note: This parameter is for advanced users*

Extra 5th compass's device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_DEV_ID6: Compass6 device id

*Note: This parameter is for advanced users*

Extra 6th compass's device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_DEV_ID7: Compass7 device id

*Note: This parameter is for advanced users*

Extra 7th compass's device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_DEV_ID8: Compass8 device id

*Note: This parameter is for advanced users*

Extra 8th compass's device id.  Automatically detected, do not set manually

- ReadOnly: True

## COMPASS_CUS_ROLL: Custom orientation roll offset

*Note: This parameter is for advanced users*

Compass mounting position roll offset. Positive values = roll right, negative values = roll left. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.

- Range: -180 180

- Units: deg

- Increment: 1

- RebootRequired: True

## COMPASS_CUS_PIT: Custom orientation pitch offset

*Note: This parameter is for advanced users*

Compass mounting position pitch offset. Positive values = pitch up, negative values = pitch down. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.

- Range: -180 180

- Units: deg

- Increment: 1

- RebootRequired: True

## COMPASS_CUS_YAW: Custom orientation yaw offset

*Note: This parameter is for advanced users*

Compass mounting position yaw offset. Positive values = yaw right, negative values = yaw left. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.

- Range: -180 180

- Units: deg

- Increment: 1

- RebootRequired: True

# COMPASSPMOT Parameters

## COMPASS_PMOT_EN: per-motor compass correction enable

*Note: This parameter is for advanced users*

This enables per-motor compass corrections

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## COMPASS_PMOT_EXP: per-motor exponential correction

*Note: This parameter is for advanced users*

This is the exponential correction for the power output of the motor for per-motor compass correction

- Range: 0 2

- Increment: 0.01

## COMPASS_PMOT1_X: Compass per-motor1 X

*Note: This parameter is for advanced users*

Compensation for X axis of motor1

## COMPASS_PMOT1_Y: Compass per-motor1 Y

*Note: This parameter is for advanced users*

Compensation for Y axis of motor1

## COMPASS_PMOT1_Z: Compass per-motor1 Z

*Note: This parameter is for advanced users*

Compensation for Z axis of motor1

## COMPASS_PMOT2_X: Compass per-motor2 X

*Note: This parameter is for advanced users*

Compensation for X axis of motor2

## COMPASS_PMOT2_Y: Compass per-motor2 Y

*Note: This parameter is for advanced users*

Compensation for Y axis of motor2

## COMPASS_PMOT2_Z: Compass per-motor2 Z

*Note: This parameter is for advanced users*

Compensation for Z axis of motor2

## COMPASS_PMOT3_X: Compass per-motor3 X

*Note: This parameter is for advanced users*

Compensation for X axis of motor3

## COMPASS_PMOT3_Y: Compass per-motor3 Y

*Note: This parameter is for advanced users*

Compensation for Y axis of motor3

## COMPASS_PMOT3_Z: Compass per-motor3 Z

*Note: This parameter is for advanced users*

Compensation for Z axis of motor3

## COMPASS_PMOT4_X: Compass per-motor4 X

*Note: This parameter is for advanced users*

Compensation for X axis of motor4

## COMPASS_PMOT4_Y: Compass per-motor4 Y

*Note: This parameter is for advanced users*

Compensation for Y axis of motor4

## COMPASS_PMOT4_Z: Compass per-motor4 Z

*Note: This parameter is for advanced users*

Compensation for Z axis of motor4

# CUSTROT Parameters

## CUST_ROT_ENABLE: Enable Custom rotations

This enables custom rotations

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

- RebootRequired: True

# CUSTROT1 Parameters

## CUST_ROT1_ROLL: Custom roll

Custom euler roll, euler 321 (yaw, pitch, roll) ordering

- Units: deg

- RebootRequired: True

## CUST_ROT1_PITCH: Custom pitch

Custom euler pitch, euler 321 (yaw, pitch, roll) ordering

- Units: deg

- RebootRequired: True

## CUST_ROT1_YAW: Custom yaw

Custom euler yaw, euler 321 (yaw, pitch, roll) ordering

- Units: deg

- RebootRequired: True

# CUSTROT2 Parameters

## CUST_ROT2_ROLL: Custom roll

Custom euler roll, euler 321 (yaw, pitch, roll) ordering

- Units: deg

- RebootRequired: True

## CUST_ROT2_PITCH: Custom pitch

Custom euler pitch, euler 321 (yaw, pitch, roll) ordering

- Units: deg

- RebootRequired: True

## CUST_ROT2_YAW: Custom yaw

Custom euler yaw, euler 321 (yaw, pitch, roll) ordering

- Units: deg

- RebootRequired: True

# DDS Parameters

## DDS_ENABLE: DDS enable

*Note: This parameter is for advanced users*

Enable DDS subsystem

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## DDS_UDP_PORT: DDS UDP port

UDP port number for DDS

- Range: 1 65535

- RebootRequired: True

# DID Parameters

## DID_ENABLE: Enable ODID subsystem

Enable ODID subsystem

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## DID_MAVPORT: MAVLink serial port

Serial port number to send OpenDroneID MAVLink messages to. Can be -1 if using DroneCAN.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Serial0|
|1|Serial1|
|2|Serial2|
|3|Serial3|
|4|Serial4|
|5|Serial5|
|6|Serial6|

## DID_CANDRIVER: DroneCAN driver number

DroneCAN driver index, 0 to disable DroneCAN

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Driver1|
|2|Driver2|

## DID_OPTIONS: OpenDroneID options

Options for OpenDroneID subsystem

- Bitmask: 0:EnforceArming, 1:AllowNonGPSPosition, 2:LockUASIDOnFirstBasicIDRx

## DID_BARO_ACC: Barometer vertical accuraacy

*Note: This parameter is for advanced users*

Barometer Vertical Accuracy when installed in the vehicle. Note this is dependent upon installation conditions and thus disabled by default

- Units: m

# EAHRS Parameters

## EAHRS_TYPE: AHRS type

Type of AHRS device

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|VectorNav|
|2|LORD|

## EAHRS_RATE: AHRS data rate

Requested rate for AHRS device

- Units: Hz

## EAHRS_OPTIONS: External AHRS options

External AHRS options bitmask

- Bitmask: 0:Vector Nav use uncompensated values for accel gyro and mag.

## EAHRS_SENSORS: External AHRS sensors

*Note: This parameter is for advanced users*

External AHRS sensors bitmask

- Bitmask: 0:GPS,1:IMU,2:Baro,3:Compass

# EFI Parameters

## EFI_TYPE: EFI communication type

*Note: This parameter is for advanced users*

What method of communication is used for EFI #1

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Serial-MS|
|2|NWPMU|
|3|Serial-Lutan|
|5|DroneCAN|
|6|Currawong-ECU|
|7|Scripting|
|9|MAV|

- RebootRequired: True

## EFI_COEF1: EFI Calibration Coefficient 1

*Note: This parameter is for advanced users*

Used to calibrate fuel flow for MS protocol (Slope). This should be calculated from a log at constant fuel usage rate. Plot (ECYL[0].InjT*EFI.Rpm)/600.0 to get the duty_cycle. Measure actual fuel usage in cm^3/min, and set EFI_COEF1 = fuel_usage_cm3permin / duty_cycle

- Range: 0 1

## EFI_COEF2: EFI Calibration Coefficient 2

*Note: This parameter is for advanced users*

Used to calibrate fuel flow for MS protocol (Offset). This can be used to correct for a non-zero offset in the fuel consumption calculation of EFI_COEF1

- Range: 0 10

## EFI_FUEL_DENS: ECU Fuel Density

*Note: This parameter is for advanced users*

Used to calculate fuel consumption

- Units: kg/m/m/m

- Range: 0 10000

# ESCTLM Parameters

## ESC_TLM_MAV_OFS: ESC Telemetry mavlink offset

Offset to apply to ESC numbers when reporting as ESC_TELEMETRY packets over MAVLink. This allows high numbered motors to be displayed as low numbered ESCs for convenience on GCS displays. A value of 4 would send ESC on output 5 as ESC number 1 in ESC_TELEMETRY packets

- Increment: 1

- Range: 0 31

# FENCE Parameters

## FENCE_ENABLE: Fence enable/disable

Allows you to enable (1) or disable (0) the fence functionality

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## FENCE_ACTION: Fence Action

What action should be taken when fence is breached

|Value|Meaning|
|:---:|:---:|
|0|Report Only|
|1|RTL or Land|

## FENCE_RADIUS: Circular Fence Radius

Circle fence radius which when breached will cause an RTL

- Units: m

- Range: 30 10000

## FENCE_MARGIN: Fence Margin

Distance that autopilot's should maintain from the fence to avoid a breach

- Units: m

- Range: 1 10

## FENCE_TOTAL: Fence polygon point total

Number of polygon points saved in eeprom (do not update manually)

- Range: 1 20

# FFT Parameters

## FFT_ENABLE: Enable

*Note: This parameter is for advanced users*

Enable Gyro FFT analyser

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## FFT_MINHZ: Minimum Frequency

*Note: This parameter is for advanced users*

Lower bound of FFT frequency detection in Hz. On larger vehicles the minimum motor frequency is likely to be significantly lower than for smaller vehicles.

- Range: 20 400

- Units: Hz

## FFT_MAXHZ: Maximum Frequency

*Note: This parameter is for advanced users*

Upper bound of FFT frequency detection in Hz. On smaller vehicles the maximum motor frequency is likely to be significantly higher than for larger vehicles.

- Range: 20 495

- Units: Hz

## FFT_SAMPLE_MODE: Sample Mode

*Note: This parameter is for advanced users*

Sampling mode (and therefore rate). 0: Gyro rate sampling, 1: Fast loop rate sampling, 2: Fast loop rate / 2 sampling, 3: Fast loop rate / 3 sampling. Takes effect on reboot.

- Range: 0 4

- RebootRequired: True

## FFT_WINDOW_SIZE: FFT window size

*Note: This parameter is for advanced users*

Size of window to be used in FFT calculations. Takes effect on reboot. Must be a power of 2 and between 32 and 512. Larger windows give greater frequency resolution but poorer time resolution, consume more CPU time and may not be appropriate for all vehicles. Time and frequency resolution are given by the sample-rate / window-size. Windows of 256 are only really recommended for F7 class boards, windows of 512 or more H7 class.

- Range: 32 1024

- RebootRequired: True

## FFT_WINDOW_OLAP: FFT window overlap

*Note: This parameter is for advanced users*

Percentage of window to be overlapped before another frame is process. Takes effect on reboot. A good default is 50% overlap. Higher overlap results in more processed frames but not necessarily more temporal resolution. Lower overlap results in lost information at the frame edges.

- Range: 0 0.9

- RebootRequired: True

## FFT_FREQ_HOVER: FFT learned hover frequency

*Note: This parameter is for advanced users*

The learned hover noise frequency

- Range: 0 250

## FFT_THR_REF: FFT learned thrust reference

*Note: This parameter is for advanced users*

FFT learned thrust reference for the hover frequency and FFT minimum frequency.

- Range: 0.01 0.9

## FFT_SNR_REF: FFT SNR reference threshold

*Note: This parameter is for advanced users*

FFT SNR reference threshold in dB at which a signal is determined to be present.

- Range: 0.0 100.0

## FFT_ATT_REF: FFT attenuation for bandwidth calculation

*Note: This parameter is for advanced users*

FFT attenuation level in dB for bandwidth calculation and peak detection. The bandwidth is calculated by comparing peak power output with the attenuated version. The default of 15 has shown to be a good compromise in both simulations and real flight.

- Range: 0 100

## FFT_BW_HOVER: FFT learned bandwidth at hover

*Note: This parameter is for advanced users*

FFT learned bandwidth at hover for the attenuation frequencies.

- Range: 0 200

## FFT_HMNC_FIT: FFT harmonic fit frequency threshold

*Note: This parameter is for advanced users*

FFT harmonic fit frequency threshold percentage at which a signal of the appropriate frequency is determined to be the harmonic of another. Signals that have a harmonic relationship that varies at most by this percentage are considered harmonics of each other for the purpose of selecting the harmonic notch frequency. If a match is found then the lower frequency harmonic is always used as the basis for the dynamic harmonic notch. A value of zero completely disables harmonic matching.

- Range: 0 100

- RebootRequired: True

## FFT_HMNC_PEAK: FFT harmonic peak target

*Note: This parameter is for advanced users*

The FFT harmonic peak target that should be returned by FTN1.PkAvg. The resulting value will be used by the harmonic notch if configured to track the FFT frequency. By default the appropriate peak is auto-detected based on the harmonic fit between peaks and the energy-weighted average frequency on roll on pitch is used. Setting this to 1 will always target the highest energy peak. Setting this to 2 will target the highest energy peak that is lower in frequency than the highest energy peak. Setting this to 3 will target the highest energy peak that is higher in frequency than the highest energy peak. Setting this to 4 will target the highest energy peak on the roll axis only and only the roll frequency will be used (some vehicles have a much more pronounced peak on roll). Setting this to 5 will target the highest energy peak on the pitch axis only and only the pitch frequency will be used (some vehicles have a much more pronounced peak on roll).

|Value|Meaning|
|:---:|:---:|
|0|Auto|
|1|Center Frequency|
|2|Lower-Shoulder Frequency|
|3|Upper-Shoulder Frequency|
|4|Roll-Axis|
|5|Pitch-Axis|

## FFT_NUM_FRAMES: FFT output frames to retain and average

*Note: This parameter is for advanced users*

Number of output frequency frames to retain and average in order to calculate final frequencies. Averaging output frames can drastically reduce noise and jitter at the cost of latency as long as the input is stable. The default is to perform no averaging. For rapidly changing frequencies (e.g. smaller aircraft) fewer frames should be averaged.

- Range: 0 8

- RebootRequired: True

## FFT_OPTIONS: FFT options

*Note: This parameter is for advanced users*

FFT configuration options. Values: 1:Apply the FFT *after* the filter bank,2:Check noise at the motor frequencies using ESC data as a reference

- Bitmask: 0:Enable post-filter FFT,1:Check motor noise

- RebootRequired: True

# FRSKY Parameters

## FRSKY_UPLINK_ID: Uplink sensor id

*Note: This parameter is for advanced users*

Change the uplink sensor id (SPort only)

|Value|Meaning|
|:---:|:---:|
|-1|Disable|
|7|7|
|8|8|
|9|9|
|10|10|
|11|11|
|12|12|
|13|13|
|14|14|
|15|15|
|16|16|
|17|17|
|18|18|
|19|19|
|20|20|
|21|21|
|22|22|
|23|23|
|24|24|
|25|25|
|26|26|

## FRSKY_DNLINK1_ID: First downlink sensor id

*Note: This parameter is for advanced users*

Change the first extra downlink sensor id (SPort only)

|Value|Meaning|
|:---:|:---:|
|-1|Disable|
|7|7|
|8|8|
|9|9|
|10|10|
|11|11|
|12|12|
|13|13|
|14|14|
|15|15|
|16|16|
|17|17|
|18|18|
|19|19|
|20|20|
|21|21|
|22|22|
|23|23|
|24|24|
|25|25|
|26|26|

## FRSKY_DNLINK2_ID: Second downlink sensor id

*Note: This parameter is for advanced users*

Change the second extra downlink sensor id (SPort only)

|Value|Meaning|
|:---:|:---:|
|-1|Disable|
|7|7|
|8|8|
|9|9|
|10|10|
|11|11|
|12|12|
|13|13|
|14|14|
|15|15|
|16|16|
|17|17|
|18|18|
|19|19|
|20|20|
|21|21|
|22|22|
|23|23|
|24|24|
|25|25|
|26|26|

## FRSKY_DNLINK_ID: Default downlink sensor id

*Note: This parameter is for advanced users*

Change the default downlink sensor id (SPort only)

|Value|Meaning|
|:---:|:---:|
|-1|Disable|
|7|7|
|8|8|
|9|9|
|10|10|
|11|11|
|12|12|
|13|13|
|14|14|
|15|15|
|16|16|
|17|17|
|18|18|
|19|19|
|20|20|
|21|21|
|22|22|
|23|23|
|24|24|
|25|25|
|26|26|
|27|27|

## FRSKY_OPTIONS: FRSky Telemetry Options

A bitmask to set some FRSky Telemetry specific options

- Bitmask: 0:EnableAirspeedAndGroundspeed

# GEN Parameters

## GEN_TYPE: Generator type

Generator type

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|IE 650w 800w Fuel Cell|
|2|IE 2.4kW Fuel Cell|
|3|Richenpower|

- RebootRequired: True

## GEN_OPTIONS: Generator Options

Bitmask of options for generators

- Bitmask: 0:Supress Maintenance-Required Warnings

# GPS Parameters

## GPS_TYPE: 1st GPS type

*Note: This parameter is for advanced users*

GPS type of 1st GPS

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|AUTO|
|2|uBlox|
|5|NMEA|
|6|SiRF|
|7|HIL|
|8|SwiftNav|
|9|DroneCAN|
|10|SBF|
|11|GSOF|
|13|ERB|
|14|MAV|
|15|NOVA|
|16|HemisphereNMEA|
|17|uBlox-MovingBaseline-Base|
|18|uBlox-MovingBaseline-Rover|
|19|MSP|
|20|AllyStar|
|21|ExternalAHRS|
|22|DroneCAN-MovingBaseline-Base|
|23|DroneCAN-MovingBaseline-Rover|
|24|UnicoreNMEA|
|25|UnicoreMovingBaselineNMEA|

- RebootRequired: True

## GPS_TYPE2: 2nd GPS type

*Note: This parameter is for advanced users*

GPS type of 2nd GPS

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|AUTO|
|2|uBlox|
|5|NMEA|
|6|SiRF|
|7|HIL|
|8|SwiftNav|
|9|DroneCAN|
|10|SBF|
|11|GSOF|
|13|ERB|
|14|MAV|
|15|NOVA|
|16|HemisphereNMEA|
|17|uBlox-MovingBaseline-Base|
|18|uBlox-MovingBaseline-Rover|
|19|MSP|
|20|AllyStar|
|21|ExternalAHRS|
|22|DroneCAN-MovingBaseline-Base|
|23|DroneCAN-MovingBaseline-Rover|
|24|UnicoreNMEA|
|25|UnicoreMovingBaselineNMEA|

- RebootRequired: True

## GPS_NAVFILTER: Navigation filter setting

*Note: This parameter is for advanced users*

Navigation filter engine setting

|Value|Meaning|
|:---:|:---:|
|0|Portable|
|2|Stationary|
|3|Pedestrian|
|4|Automotive|
|5|Sea|
|6|Airborne1G|
|7|Airborne2G|
|8|Airborne4G|

## GPS_AUTO_SWITCH: Automatic Switchover Setting

*Note: This parameter is for advanced users*

Automatic switchover to GPS reporting best lock, 1:UseBest selects the GPS with highest status, if both are equal the GPS with highest satellite count is used 4:Use primary if 3D fix or better, will revert to 'UseBest' behaviour if 3D fix is lost on primary

|Value|Meaning|
|:---:|:---:|
|0|Use primary|
|1|UseBest|
|2|Blend|
|4|Use primary if 3D fix or better|

## GPS_MIN_DGPS: Minimum Lock Type Accepted for DGPS

*Note: This parameter is for advanced users*

Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode.

|Value|Meaning|
|:---:|:---:|
|0|Any|
|50|FloatRTK|
|100|IntegerRTK|

- RebootRequired: True

## GPS_SBAS_MODE: SBAS Mode

*Note: This parameter is for advanced users*

This sets the SBAS (satellite based augmentation system) mode if available on this GPS. If set to 2 then the SBAS mode is not changed in the GPS. Otherwise the GPS will be reconfigured to enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|NoChange|

## GPS_MIN_ELEV: Minimum elevation

*Note: This parameter is for advanced users*

This sets the minimum elevation of satellites above the horizon for them to be used for navigation. Setting this to -100 leaves the minimum elevation set to the GPS modules default.

- Range: -100 90

- Units: deg

## GPS_INJECT_TO: Destination for GPS_INJECT_DATA MAVLink packets

*Note: This parameter is for advanced users*

The GGS can send raw serial packets to inject data to multiple GPSes.

|Value|Meaning|
|:---:|:---:|
|0|send to first GPS|
|1|send to 2nd GPS|
|127|send to all|

## GPS_SBP_LOGMASK: Swift Binary Protocol Logging Mask

*Note: This parameter is for advanced users*

Masked with the SBP msg_type field to determine whether SBR1/SBR2 data is logged

|Value|Meaning|
|:---:|:---:|
|0|None (0x0000)|
|-1|All (0xFFFF)|
|-256|External only (0xFF00)|

## GPS_RAW_DATA: Raw data logging

*Note: This parameter is for advanced users*

Handles logging raw data; on uBlox chips that support raw data this will log RXM messages into logger; on Septentrio this will log on the equipment's SD card and when set to 2, the autopilot will try to stop logging after disarming and restart after arming

|Value|Meaning|
|:---:|:---:|
|0|Ignore|
|1|Always log|
|2|Stop logging when disarmed (SBF only)|
|5|Only log every five samples (uBlox only)|

- RebootRequired: True

## GPS_GNSS_MODE: GNSS system configuration

*Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured)

- Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS

## GPS_SAVE_CFG: Save GPS configuration

*Note: This parameter is for advanced users*

Determines whether the configuration for this GPS should be written to non-volatile memory on the GPS. Currently working for UBlox 6 series and above.

|Value|Meaning|
|:---:|:---:|
|0|Do not save config|
|1|Save config|
|2|Save only when needed|

## GPS_GNSS_MODE2: GNSS system configuration

*Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the second GPS (all unchecked or zero to leave GPS as configured)

- Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS

## GPS_AUTO_CONFIG: Automatic GPS configuration

*Note: This parameter is for advanced users*

Controls if the autopilot should automatically configure the GPS based on the parameters and default settings

|Value|Meaning|
|:---:|:---:|
|0|Disables automatic configuration|
|1|Enable automatic configuration for Serial GPSes only|
|2|Enable automatic configuration for DroneCAN as well|

## GPS_RATE_MS: GPS update rate in milliseconds

*Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.

- Units: ms

|Value|Meaning|
|:---:|:---:|
|100|10Hz|
|125|8Hz|
|200|5Hz|

- Range: 50 200

## GPS_RATE_MS2: GPS 2 update rate in milliseconds

*Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.

- Units: ms

|Value|Meaning|
|:---:|:---:|
|100|10Hz|
|125|8Hz|
|200|5Hz|

- Range: 50 200

## GPS_POS1_X: Antenna X position offset

*Note: This parameter is for advanced users*

X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS1_Y: Antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS1_Z: Antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS2_X: Antenna X position offset

*Note: This parameter is for advanced users*

X position of the second GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS2_Y: Antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the second GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS2_Z: Antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the second GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_DELAY_MS: GPS delay in milliseconds

*Note: This parameter is for advanced users*

Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.

- Units: ms

- Range: 0 250

- RebootRequired: True

## GPS_DELAY_MS2: GPS 2 delay in milliseconds

*Note: This parameter is for advanced users*

Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.

- Units: ms

- Range: 0 250

- RebootRequired: True

## GPS_BLEND_MASK: Multi GPS Blending Mask

*Note: This parameter is for advanced users*

Determines which of the accuracy measures Horizontal position, Vertical Position and Speed are used to calculate the weighting on each GPS receiver when soft switching has been selected by setting GPS_AUTO_SWITCH to 2(Blend)

- Bitmask: 0:Horiz Pos,1:Vert Pos,2:Speed

## GPS_BLEND_TC: Blending time constant

*Note: This parameter is for advanced users*

Controls the slowest time constant applied to the calculation of GPS position and height offsets used to adjust different GPS receivers for steady state position differences.

- Units: s

- Range: 5.0 30.0

## GPS_DRV_OPTIONS: driver options

*Note: This parameter is for advanced users*

Additional backend specific options

- Bitmask: 0:Use UART2 for moving baseline on ublox,1:Use base station for GPS yaw on SBF,2:Use baudrate 115200,3:Use dedicated CAN port b/w GPSes for moving baseline,4:Use ellipsoid height instead of AMSL for uBlox driver

## GPS_COM_PORT: GPS physical COM port

*Note: This parameter is for advanced users*

The physical COM port on the connected device, currently only applies to SBF GPS

- Range: 0 10

- Increment: 1

- RebootRequired: True

## GPS_COM_PORT2: GPS physical COM port

*Note: This parameter is for advanced users*

The physical COM port on the connected device, currently only applies to SBF GPS

- Range: 0 10

- Increment: 1

- RebootRequired: True

## GPS_PRIMARY: Primary GPS

*Note: This parameter is for advanced users*

This GPS will be used when GPS_AUTO_SWITCH is 0 and used preferentially with GPS_AUTO_SWITCH = 4.

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|FirstGPS|
|1|SecondGPS|

## GPS_CAN_NODEID1: GPS Node ID 1

*Note: This parameter is for advanced users*

GPS Node id for first-discovered GPS.

- ReadOnly: True

## GPS_CAN_NODEID2: GPS Node ID 2

*Note: This parameter is for advanced users*

GPS Node id for second-discovered GPS.

- ReadOnly: True

## GPS1_CAN_OVRIDE: First DroneCAN GPS NODE ID

*Note: This parameter is for advanced users*

GPS Node id for first GPS. If 0 the gps will be automatically selected on a first-come-first-GPS basis.

## GPS2_CAN_OVRIDE: Second DroneCAN GPS NODE ID

*Note: This parameter is for advanced users*

GPS Node id for second GPS. If 0 the gps will be automatically selected on a second-come-second-GPS basis.

# GPSMB1 Parameters

## GPS_MB1_TYPE: Moving base type

*Note: This parameter is for advanced users*

Controls the type of moving base used if using moving base.

|Value|Meaning|
|:---:|:---:|
|0|Relative to alternate GPS instance|
|1|RelativeToCustomBase|

- RebootRequired: True

## GPS_MB1_OFS_X: Base antenna X position offset

*Note: This parameter is for advanced users*

X position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive X is forward of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB1_OFS_Y: Base antenna Y position offset    

*Note: This parameter is for advanced users*

Y position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Y is to the right of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB1_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Z is down from the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

# GPSMB2 Parameters

## GPS_MB2_TYPE: Moving base type

*Note: This parameter is for advanced users*

Controls the type of moving base used if using moving base.

|Value|Meaning|
|:---:|:---:|
|0|Relative to alternate GPS instance|
|1|RelativeToCustomBase|

- RebootRequired: True

## GPS_MB2_OFS_X: Base antenna X position offset

*Note: This parameter is for advanced users*

X position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive X is forward of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB2_OFS_Y: Base antenna Y position offset    

*Note: This parameter is for advanced users*

Y position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Y is to the right of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB2_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Z is down from the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

# KDE Parameters

## KDE_NPOLE: Number of motor poles

Sets the number of motor poles to calculate the correct RPM value

# LOG Parameters

## LOG_BACKEND_TYPE: AP_Logger Backend Storage type

Bitmap of what Logger backend types to enable. Block-based logging is available on SITL and boards with dataflash chips. Multiple backends can be selected.

- Bitmask: 0:File,1:MAVLink,2:Block

## LOG_FILE_BUFSIZE: Maximum AP_Logger File and Block Backend buffer size (in kilobytes)

The File and Block backends use a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.

## LOG_DISARMED: Enable logging while disarmed

If LOG_DISARMED is set to 1 then logging will be enabled at all times including when disarmed. Logging before arming can make for very large logfiles but can help a lot when tracking down startup issues and is necessary if logging of EKF replay data is selected via the LOG_REPLAY parameter. If LOG_DISARMED is set to 2, then logging will be enabled when disarmed, but not if a USB connection is detected. This can be used to prevent unwanted data logs being generated when the vehicle is connected via USB for log downloading or parameter changes. If LOG_DISARMED is set to 3 then logging will happen while disarmed, but if the vehicle never arms then the logs using the filesystem backend will be discarded on the next boot.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Disabled on USB connection|
|3|Discard log on reboot if never armed|

## LOG_REPLAY: Enable logging of information needed for Replay

If LOG_REPLAY is set to 1 then the EKF2 and EKF3 state estimators will log detailed information needed for diagnosing problems with the Kalman filter. LOG_DISARMED must be set to 1 or 2 or else the log will not contain the pre-flight data required for replay testing of the EKF's. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## LOG_FILE_DSRMROT: Stop logging to current file on disarm

When set, the current log file is closed when the vehicle is disarmed.  If LOG_DISARMED is set then a fresh log will be opened. Applies to the File and Block logging backends.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## LOG_MAV_BUFSIZE: Maximum AP_Logger MAVLink Backend buffer size

*Note: This parameter is for advanced users*

Maximum amount of memory to allocate to AP_Logger-over-mavlink

- Units: kB

## LOG_FILE_TIMEOUT: Timeout before giving up on file writes

This controls the amount of time before failing writes to a log file cause the file to be closed and logging stopped.

- Units: s

## LOG_FILE_MB_FREE: Old logs on the SD card will be deleted to maintain this amount of free space

Set this such that the free space is larger than your largest typical flight log

- Units: MB

- Range: 10 1000

## LOG_FILE_RATEMAX: Maximum logging rate for file backend

This sets the maximum rate that streaming log messages will be logged to the file backend. A value of zero means that rate limiting is disabled.

- Units: Hz

- Range: 0 1000

- Increment: 0.1

## LOG_MAV_RATEMAX: Maximum logging rate for mavlink backend

This sets the maximum rate that streaming log messages will be logged to the mavlink backend. A value of zero means that rate limiting is disabled.

- Units: Hz

- Range: 0 1000

- Increment: 0.1

## LOG_BLK_RATEMAX: Maximum logging rate for block backend

This sets the maximum rate that streaming log messages will be logged to the block backend. A value of zero means that rate limiting is disabled.

- Units: Hz

- Range: 0 1000

- Increment: 0.1

## LOG_DARM_RATEMAX: Maximum logging rate when disarmed

This sets the maximum rate that streaming log messages will be logged to any backend when disarmed. A value of zero means that the normal backend rate limit is applied.

- Units: Hz

- Range: 0 1000

- Increment: 0.1

# MSP Parameters

## MSP_OSD_NCELLS: Cell count override

Used for average cell voltage calculation

|Value|Meaning|
|:---:|:---:|
|0|Auto|
|1|1|
|2|2|
|3|3|
|4|4|
|5|5|
|6|6|
|7|7|
|8|8|
|9|9|
|10|10|
|11|11|
|12|12|
|13|13|
|14|14|

## MSP_OPTIONS: MSP OSD Options

A bitmask to set some MSP specific options

- Bitmask: 0:EnableTelemetryMode, 1: DisableDJIWorkarounds, 2:EnableBTFLFonts

# NMEA Parameters

## NMEA_RATE_MS: NMEA Output rate

NMEA Output rate. This controls the interval at which all the enabled NMEA messages are sent. Most NMEA systems expect 100ms (10Hz) or slower.

- Range: 20 2000

- Increment: 1

- Units: ms

## NMEA_MSG_EN: Messages Enable bitmask

This is a bitmask of enabled NMEA messages. All messages will be sent consecutively at the same rate interval

- Bitmask: 0:GPGGA,1:GPRMC,2:PASHR

# NTF Parameters

## NTF_LED_BRIGHT: LED Brightness

*Note: This parameter is for advanced users*

Select the RGB LED brightness level. When USB is connected brightness will never be higher than low regardless of the setting.

|Value|Meaning|
|:---:|:---:|
|0|Off|
|1|Low|
|2|Medium|
|3|High|

## NTF_BUZZ_TYPES: Buzzer Driver Types

*Note: This parameter is for advanced users*

Controls what types of Buzzer will be enabled

- Bitmask: 0:Built-in buzzer, 1:DShot, 2:DroneCAN

## NTF_LED_OVERRIDE: Specifies colour source for the RGBLed

*Note: This parameter is for advanced users*

Specifies the source for the colours and brightness for the LED.  OutbackChallenge conforms to the MedicalExpress (https://uavchallenge.org/medical-express/) rules, essentially "Green" is disarmed (safe-to-approach), "Red" is armed (not safe-to-approach). Traffic light is a simplified color set, red when armed, yellow when the safety switch is not surpressing outputs (but disarmed), and green when outputs are surpressed and disarmed, the LED will blink faster if disarmed and failing arming checks.

|Value|Meaning|
|:---:|:---:|
|0|Standard|
|1|MAVLink/Scripting/AP_Periph|
|2|OutbackChallenge|
|3|TrafficLight|

## NTF_DISPLAY_TYPE: Type of on-board I2C display

*Note: This parameter is for advanced users*

This sets up the type of on-board I2C display. Disabled by default.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|ssd1306|
|2|sh1106|
|10|SITL|

## NTF_OREO_THEME: OreoLED Theme

*Note: This parameter is for advanced users*

Enable/Disable Solo Oreo LED driver, 0 to disable, 1 for Aircraft theme, 2 for Rover theme

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Aircraft|
|2|Rover|

## NTF_BUZZ_PIN: Buzzer pin

*Note: This parameter is for advanced users*

Enables to connect active buzzer to arbitrary pin. Requires 3-pin buzzer or additional MOSFET! Some the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

## NTF_LED_TYPES: LED Driver Types

*Note: This parameter is for advanced users*

Controls what types of LEDs will be enabled

- Bitmask: 0:Built-in LED, 1:Internal ToshibaLED, 2:External ToshibaLED, 3:External PCA9685, 4:Oreo LED, 5:DroneCAN, 6:NCP5623 External, 7:NCP5623 Internal, 8:NeoPixel, 9:ProfiLED, 10:Scripting, 11:DShot, 12:ProfiLED_SPI, 13:LP5562 External, 14: LP5562 Internal

## NTF_BUZZ_ON_LVL: Buzzer-on pin logic level

*Note: This parameter is for advanced users*

Specifies pin level that indicates buzzer should play

|Value|Meaning|
|:---:|:---:|
|0|LowIsOn|
|1|HighIsOn|

## NTF_BUZZ_VOLUME: Buzzer volume

Control the volume of the buzzer

- Range: 0 100

- Units: %

## NTF_LED_LEN: Serial LED String Length

*Note: This parameter is for advanced users*

The number of Serial LED's to use for notifications (NeoPixel's and ProfiLED)

- Range: 1 32

- RebootRequired: True

# Node Parameters

## Node_BOOTCNT: Boot Count

Number of times board has been booted

- ReadOnly: True

## Node_FLTTIME: Total FlightTime

Total FlightTime (seconds)

- Units: s

- ReadOnly: True

## Node_RUNTIME: Total RunTime

Total time autopilot has run

- Units: s

- ReadOnly: True

## Node_RESET: Statistics Reset Time

Seconds since January 1st 2016 (Unix epoch+1451606400) since statistics reset (set to 0 to reset statistics)

- Units: s

- ReadOnly: True

# OUT Parameters

## OUT_RATE: Servo default output rate

*Note: This parameter is for advanced users*

Default output rate in Hz for all PWM outputs.

- Range: 25 400

- Units: Hz

## OUT_DSHOT_RATE: Servo DShot output rate

*Note: This parameter is for advanced users*

DShot output rate for all outputs as a multiple of the loop rate. 0 sets the output rate to be fixed at 1Khz for low loop rates. This value should never be set below 500Hz.

|Value|Meaning|
|:---:|:---:|
|0|1Khz|
|1|loop-rate|
|2|double loop-rate|
|3|triple loop-rate|
|4|quadruple loop rate|

## OUT_DSHOT_ESC: Servo DShot ESC type

*Note: This parameter is for advanced users*

DShot ESC type for all outputs. The ESC type affects the range of DShot commands available and the bit widths used. None means that no dshot commands will be executed. Some ESC types support Extended DShot Telemetry (EDT) which allows telemetry other than RPM data to be returned when using bi-directional dshot. If you enable EDT you must install EDT capable firmware for correct operation.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|BLHeli32/Kiss|
|2|BLHeli_S|
|3|BLHeli32/Kiss+EDT|
|4|BLHeli_S+EDT|

## OUT_GPIO_MASK: Servo GPIO mask

*Note: This parameter is for advanced users*

Bitmask of outputs which will be available as GPIOs. Any output with either the function set to -1 or with the corresponding bit set in this mask will be available for use as a GPIO pin

- Bitmask: 0:Servo 1, 1:Servo 2, 2:Servo 3, 3:Servo 4, 4:Servo 5, 5:Servo 6, 6:Servo 7, 7:Servo 8, 8:Servo 9, 9:Servo 10, 10:Servo 11, 11:Servo 12, 12:Servo 13, 13:Servo 14, 14:Servo 15, 15:Servo 16, 16:Servo 17, 17:Servo 18, 18:Servo 19, 19:Servo 20, 20:Servo 21, 21:Servo 22, 22:Servo 23, 23:Servo 24, 24:Servo 25, 25:Servo 26, 26:Servo 27, 27:Servo 28, 28:Servo 29, 29:Servo 30, 30:Servo 31, 31:Servo 32

- RebootRequired: True

## OUT_RC_FS_MSK: Servo RC Failsafe Mask

*Note: This parameter is for advanced users*

Bitmask of scaled passthru output function which will be set to their trim value during rc failsafe instead of holding their last position before failsafe.

- Bitmask: 0:RCIN1Scaled, 1:RCIN2Scaled, 2:RCIN3Scaled, 3:RCIN4Scaled, 4:RCIN5Scaled, 5:RCIN6Scaled, 6:RCIN7Scaled, 7:RCIN8Scaled, 8:RCIN9Scaled, 9:RCIN10Scaled, 10:RCIN11Scaled, 11:SRCIN12Scaled, 12:RCIN13Scaled, 13:RCIN14Scaled, 14:RCIN15Scaled, 15:RCIN16Scaled

## OUT_32_ENABLE: Enable outputs 17 to 31

*Note: This parameter is for advanced users*

This allows for up to 32 outputs, enabling parameters for outputs above 16

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

# OUT10 Parameters

## OUT10_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT10_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT10_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT10_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT10_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT11 Parameters

## OUT11_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT11_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT11_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT11_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT11_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT12 Parameters

## OUT12_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT12_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT12_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT12_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT12_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT13 Parameters

## OUT13_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT13_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT13_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT13_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT13_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT14 Parameters

## OUT14_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT14_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT14_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT14_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT14_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT15 Parameters

## OUT15_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT15_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT15_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT15_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT15_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT16 Parameters

## OUT16_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT16_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT16_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT16_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT16_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT17 Parameters

## OUT17_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT17_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT17_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT17_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT17_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT18 Parameters

## OUT18_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT18_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT18_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT18_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT18_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT19 Parameters

## OUT19_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT19_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT19_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT19_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT19_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT1 Parameters

## OUT1_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT1_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT1_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT1_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT1_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT20 Parameters

## OUT20_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT20_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT20_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT20_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT20_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT21 Parameters

## OUT21_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT21_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT21_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT21_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT21_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT22 Parameters

## OUT22_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT22_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT22_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT22_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT22_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT23 Parameters

## OUT23_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT23_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT23_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT23_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT23_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT24 Parameters

## OUT24_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT24_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT24_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT24_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT24_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT25 Parameters

## OUT25_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT25_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT25_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT25_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT25_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT26 Parameters

## OUT26_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT26_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT26_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT26_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT26_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT27 Parameters

## OUT27_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT27_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT27_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT27_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT27_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT28 Parameters

## OUT28_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT28_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT28_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT28_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT28_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT29 Parameters

## OUT29_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT29_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT29_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT29_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT29_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT2 Parameters

## OUT2_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT2_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT2_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT2_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT2_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT30 Parameters

## OUT30_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT30_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT30_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT30_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT30_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT31 Parameters

## OUT31_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT31_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT31_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT31_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT31_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT32 Parameters

## OUT32_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT32_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT32_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT32_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT32_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT3 Parameters

## OUT3_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT3_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT3_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT3_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT3_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT4 Parameters

## OUT4_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT4_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT4_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT4_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT4_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT5 Parameters

## OUT5_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT5_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT5_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT5_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT5_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT6 Parameters

## OUT6_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT6_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT6_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT6_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT6_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT7 Parameters

## OUT7_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT7_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT7_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT7_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT7_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT8 Parameters

## OUT8_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT8_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT8_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT8_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT8_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUT9 Parameters

## OUT9_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT9_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT9_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## OUT9_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## OUT9_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|16|DifferentialSpoilerLeft1|
|17|DifferentialSpoilerRight1|
|19|Elevator|
|21|Rudder|
|22|SprayerPump|
|23|SprayerSpinner|
|24|FlaperonLeft|
|25|FlaperonRight|
|26|GroundSteering|
|27|Parachute|
|28|Gripper|
|29|LandingGear|
|30|EngineRunEnable|
|31|HeliRSC|
|32|HeliTailRSC|
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7|
|40|Motor8|
|41|TiltMotorsFront|
|45|TiltMotorsRear|
|46|TiltMotorRearLeft|
|47|TiltMotorRearRight|
|51|RCIN1|
|52|RCIN2|
|53|RCIN3|
|54|RCIN4|
|55|RCIN5|
|56|RCIN6|
|57|RCIN7|
|58|RCIN8|
|59|RCIN9|
|60|RCIN10|
|61|RCIN11|
|62|RCIN12|
|63|RCIN13|
|64|RCIN14|
|65|RCIN15|
|66|RCIN16|
|67|Ignition|
|69|Starter|
|70|Throttle|
|71|TrackerYaw|
|72|TrackerPitch|
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
|88|Winch|
|89|Main Sail|
|90|CameraISO|
|91|CameraAperture|
|92|CameraFocus|
|93|CameraShutterSpeed|
|94|Script1|
|95|Script2|
|96|Script3|
|97|Script4|
|98|Script5|
|99|Script6|
|100|Script7|
|101|Script8|
|102|Script9|
|103|Script10|
|104|Script11|
|105|Script12|
|106|Script13|
|107|Script14|
|108|Script15|
|109|Script16|
|120|NeoPixel1|
|121|NeoPixel2|
|122|NeoPixel3|
|123|NeoPixel4|
|124|RateRoll|
|125|RatePitch|
|126|RateThrust|
|127|RateYaw|
|128|WingSailElevator|
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
|137|SailMastRotation|
|138|Alarm|
|139|Alarm Inverted|
|140|RCIN1Scaled|
|141|RCIN2Scaled|
|142|RCIN3Scaled|
|143|RCIN4Scaled|
|144|RCIN5Scaled|
|145|RCIN6Scaled|
|146|RCIN7Scaled|
|147|RCIN8Scaled|
|148|RCIN9Scaled|
|149|RCIN10Scaled|
|150|RCIN11Scaled|
|151|RCIN12Scaled|
|152|RCIN13Scaled|
|153|RCIN14Scaled|
|154|RCIN15Scaled|
|155|RCIN16Scaled|

- RebootRequired: True

# OUTBLH Parameters

## OUT_BLH_MASK: BLHeli Channel Bitmask

*Note: This parameter is for advanced users*

Enable of BLHeli pass-thru servo protocol support to specific channels. This mask is in addition to motors enabled using SERVO_BLH_AUTO (if any)

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## OUT_BLH_AUTO: BLHeli pass-thru auto-enable for multicopter motors

If set to 1 this auto-enables BLHeli pass-thru support for all multicopter motors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## OUT_BLH_TEST: BLHeli internal interface test

*Note: This parameter is for advanced users*

Setting SERVO_BLH_TEST to a motor number enables an internal test of the BLHeli ESC protocol to the corresponding ESC. The debug output is displayed on the USB console.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TestMotor1|
|2|TestMotor2|
|3|TestMotor3|
|4|TestMotor4|
|5|TestMotor5|
|6|TestMotor6|
|7|TestMotor7|
|8|TestMotor8|

## OUT_BLH_TMOUT: BLHeli protocol timeout

This sets the inactivity timeout for the BLHeli protocol in seconds. If no packets are received in this time normal MAVLink operations are resumed. A value of 0 means no timeout

- Units: s

- Range: 0 300

## OUT_BLH_TRATE: BLHeli telemetry rate

This sets the rate in Hz for requesting telemetry from ESCs. It is the rate per ESC. Setting to zero disables telemetry requests

- Units: Hz

- Range: 0 500

## OUT_BLH_DEBUG: BLHeli debug level

When set to 1 this enabled verbose debugging output over MAVLink when the blheli protocol is active. This can be used to diagnose failures.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OUT_BLH_OTYPE: BLHeli output type override

*Note: This parameter is for advanced users*

When set to a non-zero value this overrides the output type for the output channels given by SERVO_BLH_MASK. This can be used to enable DShot on outputs that are not part of the multicopter motors group.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|OneShot|
|2|OneShot125|
|3|Brushed|
|4|DShot150|
|5|DShot300|
|6|DShot600|
|7|DShot1200|

- RebootRequired: True

## OUT_BLH_PORT: Control port

*Note: This parameter is for advanced users*

This sets the mavlink channel to use for blheli pass-thru. The channel number is determined by the number of serial ports configured to use mavlink. So 0 is always the console, 1 is the next serial port using mavlink, 2 the next after that and so on.

|Value|Meaning|
|:---:|:---:|
|0|Console|
|1|Mavlink Serial Channel1|
|2|Mavlink Serial Channel2|
|3|Mavlink Serial Channel3|
|4|Mavlink Serial Channel4|
|5|Mavlink Serial Channel5|

## OUT_BLH_POLES: BLHeli Motor Poles

*Note: This parameter is for advanced users*

This allows calculation of true RPM from ESC's eRPM. The default is 14.

- Range: 1 127

- RebootRequired: True

## OUT_BLH_3DMASK: BLHeli bitmask of 3D channels

*Note: This parameter is for advanced users*

Mask of channels which are dynamically reversible. This is used to configure ESCs in '3D' mode, allowing for the motor to spin in either direction

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## OUT_BLH_BDMASK: BLHeli bitmask of bi-directional dshot channels

*Note: This parameter is for advanced users*

Mask of channels which support bi-directional dshot. This is used for ESCs which have firmware that supports bi-directional dshot allowing fast rpm telemetry values to be returned for the harmonic notch.

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## OUT_BLH_RVMASK: BLHeli bitmask of reversed channels

*Note: This parameter is for advanced users*

Mask of channels which are reversed. This is used to configure ESCs in reversed mode

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

# OUTFTW Parameters

## OUT_FTW_MASK: Servo channel output bitmask

Servo channel mask specifying FETtec ESC output.

- Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12

- RebootRequired: True

## OUT_FTW_RVMASK: Servo channel reverse rotation bitmask

Servo channel mask to reverse rotation of FETtec ESC outputs.

- Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12

## OUT_FTW_POLES: Nr. electrical poles

Number of motor electrical poles

- Range: 2 50

# OUTROB Parameters

## OUT_ROB_POSMIN: Robotis servo position min

Position minimum at servo min value. This should be within the position control range of the servos, normally 0 to 4095

- Range: 0 4095

## OUT_ROB_POSMAX: Robotis servo position max

Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095

- Range: 0 4095

# OUTSBUS Parameters

## OUT_SBUS_RATE: SBUS default output rate

*Note: This parameter is for advanced users*

This sets the SBUS output frame rate in Hz.

- Range: 25 250

- Units: Hz

# OUTVOLZ Parameters

## OUT_VOLZ_MASK: Channel Bitmask

Enable of volz servo protocol to specific channels

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16,16:Channel17,17:Channel18,18:Channel19,19:Channel20,20:Channel21,21:Channel22,22:Channel23,23:Channel24,24:Channel25,25:Channel26,26:Channel27,28:Channel29,29:Channel30,30:Channel31,31:Channel32

# PRX Parameters

## PRX_LOG_RAW: Proximity raw distances log

*Note: This parameter is for advanced users*

Set this parameter to one if logging unfiltered(raw) distances from sensor should be enabled

|Value|Meaning|
|:---:|:---:|
|0|Off|
|1|On|

## PRX_FILT: Proximity filter cutoff frequency

*Note: This parameter is for advanced users*

Cutoff frequency for low pass filter applied to each face in the proximity boundary

- Units: Hz

- Range: 0 20

# PRX1 Parameters

## PRX1_TYPE: Proximity type

What type of proximity sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|7|LightwareSF40c|
|2|MAVLink|
|3|TeraRangerTower|
|4|RangeFinder|
|5|RPLidarA2|
|6|TeraRangerTowerEvo|
|8|LightwareSF45B|
|10|SITL|
|12|AirSimSITL|
|13|CygbotD1|
|14|DroneCAN|

- RebootRequired: True

## PRX1_ORIENT: Proximity sensor orientation

Proximity sensor orientation

|Value|Meaning|
|:---:|:---:|
|0|Default|
|1|Upside Down|

## PRX1_YAW_CORR: Proximity sensor yaw correction

Proximity sensor yaw correction

- Units: deg

- Range: -180 180

## PRX1_IGN_ANG1: Proximity sensor ignore angle 1

Proximity sensor ignore angle 1

- Units: deg

- Range: 0 360

## PRX1_IGN_WID1: Proximity sensor ignore width 1

Proximity sensor ignore width 1

- Units: deg

- Range: 0 127

## PRX1_IGN_ANG2: Proximity sensor ignore angle 2

Proximity sensor ignore angle 2

- Units: deg

- Range: 0 360

## PRX1_IGN_WID2: Proximity sensor ignore width 2

Proximity sensor ignore width 2

- Units: deg

- Range: 0 127

## PRX1_IGN_ANG3: Proximity sensor ignore angle 3

Proximity sensor ignore angle 3

- Units: deg

- Range: 0 360

## PRX1_IGN_WID3: Proximity sensor ignore width 3

Proximity sensor ignore width 3

- Units: deg

- Range: 0 127

## PRX1_IGN_ANG4: Proximity sensor ignore angle 4

Proximity sensor ignore angle 4

- Units: deg

- Range: 0 360

## PRX1_IGN_WID4: Proximity sensor ignore width 4

Proximity sensor ignore width 4

- Units: deg

- Range: 0 127

## PRX1_MIN: Proximity minimum range

*Note: This parameter is for advanced users*

Minimum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX1_MAX: Proximity maximum range

*Note: This parameter is for advanced users*

Maximum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX1_ADDR: Bus address of sensor

The bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

# PRX2 Parameters

## PRX2_TYPE: Proximity type

What type of proximity sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|7|LightwareSF40c|
|2|MAVLink|
|3|TeraRangerTower|
|4|RangeFinder|
|5|RPLidarA2|
|6|TeraRangerTowerEvo|
|8|LightwareSF45B|
|10|SITL|
|12|AirSimSITL|
|13|CygbotD1|
|14|DroneCAN|

- RebootRequired: True

## PRX2_ORIENT: Proximity sensor orientation

Proximity sensor orientation

|Value|Meaning|
|:---:|:---:|
|0|Default|
|1|Upside Down|

## PRX2_YAW_CORR: Proximity sensor yaw correction

Proximity sensor yaw correction

- Units: deg

- Range: -180 180

## PRX2_IGN_ANG1: Proximity sensor ignore angle 1

Proximity sensor ignore angle 1

- Units: deg

- Range: 0 360

## PRX2_IGN_WID1: Proximity sensor ignore width 1

Proximity sensor ignore width 1

- Units: deg

- Range: 0 127

## PRX2_IGN_ANG2: Proximity sensor ignore angle 2

Proximity sensor ignore angle 2

- Units: deg

- Range: 0 360

## PRX2_IGN_WID2: Proximity sensor ignore width 2

Proximity sensor ignore width 2

- Units: deg

- Range: 0 127

## PRX2_IGN_ANG3: Proximity sensor ignore angle 3

Proximity sensor ignore angle 3

- Units: deg

- Range: 0 360

## PRX2_IGN_WID3: Proximity sensor ignore width 3

Proximity sensor ignore width 3

- Units: deg

- Range: 0 127

## PRX2_IGN_ANG4: Proximity sensor ignore angle 4

Proximity sensor ignore angle 4

- Units: deg

- Range: 0 360

## PRX2_IGN_WID4: Proximity sensor ignore width 4

Proximity sensor ignore width 4

- Units: deg

- Range: 0 127

## PRX2_MIN: Proximity minimum range

*Note: This parameter is for advanced users*

Minimum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX2_MAX: Proximity maximum range

*Note: This parameter is for advanced users*

Maximum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX2_ADDR: Bus address of sensor

The bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

# PRX3 Parameters

## PRX3_TYPE: Proximity type

What type of proximity sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|7|LightwareSF40c|
|2|MAVLink|
|3|TeraRangerTower|
|4|RangeFinder|
|5|RPLidarA2|
|6|TeraRangerTowerEvo|
|8|LightwareSF45B|
|10|SITL|
|12|AirSimSITL|
|13|CygbotD1|
|14|DroneCAN|

- RebootRequired: True

## PRX3_ORIENT: Proximity sensor orientation

Proximity sensor orientation

|Value|Meaning|
|:---:|:---:|
|0|Default|
|1|Upside Down|

## PRX3_YAW_CORR: Proximity sensor yaw correction

Proximity sensor yaw correction

- Units: deg

- Range: -180 180

## PRX3_IGN_ANG1: Proximity sensor ignore angle 1

Proximity sensor ignore angle 1

- Units: deg

- Range: 0 360

## PRX3_IGN_WID1: Proximity sensor ignore width 1

Proximity sensor ignore width 1

- Units: deg

- Range: 0 127

## PRX3_IGN_ANG2: Proximity sensor ignore angle 2

Proximity sensor ignore angle 2

- Units: deg

- Range: 0 360

## PRX3_IGN_WID2: Proximity sensor ignore width 2

Proximity sensor ignore width 2

- Units: deg

- Range: 0 127

## PRX3_IGN_ANG3: Proximity sensor ignore angle 3

Proximity sensor ignore angle 3

- Units: deg

- Range: 0 360

## PRX3_IGN_WID3: Proximity sensor ignore width 3

Proximity sensor ignore width 3

- Units: deg

- Range: 0 127

## PRX3_IGN_ANG4: Proximity sensor ignore angle 4

Proximity sensor ignore angle 4

- Units: deg

- Range: 0 360

## PRX3_IGN_WID4: Proximity sensor ignore width 4

Proximity sensor ignore width 4

- Units: deg

- Range: 0 127

## PRX3_MIN: Proximity minimum range

*Note: This parameter is for advanced users*

Minimum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX3_MAX: Proximity maximum range

*Note: This parameter is for advanced users*

Maximum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX3_ADDR: Bus address of sensor

The bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

# PRX4 Parameters

## PRX4_TYPE: Proximity type

What type of proximity sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|7|LightwareSF40c|
|2|MAVLink|
|3|TeraRangerTower|
|4|RangeFinder|
|5|RPLidarA2|
|6|TeraRangerTowerEvo|
|8|LightwareSF45B|
|10|SITL|
|12|AirSimSITL|
|13|CygbotD1|
|14|DroneCAN|

- RebootRequired: True

## PRX4_ORIENT: Proximity sensor orientation

Proximity sensor orientation

|Value|Meaning|
|:---:|:---:|
|0|Default|
|1|Upside Down|

## PRX4_YAW_CORR: Proximity sensor yaw correction

Proximity sensor yaw correction

- Units: deg

- Range: -180 180

## PRX4_IGN_ANG1: Proximity sensor ignore angle 1

Proximity sensor ignore angle 1

- Units: deg

- Range: 0 360

## PRX4_IGN_WID1: Proximity sensor ignore width 1

Proximity sensor ignore width 1

- Units: deg

- Range: 0 127

## PRX4_IGN_ANG2: Proximity sensor ignore angle 2

Proximity sensor ignore angle 2

- Units: deg

- Range: 0 360

## PRX4_IGN_WID2: Proximity sensor ignore width 2

Proximity sensor ignore width 2

- Units: deg

- Range: 0 127

## PRX4_IGN_ANG3: Proximity sensor ignore angle 3

Proximity sensor ignore angle 3

- Units: deg

- Range: 0 360

## PRX4_IGN_WID3: Proximity sensor ignore width 3

Proximity sensor ignore width 3

- Units: deg

- Range: 0 127

## PRX4_IGN_ANG4: Proximity sensor ignore angle 4

Proximity sensor ignore angle 4

- Units: deg

- Range: 0 360

## PRX4_IGN_WID4: Proximity sensor ignore width 4

Proximity sensor ignore width 4

- Units: deg

- Range: 0 127

## PRX4_MIN: Proximity minimum range

*Note: This parameter is for advanced users*

Minimum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX4_MAX: Proximity maximum range

*Note: This parameter is for advanced users*

Maximum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.

- Units: m

- Range: 0 500

## PRX4_ADDR: Bus address of sensor

The bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

# RNGFND1 Parameters

## RNGFND1_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND1_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND1_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND1_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND1_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND1_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND1_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND1_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND1_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND1_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND1_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND1_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND1_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND1_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND1_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND1_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND1_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND1_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND1_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND1_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND1_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND1_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND1_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND1_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND1_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND2 Parameters

## RNGFND2_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND2_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND2_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND2_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND2_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND2_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND2_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND2_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND2_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND2_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND2_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND2_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND2_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND2_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND2_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND2_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND2_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND2_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND2_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND2_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND2_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND2_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND2_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND2_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND2_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND3 Parameters

## RNGFND3_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND3_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND3_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND3_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND3_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND3_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND3_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND3_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND3_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND3_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND3_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND3_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND3_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND3_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND3_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND3_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND3_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND3_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND3_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND3_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND3_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND3_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND3_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND3_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND3_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND4 Parameters

## RNGFND4_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND4_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND4_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND4_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND4_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND4_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND4_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND4_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND4_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND4_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND4_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND4_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND4_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND4_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND4_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND4_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND4_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND4_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND4_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND4_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND4_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND4_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND4_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND4_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND4_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND5 Parameters

## RNGFND5_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND5_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND5_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND5_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND5_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND5_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND5_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND5_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND5_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND5_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND5_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND5_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND5_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND5_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND5_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND5_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND5_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND5_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND5_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND5_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND5_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND5_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND5_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND5_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND5_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND6 Parameters

## RNGFND6_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND6_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND6_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND6_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND6_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND6_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND6_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND6_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND6_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND6_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND6_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND6_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND6_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND6_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND6_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND6_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND6_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND6_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND6_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND6_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND6_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND6_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND6_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND6_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND6_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND7 Parameters

## RNGFND7_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND7_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND7_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND7_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND7_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND7_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND7_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND7_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND7_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND7_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND7_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND7_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND7_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND7_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND7_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND7_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND7_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND7_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND7_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND7_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND7_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND7_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND7_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND7_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND7_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND8 Parameters

## RNGFND8_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND8_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND8_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND8_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND8_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND8_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND8_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND8_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND8_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND8_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND8_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND8_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND8_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND8_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND8_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND8_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND8_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND8_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND8_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND8_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND8_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND8_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND8_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND8_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND8_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFND9 Parameters

## RNGFND9_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFND9_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFND9_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFND9_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFND9_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFND9_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND9_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFND9_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFND9_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFND9_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFND9_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFND9_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFND9_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND9_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND9_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFND9_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFND9_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFND9_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFND9_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFND9_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFND9_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFND9_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFND9_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFND9_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFND9_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# RNGFNDA Parameters

## RNGFNDA_TYPE: Rangefinder type

Type of connected rangefinder

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Analog|
|2|MaxbotixI2C|
|3|LidarLite-I2C|
|5|PWM|
|6|BBB-PRU|
|7|LightWareI2C|
|8|LightWareSerial|
|9|Bebop|
|10|MAVLink|
|11|USD1_Serial|
|12|LeddarOne|
|13|MaxbotixSerial|
|14|TeraRangerI2C|
|15|LidarLiteV3-I2C|
|16|VL53L0X or VL53L1X|
|17|NMEA|
|18|WASP-LRF|
|19|BenewakeTF02|
|20|Benewake-Serial|
|21|LidarLightV3HP|
|22|PWM|
|23|BlueRoboticsPing|
|24|DroneCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|34|Benewake_CAN|
|35|TeraRangerSerial|
|36|Lua_Scripting|
|37|NoopLoop_TOFSense|
|100|SITL|

## RNGFNDA_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|Pixracer|
|13|Pixhawk ADC4|
|14|Pixhawk ADC3|
|15|Pixhawk ADC6/Pixhawk2 ADC|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|103|Pixhawk SBUS|

## RNGFNDA_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.

- Units: m/V

- Increment: 0.001

## RNGFNDA_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFNDA_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFNDA_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFNDA_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFNDA_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFNDA_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFNDA_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFNDA_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFNDA_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFNDA_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFNDA_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFNDA_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFNDA_ORIENT: Rangefinder orientation

*Note: This parameter is for advanced users*

Orientation of rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|1|Forward-Right|
|2|Right|
|3|Back-Right|
|4|Back|
|5|Back-Left|
|6|Left|
|7|Forward-Left|
|24|Up|
|25|Down|

## RNGFNDA_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFNDA_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFNDA_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFNDA_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFNDA_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFNDA_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

## RNGFNDA_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

## RNGFNDA_SNR_MIN: Minimum signal strength

*Note: This parameter is for advanced users*

Minimum signal strength (SNR) to accept distance

- Range: 0 65535

## RNGFNDA_RECV_ID: CAN receive ID

*Note: This parameter is for advanced users*

The receive ID of the CAN frames. A value of zero means all IDs are accepted.

- Range: 0 65535

# SCR Parameters

## SCR_ENABLE: Enable Scripting

*Note: This parameter is for advanced users*

Controls if scripting is enabled

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Lua Scripts|

- RebootRequired: True

## SCR_VM_I_COUNT: Scripting Virtual Machine Instruction Count

*Note: This parameter is for advanced users*

The number virtual machine instructions that can be run before considering a script to have taken an excessive amount of time

- Range: 1000 1000000

- Increment: 10000

## SCR_HEAP_SIZE: Scripting Heap Size

*Note: This parameter is for advanced users*

Amount of memory available for scripting

- Range: 1024 1048576

- Increment: 1024

- RebootRequired: True

## SCR_DEBUG_OPTS: Scripting Debug Level

*Note: This parameter is for advanced users*

Debugging options

- Bitmask: 0:No Scripts to run message if all scripts have stopped, 1:Runtime messages for memory usage and execution time, 2:Suppress logging scripts to dataflash, 3:log runtime memory usage and execution time, 4:Disable pre-arm check

## SCR_USER1: Scripting User Parameter1

General purpose user variable input for scripts

## SCR_USER2: Scripting User Parameter2

General purpose user variable input for scripts

## SCR_USER3: Scripting User Parameter3

General purpose user variable input for scripts

## SCR_USER4: Scripting User Parameter4

General purpose user variable input for scripts

## SCR_USER5: Scripting User Parameter5

General purpose user variable input for scripts

## SCR_USER6: Scripting User Parameter6

General purpose user variable input for scripts

## SCR_DIR_DISABLE: Directory disable

*Note: This parameter is for advanced users*

This will stop scripts being loaded from the given locations

- Bitmask: 0:ROMFS, 1:APM/scripts

- RebootRequired: True

# SERIAL Parameters

## SERIAL0_BAUD: Serial0 baud rate

The baud rate used on the USB console. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL0_PROTOCOL: Console protocol selection

Control what protocol to use on the console. 

|Value|Meaning|
|:---:|:---:|
|1|MAVlink1|
|2|MAVLink2|

- RebootRequired: True

## SERIAL1_PROTOCOL: Telem1 protocol selection

Control what protocol to use on the Telem1 port. Note that the Frsky options require external converter hardware. See the wiki for details.

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

- RebootRequired: True

## SERIAL1_BAUD: Telem1 Baud Rate

The baud rate used on the Telem1 port. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL2_PROTOCOL: Telemetry 2 protocol selection

Control what protocol to use on the Telem2 port. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL2_BAUD: Telemetry 2 Baud Rate

The baud rate of the Telem2 port. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL3_PROTOCOL: Serial 3 (GPS) protocol selection

Control what protocol Serial 3 (GPS) should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL3_BAUD: Serial 3 (GPS) Baud Rate

The baud rate used for the Serial 3 (GPS). Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL4_PROTOCOL: Serial4 protocol selection

Control what protocol Serial4 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL4_BAUD: Serial 4 Baud Rate

The baud rate used for Serial4. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL5_PROTOCOL: Serial5 protocol selection

Control what protocol Serial5 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL5_BAUD: Serial 5 Baud Rate

The baud rate used for Serial5. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL6_PROTOCOL: Serial6 protocol selection

Control what protocol Serial6 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL6_BAUD: Serial 6 Baud Rate

The baud rate used for Serial6. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL1_OPTIONS: Telem1 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL2_OPTIONS: Telem2 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL3_OPTIONS: Serial3 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL4_OPTIONS: Serial4 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL5_OPTIONS: Serial5 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL6_OPTIONS: Serial6 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL_PASS1: Serial passthru first port

*Note: This parameter is for advanced users*

This sets one side of pass-through between two serial ports. Once both sides are set then all data received on either port will be passed to the other port

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Serial0|
|1|Serial1|
|2|Serial2|
|3|Serial3|
|4|Serial4|
|5|Serial5|
|6|Serial6|

## SERIAL_PASS2: Serial passthru second port

*Note: This parameter is for advanced users*

This sets one side of pass-through between two serial ports. Once both sides are set then all data received on either port will be passed to the other port

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Serial0|
|1|Serial1|
|2|Serial2|
|3|Serial3|
|4|Serial4|
|5|Serial5|
|6|Serial6|

## SERIAL_PASSTIMO: Serial passthru timeout

*Note: This parameter is for advanced users*

This sets a timeout for serial pass-through in seconds. When the pass-through is enabled by setting the SERIAL_PASS1 and SERIAL_PASS2 parameters then it remains in effect until no data comes from the first port for SERIAL_PASSTIMO seconds. This allows the port to revent to its normal usage (such as MAVLink connection to a GCS) when it is no longer needed. A value of 0 means no timeout.

- Range: 0 120

- Units: s

## SERIAL7_PROTOCOL: Serial7 protocol selection

Control what protocol Serial7 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL7_BAUD: Serial 7 Baud Rate

The baud rate used for Serial7. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL7_OPTIONS: Serial7 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL8_PROTOCOL: Serial8 protocol selection

Control what protocol Serial8 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL8_BAUD: Serial 8 Baud Rate

The baud rate used for Serial8. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL8_OPTIONS: Serial8 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL9_PROTOCOL: Serial9 protocol selection

Control what protocol Serial9 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|-1|None|
|1|MAVLink1|
|2|MAVLink2|
|3|Frsky D|
|4|Frsky SPort|
|5|GPS|
|7|Alexmos Gimbal Serial|
|8|Gimbal|
|9|Rangefinder|
|10|FrSky SPort Passthrough (OpenTX)|
|11|Lidar360|
|13|Beacon|
|14|Volz servo out|
|15|SBus servo out|
|16|ESC Telemetry|
|17|Devo Telemetry|
|18|OpticalFlow|
|19|RobotisServo|
|20|NMEA Output|
|21|WindVane|
|22|SLCAN|
|23|RCIN|
|24|EFI Serial|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire VTX|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|
|39|Torqeedo|
|40|AIS|
|41|CoDevESC|
|42|DisplayPort|
|43|MAVLink High Latency|
|44|IRC Tramp|

## SERIAL9_BAUD: Serial 9 Baud Rate

The baud rate used for Serial8. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

|Value|Meaning|
|:---:|:---:|
|1|1200|
|2|2400|
|4|4800|
|9|9600|
|19|19200|
|38|38400|
|57|57600|
|111|111100|
|115|115200|
|230|230400|
|256|256000|
|460|460800|
|500|500000|
|921|921600|
|1500|1500000|
|2000|2000000|

## SERIAL9_OPTIONS: Serial9 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

# TEMP Parameters

## TEMP_LOG: Logging

Enables temperature sensor logging

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

# TEMP1 Parameters

## TEMP1_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP1_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP1_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP1_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP1_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP2 Parameters

## TEMP2_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP2_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP2_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP2_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP2_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP3 Parameters

## TEMP3_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP3_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP3_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP3_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP3_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP4 Parameters

## TEMP4_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP4_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP4_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP4_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP4_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP5 Parameters

## TEMP5_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP5_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP5_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP5_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP5_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP6 Parameters

## TEMP6_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP6_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP6_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP6_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP6_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP7 Parameters

## TEMP7_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP7_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP7_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP7_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP7_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP8 Parameters

## TEMP8_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP8_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP8_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP8_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP8_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# TEMP9 Parameters

## TEMP9_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865|

- RebootRequired: True

## TEMP9_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP9_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP9_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor(not implemented yet)|
|3|Battery Index|
|4|Battery ID/SerialNumber|

## TEMP9_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

# VISO Parameters

## VISO_TYPE: Visual odometry camera connection type

*Note: This parameter is for advanced users*

Visual odometry camera connection type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|MAVLink|
|2|IntelT265|
|3|VOXL(ModalAI)|

- RebootRequired: True

## VISO_POS_X: Visual odometry camera X position offset

*Note: This parameter is for advanced users*

X position of the camera in body frame. Positive X is forward of the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## VISO_POS_Y: Visual odometry camera Y position offset

*Note: This parameter is for advanced users*

Y position of the camera in body frame. Positive Y is to the right of the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## VISO_POS_Z: Visual odometry camera Z position offset

*Note: This parameter is for advanced users*

Z position of the camera in body frame. Positive Z is down from the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## VISO_ORIENT: Visual odometery camera orientation

*Note: This parameter is for advanced users*

Visual odometery camera orientation

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|2|Right|
|4|Back|
|6|Left|
|24|Up|
|25|Down|

## VISO_SCALE: Visual odometry scaling factor

*Note: This parameter is for advanced users*

Visual odometry scaling factor applied to position estimates from sensor

## VISO_DELAY_MS: Visual odometry sensor delay

*Note: This parameter is for advanced users*

Visual odometry sensor delay relative to inertial measurements

- Units: ms

- Range: 0 250

## VISO_VEL_M_NSE: Visual odometry velocity measurement noise

*Note: This parameter is for advanced users*

Visual odometry velocity measurement noise in m/s

- Units: m/s

- Range: 0.05 5.0

## VISO_POS_M_NSE: Visual odometry position measurement noise 

*Note: This parameter is for advanced users*

Visual odometry position measurement noise minimum (meters). This value will be used if the sensor provides a lower noise value (or no noise value)

- Units: m

- Range: 0.1 10.0

## VISO_YAW_M_NSE: Visual odometry yaw measurement noise

*Note: This parameter is for advanced users*

Visual odometry yaw measurement noise minimum (radians), This value will be used if the sensor provides a lower noise value (or no noise value)

- Units: rad

- Range: 0.05 1.0

# VTX Parameters

## VTX_ENABLE: Is the Video Transmitter enabled or not

Toggles the Video Transmitter on and off

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## VTX_POWER: Video Transmitter Power Level

Video Transmitter Power Level. Different VTXs support different power levels, the power level chosen will be rounded down to the nearest supported power level

- Range: 1 1000

## VTX_CHANNEL: Video Transmitter Channel

Video Transmitter Channel

- Range: 0 7

## VTX_BAND: Video Transmitter Band

Video Transmitter Band

|Value|Meaning|
|:---:|:---:|
|0|Band A|
|1|Band B|
|2|Band E|
|3|Airwave|
|4|RaceBand|
|5|Low RaceBand|

## VTX_FREQ: Video Transmitter Frequency

Video Transmitter Frequency. The frequency is derived from the setting of BAND and CHANNEL

- ReadOnly: True

- Range: 5000 6000

## VTX_OPTIONS: Video Transmitter Options

*Note: This parameter is for advanced users*

Video Transmitter Options. Pitmode puts the VTX in a low power state. Unlocked enables certain restricted frequencies and power levels. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels. One stop-bit may be required for VTXs that erroneously mimic iNav behaviour.

- Bitmask: 0:Pitmode,1:Pitmode until armed,2:Pitmode when disarmed,3:Unlocked,4:Add leading zero byte to requests,5:Use 1 stop-bit in SmartAudio,6:Ignore CRC in SmartAudio,7:Ignore status updates in CRSF and blindly set VTX options

## VTX_MAX_POWER: Video Transmitter Max Power Level

Video Transmitter Maximum Power Level. Different VTXs support different power levels, this prevents the power aux switch from requesting too high a power level. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting.

- Range: 25 1000