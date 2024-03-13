
This is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your autopilot to control vehicle behaviour. This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code. Some parameters may only be available for developers, and are enabled at compile-time.

# ArduCopter Parameters

## FORMAT_VERSION: Eeprom format version number

*Note: This parameter is for advanced users*

This value is incremented when changes are made to the eeprom format

- ReadOnly: True

## SYSID_THISMAV: MAVLink system ID of this vehicle

*Note: This parameter is for advanced users*

Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network

- Range: 1 255

## SYSID_MYGCS: My ground station number

*Note: This parameter is for advanced users*

Allows restricting radio overrides to only come from my ground station

- Range: 1 255

## PILOT_THR_FILT: Throttle filter cutoff

*Note: This parameter is for advanced users*

Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable

- Units: Hz

- Range: 0 10

- Increment: .5

## PILOT_TKOFF_ALT: Pilot takeoff altitude

Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.

- Units: cm

- Range: 0.0 1000.0

- Increment: 10

## PILOT_THR_BHV: Throttle stick behavior

Bitmask containing various throttle stick options. TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Feedback from mid stick|
|2|High throttle cancels landing|
|4|Disarm on land detection|

- Bitmask: 0:Feedback from mid stick,1:High throttle cancels landing,2:Disarm on land detection

## TELEM_DELAY: Telemetry startup delay

*Note: This parameter is for advanced users*

The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up

- Units: s

- Range: 0 30

- Increment: 1

## GCS_PID_MASK: GCS PID tuning mask

*Note: This parameter is for advanced users*

bitmask of PIDs to send MAVLink PID_TUNING messages for

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Roll|
|2|Pitch|
|4|Yaw|
|8|AccelZ|

- Bitmask: 0:Roll,1:Pitch,2:Yaw,3:AccelZ

## RTL_ALT: RTL Altitude

The minimum alt above home the vehicle will climb to before returning.  If the vehicle is flying higher than this value it will return at its current altitude.

- Units: cm

- Range: 200 300000

- Increment: 1

## RTL_CONE_SLOPE: RTL cone slope

Defines a cone above home which determines maximum climb

- Range: 0.5 10.0

- Increment: .1

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Shallow|
|3|Steep|

## RTL_SPEED: RTL speed

Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.

- Units: cm/s

- Range: 0 2000

- Increment: 50

## RTL_ALT_FINAL: RTL Final Altitude

This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.

- Units: cm

- Range: 0 1000

- Increment: 1

## RTL_CLIMB_MIN: RTL minimum climb

The vehicle will climb this many cm during the initial climb portion of the RTL

- Units: cm

- Range: 0 3000

- Increment: 10

## RTL_LOIT_TIME: RTL loiter time

Time (in milliseconds) to loiter above home before beginning final descent

- Units: ms

- Range: 0 60000

- Increment: 1000

## RTL_ALT_TYPE: RTL mode altitude type

RTL altitude type.  Set to 1 for Terrain following during RTL and then set WPNAV_RFND_USE=1 to use rangefinder or WPNAV_RFND_USE=0 to use Terrain database

|Value|Meaning|
|:---:|:---:|
|0|Relative to Home|
|1|Terrain|

## FS_GCS_ENABLE: Ground Station Failsafe Enable

Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. See FS_OPTIONS param for additional actions, or for cases allowing Mission continuation, when GCS failsafe is enabled.

|Value|Meaning|
|:---:|:---:|
|0|Disabled/NoAction|
|1|RTL|
|2|RTL or Continue with Mission in Auto Mode (Removed in 4.0+-see FS_OPTIONS)|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Land|
|6|Auto DO_LAND_START or RTL|

## GPS_HDOP_GOOD: GPS Hdop Good

*Note: This parameter is for advanced users*

GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks

- Range: 100 900

## SUPER_SIMPLE: Super Simple Mode

Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode. The bitmask is for flight mode switch positions

- Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6

## WP_YAW_BEHAVIOR: Yaw behaviour during missions

Determines how the autopilot controls the yaw during missions and RTL

|Value|Meaning|
|:---:|:---:|
|0|Never change yaw|
|1|Face next waypoint|
|2|Face next waypoint except RTL|
|3|Face along GPS course|

## LAND_SPEED: Land speed

The descent speed for the final stage of landing in cm/s

- Units: cm/s

- Range: 30 200

- Increment: 10

## LAND_SPEED_HIGH: Land speed high

The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used

- Units: cm/s

- Range: 0 500

- Increment: 10

## PILOT_SPEED_UP: Pilot maximum vertical speed ascending

The maximum vertical ascending velocity the pilot may request in cm/s

- Units: cm/s

- Range: 50 500

- Increment: 10

## PILOT_ACCEL_Z: Pilot vertical acceleration

The vertical acceleration used when pilot is controlling the altitude

- Units: cm/s/s

- Range: 50 500

- Increment: 10

## FS_THR_ENABLE: Throttle Failsafe Enable

The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled always RTL|
|2|Enabled Continue with Mission in Auto Mode (Removed in 4.0+)|
|3|Enabled always Land|
|4|Enabled always SmartRTL or RTL|
|5|Enabled always SmartRTL or Land|
|6|Enabled Auto DO_LAND_START or RTL|

## FS_THR_VALUE: Throttle Failsafe Value

The PWM level in microseconds on channel 3 below which throttle failsafe triggers

- Range: 910 1100

- Units: PWM

- Increment: 1

## THR_DZ: Throttle deadzone

The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes

- Range: 0 300

- Units: PWM

- Increment: 1

## FLTMODE1: Flight Mode 1

Flight mode when pwm of Flightmode channel(FLTMODE_CH) is <= 1230

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|Smart_RTL|
|22|FlowHold|
|23|Follow|
|24|ZigZag|
|25|SystemID|
|26|Heli_Autorotate|
|27|Auto RTL|

## FLTMODE2: Flight Mode 2

Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1230, <= 1360

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|Smart_RTL|
|22|FlowHold|
|23|Follow|
|24|ZigZag|
|25|SystemID|
|26|Heli_Autorotate|
|27|Auto RTL|

## FLTMODE3: Flight Mode 3

Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1360, <= 1490

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|Smart_RTL|
|22|FlowHold|
|23|Follow|
|24|ZigZag|
|25|SystemID|
|26|Heli_Autorotate|
|27|Auto RTL|

## FLTMODE4: Flight Mode 4

Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1490, <= 1620

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|Smart_RTL|
|22|FlowHold|
|23|Follow|
|24|ZigZag|
|25|SystemID|
|26|Heli_Autorotate|
|27|Auto RTL|

## FLTMODE5: Flight Mode 5

Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1620, <= 1749

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|Smart_RTL|
|22|FlowHold|
|23|Follow|
|24|ZigZag|
|25|SystemID|
|26|Heli_Autorotate|
|27|Auto RTL|

## FLTMODE6: Flight Mode 6

Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >=1750

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|Smart_RTL|
|22|FlowHold|
|23|Follow|
|24|ZigZag|
|25|SystemID|
|26|Heli_Autorotate|
|27|Auto RTL|

## FLTMODE_CH: Flightmode channel

*Note: This parameter is for advanced users*

RC Channel to use for flight mode control

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|5|Channel5|
|6|Channel6|
|7|Channel7|
|8|Channel8|
|9|Channel9|
|10|Channel 10|
|11|Channel 11|
|12|Channel 12|
|13|Channel 13|
|14|Channel 14|
|15|Channel 15|

## INITIAL_MODE: Initial flight mode

*Note: This parameter is for advanced users*

This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver.

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|Smart_RTL|
|22|FlowHold|
|23|Follow|
|24|ZigZag|
|25|SystemID|
|26|Heli_Autorotate|

## SIMPLE: Simple mode bitmask

*Note: This parameter is for advanced users*

Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode). The bitmask is for flightmode switch positions.

- Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6

## LOG_BITMASK: Log bitmask

Bitmap of what on-board log types to enable. This value is made up of the sum of each of the log types you want to be saved. It is usually best just to enable all basiclog types by setting this to 65535.

- Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System Performance,4:Control Tuning,5:Navigation Tuning,6:RC input,7:IMU,8:Mission Commands,9:Battery Monitor,10:RC output,11:Optical Flow,12:PID,13:Compass,15:Camera,17:Motors,18:Fast IMU,19:Raw IMU,20:Video Stabilization,21:Fast harmonic notch logging

## ESC_CALIBRATION: ESC Calibration

*Note: This parameter is for advanced users*

Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.

|Value|Meaning|
|:---:|:---:|
|0|Normal Start-up|
|1|Start-up in ESC Calibration mode if throttle high|
|2|Start-up in ESC Calibration mode regardless of throttle|
|3|Start-up and automatically calibrate ESCs|
|9|Disabled|

## TUNE: Channel 6 Tuning

Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Stab Roll/Pitch kP|
|4|Rate Roll/Pitch kP|
|5|Rate Roll/Pitch kI|
|21|Rate Roll/Pitch kD|
|3|Stab Yaw kP|
|6|Rate Yaw kP|
|26|Rate Yaw kD|
|56|Rate Yaw Filter|
|55|Motor Yaw Headroom|
|14|AltHold kP|
|7|Throttle Rate kP|
|34|Throttle Accel kP|
|35|Throttle Accel kI|
|36|Throttle Accel kD|
|12|Loiter Pos kP|
|22|Velocity XY kP|
|28|Velocity XY kI|
|10|WP Speed|
|25|Acro Roll/Pitch deg/s|
|40|Acro Yaw deg/s|
|45|RC Feel|
|13|Heli Ext Gyro|
|38|Declination|
|39|Circle Rate|
|46|Rate Pitch kP|
|47|Rate Pitch kI|
|48|Rate Pitch kD|
|49|Rate Roll kP|
|50|Rate Roll kI|
|51|Rate Roll kD|
|52|Rate Pitch FF|
|53|Rate Roll FF|
|54|Rate Yaw FF|
|58|SysID Magnitude|

## FRAME_TYPE: Frame Type (+, X, V, etc)

Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.

|Value|Meaning|
|:---:|:---:|
|0|Plus|
|1|X|
|2|V|
|3|H|
|4|V-Tail|
|5|A-Tail|
|10|Y6B|
|11|Y6F|
|12|BetaFlightX|
|13|DJIX|
|14|ClockwiseX|
|15|I|
|18|BetaFlightXReversed|
|19|Y4|

- RebootRequired: True

## DISARM_DELAY: Disarm delay

*Note: This parameter is for advanced users*

Delay before automatic disarm in seconds after landing touchdown detection. A value of zero disables auto disarm. If Emergency Motor stop active, delay time is half this value.

- Units: s

- Range: 0 127

## ANGLE_MAX: Angle Max

*Note: This parameter is for advanced users*

Maximum lean angle in all flight modes

- Units: cdeg

- Increment: 10

- Range: 1000 8000

## PHLD_BRAKE_RATE: PosHold braking rate

*Note: This parameter is for advanced users*

PosHold flight mode's rotation rate during braking in deg/sec

- Units: deg/s

- Range: 4 12

## PHLD_BRAKE_ANGLE: PosHold braking angle max

*Note: This parameter is for advanced users*

PosHold flight mode's max lean angle during braking in centi-degrees

- Units: cdeg

- Increment: 10

- Range: 2000 4500

## LAND_REPOSITION: Land repositioning

*Note: This parameter is for advanced users*

Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.

|Value|Meaning|
|:---:|:---:|
|0|No repositioning|
|1|Repositioning|

## FS_EKF_ACTION: EKF Failsafe Action

*Note: This parameter is for advanced users*

Controls the action that will be taken when an EKF failsafe is invoked

|Value|Meaning|
|:---:|:---:|
|1|Land|
|2|AltHold|
|3|Land even in Stabilize|

## FS_EKF_THRESH: EKF failsafe variance threshold

*Note: This parameter is for advanced users*

Allows setting the maximum acceptable compass, velocity, position and height variances. Used in arming check and EKF failsafe.

|Value|Meaning|
|:---:|:---:|
|0.6|Strict|
|0.8|Default|
|1.0|Relaxed|

## FS_CRASH_CHECK: Crash check enable

*Note: This parameter is for advanced users*

This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## RC_SPEED: ESC Update Speed

*Note: This parameter is for advanced users*

This is the speed in Hertz that your ESCs will receive updates

- Units: Hz

- Range: 50 490

- Increment: 1

## ACRO_BAL_ROLL: Acro Balance Roll

*Note: This parameter is for advanced users*

rate at which roll angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the roll axis. A higher value causes faster decay of desired to actual attitude.

- Range: 0 3

- Increment: 0.1

## ACRO_BAL_PITCH: Acro Balance Pitch

*Note: This parameter is for advanced users*

rate at which pitch angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the pitch axis. A higher value causes faster decay of desired to actual attitude.

- Range: 0 3

- Increment: 0.1

## ACRO_TRAINER: Acro Trainer

*Note: This parameter is for advanced users*

Type of trainer used in acro mode

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Leveling|
|2|Leveling and Limited|

## THROW_MOT_START: Start motors before throwing is detected

Used by Throw mode. Controls whether motors will run at the speed set by MOT_SPIN_MIN or will be stopped when armed and waiting for the throw.

|Value|Meaning|
|:---:|:---:|
|0|Stopped|
|1|Running|

## WP_NAVALT_MIN: Minimum navigation altitude

This is the altitude in meters above which for navigation can begin. This applies in auto takeoff and auto landing.

- Range: 0 5

## THROW_NEXTMODE: Throw mode's follow up mode

Vehicle will switch to this mode after the throw is successfully completed.  Default is to stay in throw mode (18)

|Value|Meaning|
|:---:|:---:|
|3|Auto|
|4|Guided|
|5|LOITER|
|6|RTL|
|9|Land|
|17|Brake|
|18|Throw|

## THROW_TYPE: Type of Type

Used by Throw mode. Specifies whether Copter is thrown upward or dropped.

|Value|Meaning|
|:---:|:---:|
|0|Upward Throw|
|1|Drop|

## GND_EFFECT_COMP: Ground Effect Compensation Enable/Disable

*Note: This parameter is for advanced users*

Ground Effect Compensation Enable/Disable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## DEV_OPTIONS: Development options

*Note: This parameter is for advanced users*

Bitmask of developer options. The meanings of the bit fields in this parameter may vary at any time. Developers should check the source code for current meaning

- Bitmask: 0:ADSBMavlinkProcessing,1:DevOptionVFR_HUDRelativeAlt

## ACRO_THR_MID: Acro Thr Mid

*Note: This parameter is for advanced users*

Acro Throttle Mid

- Range: 0 1

## SYSID_ENFORCE: GCS sysid enforcement

*Note: This parameter is for advanced users*

This controls whether packets from other than the expected GCS system ID will be accepted

|Value|Meaning|
|:---:|:---:|
|0|NotEnforced|
|1|Enforced|

## FRAME_CLASS: Frame Class

Controls major frame class for multicopter component

|Value|Meaning|
|:---:|:---:|
|0|Undefined|
|1|Quad|
|2|Hexa|
|3|Octa|
|4|OctaQuad|
|5|Y6|
|6|Heli|
|7|Tri|
|8|SingleCopter|
|9|CoaxCopter|
|10|BiCopter|
|11|Heli_Dual|
|12|DodecaHexa|
|13|HeliQuad|
|14|Deca|
|15|Scripting Matrix|
|16|6DoF Scripting|
|17|Dynamic Scripting Matrix|

- RebootRequired: True

## PILOT_SPEED_DN: Pilot maximum vertical speed descending

The maximum vertical descending velocity the pilot may request in cm/s.  If 0 PILOT_SPEED_UP value is used.

- Units: cm/s

- Range: 0 500

- Increment: 10

## LAND_ALT_LOW: Land alt low

*Note: This parameter is for advanced users*

Altitude during Landing at which vehicle slows to LAND_SPEED

- Units: cm

- Range: 100 10000

- Increment: 10

## TUNE_MIN: Tuning minimum

Minimum value that the parameter currently being tuned with the transmitter's channel 6 knob will be set to

## TUNE_MAX: Tuning maximum

Maximum value that the parameter currently being tuned with the transmitter's channel 6 knob will be set to

## FS_VIBE_ENABLE: Vibration Failsafe enable

This enables the vibration failsafe which will use modified altitude estimation and control during high vibrations

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## FS_OPTIONS: Failsafe options bitmask

*Note: This parameter is for advanced users*

Bitmask of additional options for battery, radio, & GCS failsafes. 0 (default) disables all options.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Continue if in Auto on RC failsafe only|
|2|Continue if in Auto on GCS failsafe only|
|3|Continue if in Auto on RC and/or GCS failsafe|
|4|Continue if in Guided on RC failsafe only|
|8|Continue if landing on any failsafe|
|16|Continue if in pilot controlled modes on GCS failsafe|
|19|Continue if in Auto on RC and/or GCS failsafe and continue if in pilot controlled modes on GCS failsafe|

- Bitmask: 0:Continue if in Auto on RC failsafe, 1:Continue if in Auto on GCS failsafe, 2:Continue if in Guided on RC failsafe, 3:Continue if landing on any failsafe, 4:Continue if in pilot controlled modes on GCS failsafe, 5:Release Gripper

## ACRO_OPTIONS: Acro mode options

*Note: This parameter is for advanced users*

A range of options that can be applied to change acro mode behaviour. Air-mode enables ATC_THR_MIX_MAN at all times (air-mode has no effect on helicopters). Rate Loop Only disables the use of angle stabilization and uses angular rate stabilization only.

- Bitmask: 0:Air-mode,1:Rate Loop Only

## AUTO_OPTIONS: Auto mode options

*Note: This parameter is for advanced users*

A range of options that can be applied to change auto mode behaviour. Allow Arming allows the copter to be armed in Auto. Allow Takeoff Without Raising Throttle allows takeoff without the pilot having to raise the throttle. Ignore pilot yaw overrides the pilot's yaw stick being used while in auto.

- Bitmask: 0:Allow Arming,1:Allow Takeoff Without Raising Throttle,2:Ignore pilot yaw

## GUID_OPTIONS: Guided mode options

*Note: This parameter is for advanced users*

Options that can be applied to change guided mode behaviour

- Bitmask: 0:Allow Arming from Transmitter,2:Ignore pilot yaw,3:SetAttitudeTarget interprets Thrust As Thrust,4:Do not stabilize PositionXY,5:Do not stabilize VelocityXY,6:Waypoint navigation used for position targets

## FS_GCS_TIMEOUT: GCS failsafe timeout

Timeout before triggering the GCS failsafe

- Units: s

- Range: 2 120

- Increment: 1

## RTL_OPTIONS: RTL mode options

*Note: This parameter is for advanced users*

Options that can be applied to change RTL mode behaviour

- Bitmask: 2:Ignore pilot yaw

## FLIGHT_OPTIONS: Flight mode options

*Note: This parameter is for advanced users*

Flight mode specific options

- Bitmask: 0:Disable thrust loss check, 1:Disable yaw imbalance warning, 2:Release gripper on thrust loss

## RNGFND_FILT: Rangefinder filter

Rangefinder filter to smooth distance.  Set to zero to disable filtering

- Units: Hz

- Range: 0 5

- Increment: 0.05

- RebootRequired: True

## GUID_TIMEOUT: Guided mode timeout

*Note: This parameter is for advanced users*

Guided mode timeout after which vehicle will stop or return to level if no updates are received from caller. Only applicable during any combination of velocity, acceleration, angle control, and/or angular rate control

- Units: s

- Range: 0.1 5

## SURFTRAK_MODE: Surface Tracking Mode

*Note: This parameter is for advanced users*

set which surface to track in surface tracking

|Value|Meaning|
|:---:|:---:|
|0|Do not track|
|1|Ground|
|2|Ceiling|

## FS_DR_ENABLE: DeadReckon Failsafe Action

Failsafe action taken immediately as deadreckoning starts. Deadreckoning starts when EKF loses position and velocity source and relies only on wind estimates

|Value|Meaning|
|:---:|:---:|
|0|Disabled/NoAction|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|6|Auto DO_LAND_START or RTL|

## FS_DR_TIMEOUT: DeadReckon Failsafe Timeout

DeadReckoning is available for this many seconds after losing position and/or velocity source.  After this timeout elapses the EKF failsafe will trigger in modes requiring a position estimate

- Range: 0 120

## ACRO_RP_RATE: Acro Roll and Pitch Rate

Acro mode maximum roll and pitch rate.  Higher values mean faster rate of rotation

- Units: deg/s

- Range: 1 1080

## ACRO_RP_EXPO: Acro Roll/Pitch Expo

*Note: This parameter is for advanced users*

Acro roll/pitch Expo to allow faster rotation when stick at edges

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|0.1|Very Low|
|0.2|Low|
|0.3|Medium|
|0.4|High|
|0.5|Very High|

- Range: -0.5 0.95

## ACRO_RP_RATE_TC: Acro roll/pitch rate control input time constant

Acro roll and pitch rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response

- Units: s

- Range: 0 1

- Increment: 0.01

|Value|Meaning|
|:---:|:---:|
|0.5|Very Soft|
|0.2|Soft|
|0.15|Medium|
|0.1|Crisp|
|0.05|Very Crisp|

## ACRO_Y_RATE: Acro Yaw Rate

Acro mode maximum yaw rate.  Higher value means faster rate of rotation

- Units: deg/s

- Range: 1 360

## ACRO_Y_EXPO: Acro Yaw Expo

*Note: This parameter is for advanced users*

Acro yaw expo to allow faster rotation when stick at edges

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|0.1|Very Low|
|0.2|Low|
|0.3|Medium|
|0.4|High|
|0.5|Very High|

- Range: -1.0 0.95

## ACRO_Y_RATE_TC: Acro yaw rate control input time constant

Acro yaw rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response

- Units: s

- Range: 0 1

- Increment: 0.01

|Value|Meaning|
|:---:|:---:|
|0.5|Very Soft|
|0.2|Soft|
|0.15|Medium|
|0.1|Crisp|
|0.05|Very Crisp|

## PILOT_Y_RATE: Pilot controlled yaw rate

Pilot controlled yaw rate max.  Used in all pilot controlled modes except Acro

- Units: deg/s

- Range: 1 360

## PILOT_Y_EXPO: Pilot controlled yaw expo

*Note: This parameter is for advanced users*

Pilot controlled yaw expo to allow faster rotation when stick at edges

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|0.1|Very Low|
|0.2|Low|
|0.3|Medium|
|0.4|High|
|0.5|Very High|

- Range: -0.5 1.0

## PILOT_Y_RATE_TC: Pilot yaw rate control input time constant

Pilot yaw rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response

- Units: s

- Range: 0 1

- Increment: 0.01

|Value|Meaning|
|:---:|:---:|
|0.5|Very Soft|
|0.2|Soft|
|0.15|Medium|
|0.1|Crisp|
|0.05|Very Crisp|

## TKOFF_SLEW_TIME: Slew time of throttle during take-off

Time to slew the throttle from minimum to maximum while checking for a succsessful takeoff.

- Units: s

- Range: 0.25 5.0

## TKOFF_RPM_MIN: Takeoff Check RPM minimum

Takeoff is not permitted until motors report at least this RPM.  Set to zero to disable check

- Range: 0 10000

# ADSB Parameters

## ADSB_TYPE: ADSB Type

Type of ADS-B hardware for ADSB-in and ADSB-out configuration and operation. If any type is selected then MAVLink based ADSB-in messages will always be enabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|uAvionix-MAVLink|
|2|Sagetech|
|3|uAvionix-UCP|
|4|Sagetech MX Series|

- RebootRequired: True

## ADSB_LIST_MAX: ADSB vehicle list size

*Note: This parameter is for advanced users*

ADSB list size of nearest vehicles. Longer lists take longer to refresh with lower SRx_ADSB values.

- Range: 1 100

- RebootRequired: True

## ADSB_LIST_RADIUS: ADSB vehicle list radius filter

*Note: This parameter is for advanced users*

ADSB vehicle list radius filter. Vehicles detected outside this radius will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations. A value of 0 will disable this filter.

- Range: 0 100000

- Units: m

## ADSB_ICAO_ID: ICAO_ID vehicle identification number

*Note: This parameter is for advanced users*

ICAO_ID unique vehicle identification number of this aircraft. This is an integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.

- Range: -1 16777215

## ADSB_EMIT_TYPE: Emitter type

*Note: This parameter is for advanced users*

ADSB classification for the type of vehicle emitting the transponder signal. Default value is 14 (UAV).

|Value|Meaning|
|:---:|:---:|
|0|NoInfo|
|1|Light|
|2|Small|
|3|Large|
|4|HighVortexlarge|
|5|Heavy|
|6|HighlyManuv|
|7|Rotocraft|
|8|RESERVED|
|9|Glider|
|10|LightAir|
|11|Parachute|
|12|UltraLight|
|13|RESERVED|
|14|UAV|
|15|Space|
|16|RESERVED|
|17|EmergencySurface|
|18|ServiceSurface|
|19|PointObstacle|

## ADSB_LEN_WIDTH: Aircraft length and width

*Note: This parameter is for advanced users*

Aircraft length and width dimension options in Length and Width in meters. In most cases, use a value of 1 for smallest size.

|Value|Meaning|
|:---:|:---:|
|0|NO_DATA|
|1|L15W23|
|2|L25W28P5|
|3|L25W34|
|4|L35W33|
|5|L35W38|
|6|L45W39P5|
|7|L45W45|
|8|L55W45|
|9|L55W52|
|10|L65W59P5|
|11|L65W67|
|12|L75W72P5|
|13|L75W80|
|14|L85W80|
|15|L85W90|

## ADSB_OFFSET_LAT: GPS antenna lateral offset

*Note: This parameter is for advanced users*

GPS antenna lateral offset. This describes the physical location offest from center of the GPS antenna on the aircraft.

|Value|Meaning|
|:---:|:---:|
|0|NoData|
|1|Left2m|
|2|Left4m|
|3|Left6m|
|4|Center|
|5|Right2m|
|6|Right4m|
|7|Right6m|

## ADSB_OFFSET_LON: GPS antenna longitudinal offset

*Note: This parameter is for advanced users*

GPS antenna longitudinal offset. This is usually set to 1, Applied By Sensor

|Value|Meaning|
|:---:|:---:|
|0|NO_DATA|
|1|AppliedBySensor|

## ADSB_RF_SELECT: Transceiver RF selection

*Note: This parameter is for advanced users*

Transceiver RF selection for Rx enable and/or Tx enable. This only effects devices that can Tx and/or Rx. Rx-only devices should override this to always be Rx-only.

- Bitmask: 0:Rx,1:Tx

## ADSB_SQUAWK: Squawk code

*Note: This parameter is for advanced users*

VFR squawk (Mode 3/A) code is a pre-programmed default code when the pilot is flying VFR and not in contact with ATC. In the USA, the VFR squawk code is octal 1200 (hex 0x280, decimal 640) and in most parts of Europe the VFR squawk code is octal 7000. If an invalid octal number is set then it will be reset to 1200.

- Range: 0 7777

- Units: octal

## ADSB_RF_CAPABLE: RF capabilities

*Note: This parameter is for advanced users*

Describes your hardware RF In/Out capabilities.

- Bitmask: 0:UAT_in,1:1090ES_in,2:UAT_out,3:1090ES_out

## ADSB_LIST_ALT: ADSB vehicle list altitude filter

*Note: This parameter is for advanced users*

ADSB vehicle list altitude filter. Vehicles detected above this altitude will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations. A value of 0 will disable this filter.

- Range: 0 32767

- Units: m

## ADSB_ICAO_SPECL: ICAO_ID of special vehicle

*Note: This parameter is for advanced users*

ICAO_ID of special vehicle that ignores ADSB_LIST_RADIUS and ADSB_LIST_ALT. The vehicle is always tracked. Use 0 to disable.

## ADSB_LOG: ADS-B logging

*Note: This parameter is for advanced users*

0: no logging, 1: log only special ID, 2:log all

|Value|Meaning|
|:---:|:---:|
|0|no logging|
|1|log only special ID|
|2|log all|

## ADSB_OPTIONS: ADS-B Options

*Note: This parameter is for advanced users*

Options for emergency failsafe codes and device capabilities

- Bitmask: 0:Ping200X Send GPS,1:Squawk 7400 on RC failsafe,2:Squawk 7400 on GCS failsafe,3:Sagetech MXS use External Config

# AFS Parameters

## AFS_ENABLE: Enable Advanced Failsafe

*Note: This parameter is for advanced users*

This enables the advanced failsafe system. If this is set to zero (disable) then all the other AFS options have no effect

## AFS_MAN_PIN: Manual Pin

*Note: This parameter is for advanced users*

This sets a digital output pin to set high when in manual mode.  See the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

## AFS_HB_PIN: Heartbeat Pin

*Note: This parameter is for advanced users*

This sets a digital output pin which is cycled at 10Hz when termination is not activated. Note that if a FS_TERM_PIN is set then the heartbeat pin will continue to cycle at 10Hz when termination is activated, to allow the termination board to distinguish between autopilot crash and termination. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|

## AFS_WP_COMMS: Comms Waypoint

*Note: This parameter is for advanced users*

Waypoint number to navigate to on comms loss

## AFS_WP_GPS_LOSS: GPS Loss Waypoint

*Note: This parameter is for advanced users*

Waypoint number to navigate to on GPS lock loss

## AFS_TERMINATE: Force Terminate

*Note: This parameter is for advanced users*

Can be set in flight to force termination of the heartbeat signal

## AFS_TERM_ACTION: Terminate action

*Note: This parameter is for advanced users*

This can be used to force an action on flight termination. Normally this is handled by an external failsafe board, but you can setup ArduPilot to handle it here. Please consult the wiki for more information on the possible values of the parameter

## AFS_TERM_PIN: Terminate Pin

*Note: This parameter is for advanced users*

This sets a digital output pin to set high on flight termination. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|

## AFS_AMSL_LIMIT: AMSL limit

*Note: This parameter is for advanced users*

This sets the AMSL (above mean sea level) altitude limit. If the pressure altitude determined by QNH exceeds this limit then flight termination will be forced. Note that this limit is in meters, whereas pressure altitude limits are often quoted in feet. A value of zero disables the pressure altitude limit.

- Units: m

## AFS_AMSL_ERR_GPS: Error margin for GPS based AMSL limit

*Note: This parameter is for advanced users*

This sets margin for error in GPS derived altitude limit. This error margin is only used if the barometer has failed. If the barometer fails then the GPS will be used to enforce the AMSL_LIMIT, but this margin will be subtracted from the AMSL_LIMIT first, to ensure that even with the given amount of GPS altitude error the pressure altitude is not breached. OBC users should set this to comply with their D2 safety case. A value of -1 will mean that barometer failure will lead to immediate termination.

- Units: m

## AFS_QNH_PRESSURE: QNH pressure

*Note: This parameter is for advanced users*

This sets the QNH pressure in millibars to be used for pressure altitude in the altitude limit. A value of zero disables the altitude limit.

- Units: hPa

## AFS_MAX_GPS_LOSS: Maximum number of GPS loss events

*Note: This parameter is for advanced users*

Maximum number of GPS loss events before the aircraft stops returning to mission on GPS recovery. Use zero to allow for any number of GPS loss events.

## AFS_MAX_COM_LOSS: Maximum number of comms loss events

*Note: This parameter is for advanced users*

Maximum number of comms loss events before the aircraft stops returning to mission on comms recovery. Use zero to allow for any number of comms loss events.

## AFS_GEOFENCE: Enable geofence Advanced Failsafe

*Note: This parameter is for advanced users*

This enables the geofence part of the AFS. Will only be in effect if AFS_ENABLE is also 1

## AFS_RC: Enable RC Advanced Failsafe

*Note: This parameter is for advanced users*

This enables the RC part of the AFS. Will only be in effect if AFS_ENABLE is also 1

## AFS_RC_MAN_ONLY: Enable RC Termination only in manual control modes

*Note: This parameter is for advanced users*

If this parameter is set to 1, then an RC loss will only cause the plane to terminate in manual control modes. If it is 0, then the plane will terminate in any flight mode.

## AFS_DUAL_LOSS: Enable dual loss terminate due to failure of both GCS and GPS simultaneously

*Note: This parameter is for advanced users*

This enables the dual loss termination part of the AFS system. If this parameter is 1 and both GPS and the ground control station fail simultaneously, this will be considered a "dual loss" and cause termination.

## AFS_RC_FAIL_TIME: RC failure time

*Note: This parameter is for advanced users*

This is the time in seconds in manual mode that failsafe termination will activate if RC input is lost. For the OBC rules this should be (1.5). Use 0 to disable.

- Units: s

## AFS_MAX_RANGE: Max allowed range

*Note: This parameter is for advanced users*

This is the maximum range of the vehicle in kilometers from first arming. If the vehicle goes beyond this range then the TERM_ACTION is performed. A value of zero disables this feature.

- Units: km

# AHRS Parameters

## AHRS_GPS_GAIN: AHRS GPS gain

*Note: This parameter is for advanced users*

This controls how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.

- Range: 0.0 1.0

- Increment: .01

## AHRS_GPS_USE: AHRS use GPS for DCM navigation and position-down

*Note: This parameter is for advanced users*

This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight. Currently this affects only the DCM-based AHRS: the EKF uses GPS according to its own parameters. A value of 2 means to use GPS for height as well as position - both in DCM estimation and when determining altitude-above-home.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Use GPS for DCM position|
|2|Use GPS for DCM position and height|

## AHRS_YAW_P: Yaw P

*Note: This parameter is for advanced users*

This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.

- Range: 0.1 0.4

- Increment: .01

## AHRS_RP_P: AHRS RP_P

*Note: This parameter is for advanced users*

This controls how fast the accelerometers correct the attitude

- Range: 0.1 0.4

- Increment: .01

## AHRS_WIND_MAX: Maximum wind

*Note: This parameter is for advanced users*

This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is. See ARSPD_OPTIONS and ARSPD_MAX_WIND to disable airspeed sensors.

- Range: 0 127

- Units: m/s

- Increment: 1

## AHRS_TRIM_X: AHRS Trim Roll

Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.

- Units: rad

- Range: -0.1745 +0.1745

- Increment: 0.01

## AHRS_TRIM_Y: AHRS Trim Pitch

Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.

- Units: rad

- Range: -0.1745 +0.1745

- Increment: 0.01

## AHRS_TRIM_Z: AHRS Trim Yaw

*Note: This parameter is for advanced users*

Not Used

- Units: rad

- Range: -0.1745 +0.1745

- Increment: 0.01

## AHRS_ORIENTATION: Board Orientation

*Note: This parameter is for advanced users*

Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. The label for each option is specified in the order of rotations for that orientation. This option takes affect on next boot. After changing you will need to re-level your vehicle. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the AHRS_CUSTOM_ROLL/PIT/YAW angles for AHRS orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_ROT1_ROLL/PIT/YAW or CUST_ROT2_ROLL/PIT/YAW angles.

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

## AHRS_COMP_BETA: AHRS Velocity Complementary Filter Beta Coefficient

*Note: This parameter is for advanced users*

This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.

- Range: 0.001 0.5

- Increment: .01

## AHRS_GPS_MINSATS: AHRS GPS Minimum satellites

*Note: This parameter is for advanced users*

Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.

- Range: 0 10

- Increment: 1

## AHRS_EKF_TYPE: Use NavEKF Kalman filter for attitude and position estimation

*Note: This parameter is for advanced users*

This controls which NavEKF Kalman filter version is used for attitude and position estimation

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|2|Enable EKF2|
|3|Enable EKF3|
|11|ExternalAHRS|

## AHRS_CUSTOM_ROLL: Board orientation roll offset

*Note: This parameter is for advanced users*

Autopilot mounting position roll offset. Positive values = roll right, negative values = roll left. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.

- Range: -180 180

- Units: deg

- Increment: 1

## AHRS_CUSTOM_PIT: Board orientation pitch offset

*Note: This parameter is for advanced users*

Autopilot mounting position pitch offset. Positive values = pitch up, negative values = pitch down. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.

- Range: -180 180

- Units: deg

- Increment: 1

## AHRS_CUSTOM_YAW: Board orientation yaw offset

*Note: This parameter is for advanced users*

Autopilot mounting position yaw offset. Positive values = yaw right, negative values = yaw left. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.

- Range: -180 180

- Units: deg

- Increment: 1

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

# ARMING Parameters

## ARMING_ACCTHRESH: Accelerometer error threshold

*Note: This parameter is for advanced users*

Accelerometer error threshold used to determine inconsistent accelerometers. Compares this error range to other accelerometers to detect a hardware or calibration error. Lower value means tighter check and harder to pass arming check. Not all accelerometers are created equal.

- Units: m/s/s

- Range: 0.25 3.0

## ARMING_RUDDER: Arming with Rudder enable/disable

*Note: This parameter is for advanced users*

Allow arm/disarm by rudder input. When enabled arming can be done with right rudder, disarming with left rudder. Rudder arming only works in manual throttle modes with throttle at zero +- deadzone (RCx_DZ)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|ArmingOnly|
|2|ArmOrDisarm|

## ARMING_MIS_ITEMS: Required mission items

*Note: This parameter is for advanced users*

Bitmask of mission items that are required to be planned in order to arm the aircraft

- Bitmask: 0:Land,1:VTOL Land,2:DO_LAND_START,3:Takeoff,4:VTOL Takeoff,5:Rallypoint

## ARMING_CHECK: Arm Checks to Perform (bitmask)

Checks prior to arming motor. This is a bitmask of checks that will be performed before allowing arming. For most users it is recommended to leave this at the default of 1 (all checks enabled). You can select whatever checks you prefer by adding together the values of each check type to set this parameter. For example, to only allow arming when you have GPS lock and no RC failsafe you would set ARMING_CHECK to 72.

- Bitmask: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC Channels,7:Board voltage,8:Battery Level,10:Logging Available,11:Hardware safety switch,12:GPS Configuration,13:System,14:Mission,15:Rangefinder,16:Camera,17:AuxAuth,18:VisualOdometry,19:FFT

## ARMING_OPTIONS: Arming options

*Note: This parameter is for advanced users*

Options that can be applied to change arming behaviour

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Disable prearm display|

# AROT Parameters

## AROT_ENABLE: Enable settings for RSC Setpoint

*Note: This parameter is for advanced users*

Allows you to enable (1) or disable (0) the autonomous autorotation capability.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## AROT_HS_P: P gain for head speed controller

*Note: This parameter is for advanced users*

Increase value to increase sensitivity of head speed controller during autonomous autorotation.

- Range: 0.3 1

- Increment: 0.01

## AROT_HS_SET_PT: Target Head Speed

*Note: This parameter is for advanced users*

The target head speed in RPM during autorotation.  Start by setting to desired hover speed and tune from there as necessary.

- Units: RPM

- Range: 1000 2800

- Increment: 1

## AROT_TARG_SP: Target Glide Ground Speed

*Note: This parameter is for advanced users*

Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.

- Units: cm/s

- Range: 800 2000

- Increment: 50

## AROT_COL_FILT_E: Entry Phase Collective Filter

*Note: This parameter is for advanced users*

Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than AROT_COL_FILT_G.

- Units: Hz

- Range: 0.2 0.5

- Increment: 0.01

## AROT_COL_FILT_G: Glide Phase Collective Filter

*Note: This parameter is for advanced users*

Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than AROT_COL_FILT_E.

- Units: Hz

- Range: 0.03 0.15

- Increment: 0.01

## AROT_AS_ACC_MAX: Forward Acceleration Limit

*Note: This parameter is for advanced users*

Maximum forward acceleration to apply in speed controller.

- Units: cm/s/s

- Range: 30 60

- Increment: 10

## AROT_BAIL_TIME: Bail Out Timer

*Note: This parameter is for advanced users*

Time in seconds from bail out initiated to the exit of autorotation flight mode.

- Units: s

- Range: 0.5 4

- Increment: 0.1

## AROT_HS_SENSOR: Main Rotor RPM Sensor 

*Note: This parameter is for advanced users*

Allocate the RPM sensor instance to use for measuring head speed.  RPM1 = 0.  RPM2 = 1.

- Units: s

- Range: 0.5 3

- Increment: 0.1

## AROT_FW_V_P: Velocity (horizontal) P gain

*Note: This parameter is for advanced users*

Velocity (horizontal) P gain.  Determines the propotion of the target acceleration based on the velocity error.

- Range: 0.1 6.0

- Increment: 0.1

## AROT_FW_V_FF: Velocity (horizontal) feed forward

*Note: This parameter is for advanced users*

Velocity (horizontal) input filter.  Corrects the target acceleration proportionally to the desired velocity.

- Range: 0 1

- Increment: 0.01

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

## ARSPD_DEVID: Airspeed ID

*Note: This parameter is for advanced users*

Airspeed sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## ARSPD_USE: Airspeed use

This parameter is not used by this vehicle. Always set to 0.

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

## ARSPD_AUTOCAL: This parameter and function is not used by this vehicle. Always set to 0.

*Note: This parameter is for advanced users*

Enables automatic adjustment of ARSPD_RATIO during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD_TUBE_ORDER: Control pitot tube order

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

Bus number of the I2C bus where the airspeed sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|Bus0(internal)|
|1|Bus1(external)|
|2|Bus2(auxiliary)|

## ARSPD_PRIMARY: Primary airspeed sensor

*Note: This parameter is for advanced users*

This selects which airspeed sensor will be the primary if multiple sensors are found

|Value|Meaning|
|:---:|:---:|
|0|FirstSensor|
|1|2ndSensor|

## ARSPD_OPTIONS: Airspeed options bitmask

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle. Always set to 0.

- Bitmask: 0:SpeedMismatchDisable, 1:AllowSpeedMismatchRecovery, 2:DisableVoltageCorrection, 3:UseEkf3Consistency

## ARSPD_WIND_MAX: Maximum airspeed and ground speed difference

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle. Always set to 0.

- Units: m/s

## ARSPD_WIND_WARN: Airspeed and ground speed difference that gives a warning

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle. Always set to 0.

- Units: m/s

## ARSPD_WIND_GATE: Re-enable Consistency Check Gate Size

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle.

- Range: 0.0 10.0

## ARSPD2_TYPE: Second Airspeed type

Type of 2nd airspeed sensor

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

## ARSPD2_USE: Enable use of 2nd airspeed sensor

This parameter and function is not used by this vehicle. Always set to 0.

|Value|Meaning|
|:---:|:---:|
|0|Don't Use|
|1|use|
|2|UseWhenZeroThrottle|

## ARSPD2_OFFSET: Airspeed offset for 2nd airspeed sensor

*Note: This parameter is for advanced users*

Airspeed calibration offset

- Increment: 0.1

## ARSPD2_RATIO: Airspeed ratio for 2nd airspeed sensor

*Note: This parameter is for advanced users*

Airspeed calibration ratio

- Increment: 0.1

## ARSPD2_PIN: Airspeed pin for 2nd airspeed sensor

*Note: This parameter is for advanced users*

Pin number indicating location of analog airspeed sensors. Pixhawk/Cube if set to 15. 

## ARSPD2_AUTOCAL: Automatic airspeed ratio calibration for 2nd airspeed sensor

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle. Always set to 0.

## ARSPD2_TUBE_ORDR: Control pitot tube order of 2nd airspeed sensor

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD2_SKIP_CAL: Skip airspeed offset calibration on startup for 2nd sensor

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## ARSPD2_PSI_RANGE: The PSI range of the device for 2nd sensor

*Note: This parameter is for advanced users*

This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD2_BUS: Airspeed I2C bus for 2nd sensor

*Note: This parameter is for advanced users*

The bus number of the I2C bus to look for the sensor on

|Value|Meaning|
|:---:|:---:|
|0|Bus0(internal)|
|1|Bus1(external)|
|2|Bus2(auxiliary)|

## ARSPD2_DEVID: Airspeed2 ID

*Note: This parameter is for advanced users*

Airspeed2 sensor ID, taking into account its type, bus and instance

- ReadOnly: True

# ATC Parameters

## ATC_SLEW_YAW: Yaw target slew rate

*Note: This parameter is for advanced users*

Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes

- Units: cdeg/s

- Range: 500 18000

- Increment: 100

## ATC_ACCEL_Y_MAX: Acceleration Max for Yaw

*Note: This parameter is for advanced users*

Maximum acceleration in yaw axis

- Units: cdeg/s/s

- Range: 0 72000

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|9000|VerySlow|
|18000|Slow|
|36000|Medium|
|54000|Fast|

- Increment: 1000

## ATC_RATE_FF_ENAB: Rate Feedforward Enable

*Note: This parameter is for advanced users*

Controls whether body-frame rate feedfoward is enabled or disabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ATC_ACCEL_R_MAX: Acceleration Max for Roll

*Note: This parameter is for advanced users*

Maximum acceleration in roll axis

- Units: cdeg/s/s

- Range: 0 180000

- Increment: 1000

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|30000|VerySlow|
|72000|Slow|
|108000|Medium|
|162000|Fast|

## ATC_ACCEL_P_MAX: Acceleration Max for Pitch

*Note: This parameter is for advanced users*

Maximum acceleration in pitch axis

- Units: cdeg/s/s

- Range: 0 180000

- Increment: 1000

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|30000|VerySlow|
|72000|Slow|
|108000|Medium|
|162000|Fast|

## ATC_ANGLE_BOOST: Angle Boost

*Note: This parameter is for advanced users*

Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ATC_ANG_RLL_P: Roll axis angle controller P gain

Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate

- Range: 3.000 12.000

## ATC_ANG_PIT_P: Pitch axis angle controller P gain

Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate

- Range: 3.000 12.000

## ATC_ANG_YAW_P: Yaw axis angle controller P gain

Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate

- Range: 3.000 12.000

## ATC_ANG_LIM_TC: Angle Limit (to maintain altitude) Time Constant

*Note: This parameter is for advanced users*

Angle Limit (to maintain altitude) Time Constant

- Range: 0.5 10.0

## ATC_RATE_R_MAX: Angular Velocity Max for Roll

*Note: This parameter is for advanced users*

Maximum angular velocity in roll axis

- Units: deg/s

- Range: 0 1080

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|60|Slow|
|180|Medium|
|360|Fast|

## ATC_RATE_P_MAX: Angular Velocity Max for Pitch

*Note: This parameter is for advanced users*

Maximum angular velocity in pitch axis

- Units: deg/s

- Range: 0 1080

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|60|Slow|
|180|Medium|
|360|Fast|

## ATC_RATE_Y_MAX: Angular Velocity Max for Yaw

*Note: This parameter is for advanced users*

Maximum angular velocity in yaw axis

- Units: deg/s

- Range: 0 1080

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|60|Slow|
|180|Medium|
|360|Fast|

## ATC_INPUT_TC: Attitude control input time constant

Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response

- Units: s

- Range: 0 1

- Increment: 0.01

|Value|Meaning|
|:---:|:---:|
|0.5|Very Soft|
|0.2|Soft|
|0.15|Medium|
|0.1|Crisp|
|0.05|Very Crisp|

## ATC_RAT_RLL_P: Roll axis rate controller P gain

Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output

- Range: 0.01 0.5

- Increment: 0.005

## ATC_RAT_RLL_I: Roll axis rate controller I gain

Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate

- Range: 0.01 2.0

- Increment: 0.01

## ATC_RAT_RLL_IMAX: Roll axis rate controller I gain maximum

Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_RLL_D: Roll axis rate controller D gain

Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate

- Range: 0.0 0.05

- Increment: 0.001

## ATC_RAT_RLL_FF: Roll axis rate controller feed forward

Roll axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## ATC_RAT_RLL_FLTT: Roll axis rate controller target frequency in Hz

Roll axis rate controller target frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_FLTE: Roll axis rate controller error frequency in Hz

Roll axis rate controller error frequency in Hz

- Range: 0 100

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_FLTD: Roll axis rate controller derivative frequency in Hz

Roll axis rate controller derivative frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_SMAX: Roll slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_RAT_PIT_P: Pitch axis rate controller P gain

Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output

- Range: 0.01 0.50

- Increment: 0.005

## ATC_RAT_PIT_I: Pitch axis rate controller I gain

Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate

- Range: 0.01 2.0

- Increment: 0.01

## ATC_RAT_PIT_IMAX: Pitch axis rate controller I gain maximum

Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_PIT_D: Pitch axis rate controller D gain

Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate

- Range: 0.0 0.05

- Increment: 0.001

## ATC_RAT_PIT_FF: Pitch axis rate controller feed forward

Pitch axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## ATC_RAT_PIT_FLTT: Pitch axis rate controller target frequency in Hz

Pitch axis rate controller target frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_FLTE: Pitch axis rate controller error frequency in Hz

Pitch axis rate controller error frequency in Hz

- Range: 0 100

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_FLTD: Pitch axis rate controller derivative frequency in Hz

Pitch axis rate controller derivative frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_SMAX: Pitch slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_RAT_YAW_P: Yaw axis rate controller P gain

Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output

- Range: 0.10 2.50

- Increment: 0.005

## ATC_RAT_YAW_I: Yaw axis rate controller I gain

Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate

- Range: 0.010 1.0

- Increment: 0.01

## ATC_RAT_YAW_IMAX: Yaw axis rate controller I gain maximum

Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_YAW_D: Yaw axis rate controller D gain

Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate

- Range: 0.000 0.02

- Increment: 0.001

## ATC_RAT_YAW_FF: Yaw axis rate controller feed forward

Yaw axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## ATC_RAT_YAW_FLTT: Yaw axis rate controller target frequency in Hz

Yaw axis rate controller target frequency in Hz

- Range: 1 50

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_FLTE: Yaw axis rate controller error frequency in Hz

Yaw axis rate controller error frequency in Hz

- Range: 0 20

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_FLTD: Yaw axis rate controller derivative frequency in Hz

Yaw axis rate controller derivative frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_SMAX: Yaw slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_THR_MIX_MIN: Throttle Mix Minimum

*Note: This parameter is for advanced users*

Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)

- Range: 0.1 0.25

## ATC_THR_MIX_MAX: Throttle Mix Maximum

*Note: This parameter is for advanced users*

Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

- Range: 0.5 0.9

## ATC_THR_MIX_MAN: Throttle Mix Manual

*Note: This parameter is for advanced users*

Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)

- Range: 0.1 0.9

## ATC_HOVR_ROL_TRM: Hover Roll Trim

*Note: This parameter is for advanced users*

Trim the hover roll angle to counter tail rotor thrust in a hover

- Units: cdeg

- Increment: 10

- Range: 0 1000

## ATC_RAT_RLL_P: Roll axis rate controller P gain

Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output

- Range: 0.0 0.35

- Increment: 0.005

## ATC_RAT_RLL_I: Roll axis rate controller I gain

Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate

- Range: 0.0 0.6

- Increment: 0.01

## ATC_RAT_RLL_IMAX: Roll axis rate controller I gain maximum

Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_RLL_ILMI: Roll axis rate controller I-term leak minimum

*Note: This parameter is for advanced users*

Point below which I-term will not leak down

- Range: 0 1

## ATC_RAT_RLL_D: Roll axis rate controller D gain

Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate

- Range: 0.0 0.03

- Increment: 0.001

## ATC_RAT_RLL_FF: Roll axis rate controller feed forward

Roll axis rate controller feed forward

- Range: 0.05 0.5

- Increment: 0.001

## ATC_RAT_RLL_FLTT: Roll axis rate controller target frequency in Hz

Roll axis rate controller target frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_FLTE: Roll axis rate controller error frequency in Hz

Roll axis rate controller error frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_FLTD: Roll axis rate controller derivative frequency in Hz

Roll axis rate controller derivative frequency in Hz

- Range: 0 50

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_SMAX: Roll slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_RAT_PIT_P: Pitch axis rate controller P gain

Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output

- Range: 0.0 0.35

- Increment: 0.005

## ATC_RAT_PIT_I: Pitch axis rate controller I gain

Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate

- Range: 0.0 0.6

- Increment: 0.01

## ATC_RAT_PIT_IMAX: Pitch axis rate controller I gain maximum

Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_PIT_ILMI: Pitch axis rate controller I-term leak minimum

*Note: This parameter is for advanced users*

Point below which I-term will not leak down

- Range: 0 1

## ATC_RAT_PIT_D: Pitch axis rate controller D gain

Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate

- Range: 0.0 0.03

- Increment: 0.001

## ATC_RAT_PIT_FF: Pitch axis rate controller feed forward

Pitch axis rate controller feed forward

- Range: 0.05 0.5

- Increment: 0.001

## ATC_RAT_PIT_FLTT: Pitch axis rate controller target frequency in Hz

Pitch axis rate controller target frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_FLTE: Pitch axis rate controller error frequency in Hz

Pitch axis rate controller error frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_FLTD: Pitch axis rate controller derivative frequency in Hz

Pitch axis rate controller derivative frequency in Hz

- Range: 0 50

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_SMAX: Pitch slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_RAT_YAW_P: Yaw axis rate controller P gain

Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output

- Range: 0.180 0.60

- Increment: 0.005

## ATC_RAT_YAW_I: Yaw axis rate controller I gain

Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate

- Range: 0.01 0.2

- Increment: 0.01

## ATC_RAT_YAW_IMAX: Yaw axis rate controller I gain maximum

Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_YAW_ILMI: Yaw axis rate controller I-term leak minimum

*Note: This parameter is for advanced users*

Point below which I-term will not leak down

- Range: 0 1

## ATC_RAT_YAW_D: Yaw axis rate controller D gain

Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate

- Range: 0.000 0.02

- Increment: 0.001

## ATC_RAT_YAW_FF: Yaw axis rate controller feed forward

Yaw axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## ATC_RAT_YAW_FLTT: Yaw axis rate controller target frequency in Hz

Yaw axis rate controller target frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_FLTE: Yaw axis rate controller error frequency in Hz

Yaw axis rate controller error frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_FLTD: Yaw axis rate controller derivative frequency in Hz

Yaw axis rate controller derivative frequency in Hz

- Range: 0 50

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_SMAX: Yaw slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_PIRO_COMP: Piro Comp Enable

*Note: This parameter is for advanced users*

Pirouette compensation enabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

# AUTOTUNE Parameters

## AUTOTUNE_AXES: Autotune axis bitmask

1-byte bitmap of axes to autotune

- Bitmask: 0:Roll,1:Pitch,2:Yaw

## AUTOTUNE_AGGR: Autotune aggressiveness

Autotune aggressiveness. Defines the bounce back used to detect size of the D term.

- Range: 0.05 0.10

## AUTOTUNE_MIN_D: AutoTune minimum D

Defines the minimum D gain

- Range: 0.001 0.006

## AUTOTUNE_AXES: Autotune axis bitmask

1-byte bitmap of axes to autotune

- Bitmask: 0:Roll,1:Pitch,2:Yaw

## AUTOTUNE_SEQ: AutoTune Sequence Bitmask

2-byte bitmask to select what tuning should be performed.  Max gain automatically performed if Rate D is selected. Values: 7:All,1:VFF Only,2:Rate D/Rate P Only(incl max gain),4:Angle P Only,8:Max Gain Only,3:VFF and Rate D/Rate P(incl max gain),5:VFF and Angle P,6:Rate D/Rate P(incl max gain) and angle P

- Bitmask: 0:VFF,1:Rate D/Rate P(incl max gain),2:Angle P,3:Max Gain Only

## AUTOTUNE_FRQ_MIN: AutoTune minimum sweep frequency

Defines the start frequency for sweeps and dwells

- Range: 10 30

## AUTOTUNE_FRQ_MAX: AutoTune maximum sweep frequency

Defines the end frequency for sweeps and dwells

- Range: 50 120

## AUTOTUNE_GN_MAX: AutoTune maximum response gain

Defines the response gain (output/input) to tune

- Range: 1 2.5

## AUTOTUNE_VELXY_P: AutoTune velocity xy P gain

Velocity xy P gain used to hold position during Max Gain, Rate P, and Rate D frequency sweeps

- Range: 0 1

# AVD Parameters

## AVD_ENABLE: Enable Avoidance using ADSB

*Note: This parameter is for advanced users*

Enable Avoidance using ADSB

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## AVD_F_ACTION: Collision Avoidance Behavior

*Note: This parameter is for advanced users*

Specifies aircraft behaviour when a collision is imminent

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Report|
|2|Climb Or Descend|
|3|Move Horizontally|
|4|Move Perpendicularly in 3D|
|5|RTL|
|6|Hover|

## AVD_W_ACTION: Collision Avoidance Behavior - Warn

*Note: This parameter is for advanced users*

Specifies aircraft behaviour when a collision may occur

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Report|

## AVD_F_RCVRY: Recovery behaviour after a fail event

*Note: This parameter is for advanced users*

Determines what the aircraft will do after a fail event is resolved

|Value|Meaning|
|:---:|:---:|
|0|Remain in AVOID_ADSB|
|1|Resume previous flight mode|
|2|RTL|
|3|Resume if AUTO else Loiter|

## AVD_OBS_MAX: Maximum number of obstacles to track

*Note: This parameter is for advanced users*

Maximum number of obstacles to track

## AVD_W_TIME: Time Horizon Warn

*Note: This parameter is for advanced users*

Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than W_DIST_XY or W_DIST_Z then W_ACTION is undertaken (assuming F_ACTION is not undertaken)

- Units: s

## AVD_F_TIME: Time Horizon Fail

*Note: This parameter is for advanced users*

Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than F_DIST_XY or F_DIST_Z then F_ACTION is undertaken

- Units: s

## AVD_W_DIST_XY: Distance Warn XY

*Note: This parameter is for advanced users*

Closest allowed projected distance before W_ACTION is undertaken

- Units: m

## AVD_F_DIST_XY: Distance Fail XY

*Note: This parameter is for advanced users*

Closest allowed projected distance before F_ACTION is undertaken

- Units: m

## AVD_W_DIST_Z: Distance Warn Z

*Note: This parameter is for advanced users*

Closest allowed projected distance before BEHAVIOUR_W is undertaken

- Units: m

## AVD_F_DIST_Z: Distance Fail Z

*Note: This parameter is for advanced users*

Closest allowed projected distance before BEHAVIOUR_F is undertaken

- Units: m

## AVD_F_ALT_MIN: ADS-B avoidance minimum altitude

*Note: This parameter is for advanced users*

Minimum AMSL (above mean sea level) altitude for ADS-B avoidance. If the vehicle is below this altitude, no avoidance action will take place. Useful to prevent ADS-B avoidance from activating while below the tree line or around structures. Default of 0 is no minimum.

- Units: m

# AVOID Parameters

## AVOID_ENABLE: Avoidance control enable/disable

Enabled/disable avoidance input sources

- Bitmask: 0:UseFence,1:UseProximitySensor,2:UseBeaconFence

## AVOID_ANGLE_MAX: Avoidance max lean angle in non-GPS flight modes

Max lean angle used to avoid obstacles while in non-GPS modes

- Units: cdeg

- Increment: 10

- Range: 0 4500

## AVOID_DIST_MAX: Avoidance distance maximum in non-GPS flight modes

Distance from object at which obstacle avoidance will begin in non-GPS modes

- Units: m

- Range: 1 30

## AVOID_MARGIN: Avoidance distance margin in GPS modes

Vehicle will attempt to stay at least this distance (in meters) from objects while in GPS modes

- Units: m

- Range: 1 10

## AVOID_BEHAVE: Avoidance behaviour

Avoidance behaviour (slide or stop)

|Value|Meaning|
|:---:|:---:|
|0|Slide|
|1|Stop|

## AVOID_BACKUP_SPD: Avoidance maximum backup speed

Maximum speed that will be used to back away from obstacles in GPS modes (m/s). Set zero to disable

- Units: m/s

- Range: 0 2

## AVOID_ALT_MIN: Avoidance minimum altitude

Minimum altitude above which proximity based avoidance will start working. This requires a valid downward facing rangefinder reading to work. Set zero to disable

- Units: m

- Range: 0 6

## AVOID_ACCEL_MAX: Avoidance maximum acceleration

Maximum acceleration with which obstacles will be avoided with. Set zero to disable acceleration limits

- Units: m/s/s

- Range: 0 9

## AVOID_BACKUP_DZ: Avoidance deadzone between stopping and backing away from obstacle

Distance beyond AVOID_MARGIN parameter, after which vehicle will backaway from obstacles. Increase this parameter if you see vehicle going back and forth in front of obstacle.

- Units: m

- Range: 0 2

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

This selects the bus number for looking for an I2C barometer. When set to -1 it will probe all external i2c buses based on the GND_PROBE_EXT parameter.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Bus0|
|1|Bus1|

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

This sets which types of external i2c barometer to look for. It is a bitmask of barometer types. The I2C buses to probe is based on GND_EXT_BUS. If BARO_EXT_BUS is -1 then it will probe all external buses, otherwise it will probe just the bus number given in GND_EXT_BUS.

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

User provided field elevation in meters. This is used to improve the calculation of the altitude the vehicle is at. This parameter is not persistent and will be reset to 0 every time the vehicle is rebooted. A value of 0 means no correction for takeoff height above sea level is performed.

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

## BATT2_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT2_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT2_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT2_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT3_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT3_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT3_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT3_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT4_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT4_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT4_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT4_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT5_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT5_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT5_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT5_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT6_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT6_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT6_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT6_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT7_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT7_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT7_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT7_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT8_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT8_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT8_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT8_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT9_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT9_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT9_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT9_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

## BATT_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

## BATT_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|
|2|RTL|
|3|SmartRTL or RTL|
|4|SmartRTL or Land|
|5|Terminate|
|6|Auto DO_LAND_START or RTL|

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

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATT_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

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

# BCN Parameters

## BCN_TYPE: Beacon based position estimation device type

*Note: This parameter is for advanced users*

What type of beacon based position estimation device is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Pozyx|
|2|Marvelmind|
|3|Nooploop|
|10|SITL|

## BCN_LATITUDE: Beacon origin's latitude

*Note: This parameter is for advanced users*

Beacon origin's latitude

- Units: deg

- Increment: 0.000001

- Range: -90 90

## BCN_LONGITUDE: Beacon origin's longitude

*Note: This parameter is for advanced users*

Beacon origin's longitude

- Units: deg

- Increment: 0.000001

- Range: -180 180

## BCN_ALT: Beacon origin's altitude above sealevel in meters

*Note: This parameter is for advanced users*

Beacon origin's altitude above sealevel in meters

- Units: m

- Increment: 1

- Range: 0 10000

## BCN_ORIENT_YAW: Beacon systems rotation from north in degrees

*Note: This parameter is for advanced users*

Beacon systems rotation from north in degrees

- Units: deg

- Increment: 1

- Range: -180 +180

# BRD Parameters

## BRD_SER1_RTSCTS: Serial 1 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 1 (telemetry 1). You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|

- RebootRequired: True

## BRD_SER2_RTSCTS: Serial 2 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 2 (telemetry 2). You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|

- RebootRequired: True

## BRD_SER3_RTSCTS: Serial 3 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 3. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|

- RebootRequired: True

## BRD_SER4_RTSCTS: Serial 4 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 4. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|

- RebootRequired: True

## BRD_SER5_RTSCTS: Serial 5 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 5. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|

- RebootRequired: True

## BRD_SAFETYENABLE: Enable use of safety arming switch

This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## BRD_SBUS_OUT:  SBUS output rate

*Note: This parameter is for advanced users*

This sets the SBUS output frame rate in Hz

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|50Hz|
|2|75Hz|
|3|100Hz|
|4|150Hz|
|5|200Hz|
|6|250Hz|
|7|300Hz|

- RebootRequired: True

## BRD_SERIAL_NUM: User-defined serial number

User-defined serial number of this vehicle, it can be any arbitrary number you want and has no effect on the autopilot

- Range: -8388608 8388607

## BRD_SAFETY_MASK: Outputs which ignore the safety switch state

*Note: This parameter is for advanced users*

A bitmask which controls what outputs can move while the safety switch has not been pressed

- Bitmask: 0:Output1,1:Output2,2:Output3,3:Output4,4:Output5,5:Output6,6:Output7,7:Output8,8:Output9,9:Output10,10:Output11,11:Output12,12:Output13,13:Output14

- RebootRequired: True

## BRD_HEAT_TARG: Board heater temperature target

*Note: This parameter is for advanced users*

Board heater target temperature for boards with controllable heating units. DO NOT SET to -1 on the Cube. Set to -1 to disable the heater, please reboot after setting to -1.

- Range: -1 80

- Units: degC

## BRD_TYPE: Board type

*Note: This parameter is for advanced users*

This allows selection of a PX4 or VRBRAIN board type. If set to zero then the board type is auto-detected (PX4)

|Value|Meaning|
|:---:|:---:|
|0|AUTO|
|1|PX4V1|
|2|Pixhawk|
|3|Cube/Pixhawk2|
|4|Pixracer|
|5|PixhawkMini|
|6|Pixhawk2Slim|
|13|Intel Aero FC|
|14|Pixhawk Pro|
|20|AUAV2.1|
|21|PCNC1|
|22|MINDPXV2|
|23|SP01|
|24|CUAVv5/FMUV5|
|30|VRX BRAIN51|
|32|VRX BRAIN52|
|33|VRX BRAIN52E|
|34|VRX UBRAIN51|
|35|VRX UBRAIN52|
|36|VRX CORE10|
|38|VRX BRAIN54|
|39|PX4 FMUV6|
|100|PX4 OLDDRIVERS|

- RebootRequired: True

## BRD_IO_ENABLE: Enable IO co-processor

*Note: This parameter is for advanced users*

This allows for the IO co-processor on boards with an IOMCU to be disabled. Setting to 2 will enable the IOMCU but not attempt to update firmware on startup

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|EnableNoFWUpdate|

- RebootRequired: True

## BRD_SAFETYOPTION: Options for safety button behavior

This controls the activation of the safety button. It allows you to control if the safety button can be used for safety enable and/or disable, and whether the button is only active when disarmed

- Bitmask: 0:ActiveForSafetyDisable,1:ActiveForSafetyEnable,2:ActiveWhenArmed,3:Force safety on when the aircraft disarms

## BRD_VBUS_MIN: Autopilot board voltage requirement

*Note: This parameter is for advanced users*

Minimum voltage on the autopilot power rail to allow the aircraft to arm. 0 to disable the check.

- Units: V

- Range: 4.0 5.5

- Increment: 0.1

## BRD_VSERVO_MIN: Servo voltage requirement

*Note: This parameter is for advanced users*

Minimum voltage on the servo rail to allow the aircraft to arm. 0 to disable the check.

- Units: V

- Range: 3.3 12.0

- Increment: 0.1

## BRD_SD_SLOWDOWN: microSD slowdown

*Note: This parameter is for advanced users*

This is a scaling factor to slow down microSD operation. It can be used on flight board and microSD card combinations where full speed is not reliable. For normal full speed operation a value of 0 should be used.

- Range: 0 32

- Increment: 1

## BRD_PWM_VOLT_SEL: Set PWM Out Voltage

*Note: This parameter is for advanced users*

This sets the voltage max for PWM output pulses. 0 for 3.3V and 1 for 5V output.

|Value|Meaning|
|:---:|:---:|
|0|3.3V|
|1|5V|

## BRD_OPTIONS: Board options

*Note: This parameter is for advanced users*

Board specific option flags

- Bitmask: 0:Enable hardware watchdog, 1:Disable MAVftp, 2:Enable set of internal parameters, 3:Enable Debug Pins, 4:Unlock flash on reboot, 5:Write protect firmware flash on reboot, 6:Write protect bootloader flash on reboot

## BRD_BOOT_DELAY: Boot delay

*Note: This parameter is for advanced users*

This adds a delay in milliseconds to boot to ensure peripherals initialise fully

- Range: 0 10000

- Units: ms

## BRD_HEAT_P: Board Heater P gain

*Note: This parameter is for advanced users*

Board Heater P gain

- Range: 1 500

- Increment: 1

## BRD_HEAT_I: Board Heater I gain

*Note: This parameter is for advanced users*

Board Heater integrator gain

- Range: 0 1

- Increment: 0.1

## BRD_HEAT_IMAX: Board Heater IMAX

*Note: This parameter is for advanced users*

Board Heater integrator maximum

- Range: 0 100

- Increment: 1

## BRD_ALT_CONFIG: Alternative HW config

*Note: This parameter is for advanced users*

Select an alternative hardware configuration. A value of zero selects the default configuration for this board. Other values are board specific. Please see the documentation for your board for details on any alternative configuration values that may be available.

- Range: 0 10

- Increment: 1

- RebootRequired: True

## BRD_HEAT_LOWMGN: Board heater temp lower margin

*Note: This parameter is for advanced users*

Arming check will fail if temp is lower than this margin below BRD_HEAT_TARG. 0 disables the low temperature check

- Range: 0 20

- Units: degC

# BRDRADIO Parameters

## BRD_RADIO_TYPE: Set type of direct attached radio

This enables support for direct attached radio receivers

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|CYRF6936|
|2|CC2500|
|3|BK2425|

## BRD_RADIO_PROT: protocol

*Note: This parameter is for advanced users*

Select air protocol

|Value|Meaning|
|:---:|:---:|
|0|Auto|
|1|DSM2|
|2|DSMX|

## BRD_RADIO_DEBUG: debug level

*Note: This parameter is for advanced users*

radio debug level

- Range: 0 4

## BRD_RADIO_DISCRC: disable receive CRC

*Note: This parameter is for advanced users*

disable receive CRC (for debug)

|Value|Meaning|
|:---:|:---:|
|0|NotDisabled|
|1|Disabled|

## BRD_RADIO_SIGCH: RSSI signal strength

*Note: This parameter is for advanced users*

Channel to show receive RSSI signal strength, or zero for disabled

- Range: 0 16

## BRD_RADIO_PPSCH: Packet rate channel

*Note: This parameter is for advanced users*

Channel to show received packet-per-second rate, or zero for disabled

- Range: 0 16

## BRD_RADIO_TELEM: Enable telemetry

*Note: This parameter is for advanced users*

If this is non-zero then telemetry packets will be sent over DSM

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BRD_RADIO_TXPOW: Telemetry Transmit power

*Note: This parameter is for advanced users*

Set telemetry transmit power. This is the power level (from 1 to 8) for telemetry packets sent from the RX to the TX

- Range: 1 8

## BRD_RADIO_FCCTST: Put radio into FCC test mode

*Note: This parameter is for advanced users*

If this is enabled then the radio will continuously transmit as required for FCC testing. The transmit channel is set by the value of the parameter. The radio will not work for RC input while this is enabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|MinChannel|
|2|MidChannel|
|3|MaxChannel|
|4|MinChannelCW|
|5|MidChannelCW|
|6|MaxChannelCW|

## BRD_RADIO_STKMD: Stick input mode

*Note: This parameter is for advanced users*

This selects between different stick input modes. The default is mode2, which has throttle on the left stick and pitch on the right stick. You can instead set mode1, which has throttle on the right stick and pitch on the left stick.

|Value|Meaning|
|:---:|:---:|
|1|Mode1|
|2|Mode2|

## BRD_RADIO_TESTCH: Set radio to factory test channel

*Note: This parameter is for advanced users*

This sets the radio to a fixed test channel for factory testing. Using a fixed channel avoids the need for binding in factory testing.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TestChan1|
|2|TestChan2|
|3|TestChan3|
|4|TestChan4|
|5|TestChan5|
|6|TestChan6|
|7|TestChan7|
|8|TestChan8|

## BRD_RADIO_TSIGCH: RSSI value channel for telemetry data on transmitter

*Note: This parameter is for advanced users*

Channel to show telemetry RSSI value as received by TX

- Range: 0 16

## BRD_RADIO_TPPSCH: Telemetry PPS channel

*Note: This parameter is for advanced users*

Channel to show telemetry packets-per-second value, as received at TX

- Range: 0 16

## BRD_RADIO_TXMAX: Transmitter transmit power

*Note: This parameter is for advanced users*

Set transmitter maximum transmit power (from 1 to 8)

- Range: 1 8

## BRD_RADIO_BZOFS: Transmitter buzzer adjustment

*Note: This parameter is for advanced users*

Set transmitter buzzer note adjustment (adjust frequency up)

- Range: 0 40

## BRD_RADIO_ABTIME: Auto-bind time

*Note: This parameter is for advanced users*

When non-zero this sets the time with no transmitter packets before we start looking for auto-bind packets.

- Range: 0 120

## BRD_RADIO_ABLVL: Auto-bind level

*Note: This parameter is for advanced users*

This sets the minimum RSSI of an auto-bind packet for it to be accepted. This should be set so that auto-bind will only happen at short range to minimise the change of an auto-bind happening accidentially

- Range: 0 31

# BRDRTC Parameters

## BRD_RTC_TYPES: Allowed sources of RTC time

*Note: This parameter is for advanced users*

Specifies which sources of UTC time will be accepted

- Bitmask: 0:GPS,1:MAVLINK_SYSTEM_TIME,2:HW

## BRD_RTC_TZ_MIN: Timezone offset from UTC

*Note: This parameter is for advanced users*

Adds offset in +- minutes from UTC to calculate local time

- Range: -720 +840

# BTN Parameters

## BTN_ENABLE: Enable button reporting

*Note: This parameter is for advanced users*

This enables the button checking module. When this is disabled the parameters for setting button inputs are not visible

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BTN_PIN1: First button Pin

Digital pin number for first button input.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|

## BTN_PIN2: Second button Pin

Digital pin number for second button input.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|

## BTN_PIN3: Third button Pin

Digital pin number for third button input.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|

## BTN_PIN4: Fourth button Pin

Digital pin number for fourth button input. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|

## BTN_REPORT_SEND: Report send time

The duration in seconds that a BUTTON_CHANGE report is repeatedly sent to the GCS regarding a button changing state. Note that the BUTTON_CHANGE message is MAVLink2 only.

- Range: 0 3600

## BTN_OPTIONS1: Button Pin 1 Options

Options for Pin 1. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.

- Bitmask: 0:PWM Input,1:InvertInput

## BTN_OPTIONS2: Button Pin 2 Options

Options for Pin 2. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.

- Bitmask: 0:PWM Input,1:InvertInput

## BTN_OPTIONS3: Button Pin 3 Options

Options for Pin 3. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.

- Bitmask: 0:PWM Input,1:InvertInput

## BTN_OPTIONS4: Button Pin 4 Options

Options for Pin 4. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.

- Bitmask: 0:PWM Input,1:InvertInput

## BTN_FUNC1: Button Pin 1 RC Channel function

Auxiliary RC Options function executed on pin change

|Value|Meaning|
|:---:|:---:|
|0|Do Nothing|
|2|Flip|
|3|Simple Mode|
|4|RTL|
|5|Save Trim|
|7|Save WP|
|9|Camera Trigger|
|10|RangeFinder|
|11|Fence|
|13|Super Simple Mode|
|14|Acro Trainer|
|15|Sprayer|
|16|Auto|
|17|AutoTune|
|18|Land|
|19|Gripper|
|21|Parachute Enable|
|22|Parachute Release|
|23|Parachute 3pos|
|24|Auto Mission Reset|
|25|AttCon Feed Forward|
|26|AttCon Accel Limits|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Copter Sound|
|31|Motor Emergency Stop|
|32|Motor Interlock|
|33|Brake|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|37|Throw|
|38|ADSB Avoidance En|
|39|PrecLoiter|
|40|Proximity Avoidance|
|41|ArmDisarm (4.1 and lower)|
|42|SmartRTL|
|43|InvertedFlight|
|44|Winch Enable|
|46|RC Override Enable|
|47|User Function 1|
|48|User Function 2|
|49|User Function 3|
|52|Acro|
|55|Guided|
|56|Loiter|
|57|Follow|
|58|Clear Waypoints|
|60|ZigZag|
|61|ZigZag SaveWP|
|62|Compass Learn|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|68|Stabilize|
|69|PosHold|
|70|AltHold|
|71|FlowHold|
|72|Circle|
|73|Drift|
|75|SurfaceTrackingUpDown|
|76|Standby Mode|
|78|RunCam Control|
|79|RunCam OSD Control|
|80|VisOdom Align|
|81|Disarm|
|83|ZigZag Auto|
|84|Air Mode|
|85|Generator|
|90|EKF Pos Source|
|94|VTX Power|
|99|AUTO RTL|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|151|Turtle|
|152|simple heading reset|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with AirMode  (4.2 and higher)|
|158|Optflow Calibration|
|159|Force Flying|
|161|Turbine Start(heli)|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|212|Mount1 Roll|
|213|Mount1 Pitch|
|214|Mount1 Yaw|
|215|Mount2 Roll|
|216|Mount2 Pitch|
|217|Mount2 Yaw|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|

## BTN_FUNC2: Button Pin 2 RC Channel function

Auxiliary RC Options function executed on pin change

|Value|Meaning|
|:---:|:---:|
|0|Do Nothing|
|2|Flip|
|3|Simple Mode|
|4|RTL|
|5|Save Trim|
|7|Save WP|
|9|Camera Trigger|
|10|RangeFinder|
|11|Fence|
|13|Super Simple Mode|
|14|Acro Trainer|
|15|Sprayer|
|16|Auto|
|17|AutoTune|
|18|Land|
|19|Gripper|
|21|Parachute Enable|
|22|Parachute Release|
|23|Parachute 3pos|
|24|Auto Mission Reset|
|25|AttCon Feed Forward|
|26|AttCon Accel Limits|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Copter Sound|
|31|Motor Emergency Stop|
|32|Motor Interlock|
|33|Brake|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|37|Throw|
|38|ADSB Avoidance En|
|39|PrecLoiter|
|40|Proximity Avoidance|
|41|ArmDisarm (4.1 and lower)|
|42|SmartRTL|
|43|InvertedFlight|
|44|Winch Enable|
|46|RC Override Enable|
|47|User Function 1|
|48|User Function 2|
|49|User Function 3|
|52|Acro|
|55|Guided|
|56|Loiter|
|57|Follow|
|58|Clear Waypoints|
|60|ZigZag|
|61|ZigZag SaveWP|
|62|Compass Learn|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|68|Stabilize|
|69|PosHold|
|70|AltHold|
|71|FlowHold|
|72|Circle|
|73|Drift|
|75|SurfaceTrackingUpDown|
|76|Standby Mode|
|78|RunCam Control|
|79|RunCam OSD Control|
|80|VisOdom Align|
|81|Disarm|
|83|ZigZag Auto|
|84|Air Mode|
|85|Generator|
|90|EKF Pos Source|
|94|VTX Power|
|99|AUTO RTL|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|151|Turtle|
|152|simple heading reset|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with AirMode  (4.2 and higher)|
|158|Optflow Calibration|
|159|Force Flying|
|161|Turbine Start(heli)|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|212|Mount1 Roll|
|213|Mount1 Pitch|
|214|Mount1 Yaw|
|215|Mount2 Roll|
|216|Mount2 Pitch|
|217|Mount2 Yaw|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|

## BTN_FUNC3: Button Pin 3 RC Channel function

Auxiliary RC Options function executed on pin change

|Value|Meaning|
|:---:|:---:|
|0|Do Nothing|
|2|Flip|
|3|Simple Mode|
|4|RTL|
|5|Save Trim|
|7|Save WP|
|9|Camera Trigger|
|10|RangeFinder|
|11|Fence|
|13|Super Simple Mode|
|14|Acro Trainer|
|15|Sprayer|
|16|Auto|
|17|AutoTune|
|18|Land|
|19|Gripper|
|21|Parachute Enable|
|22|Parachute Release|
|23|Parachute 3pos|
|24|Auto Mission Reset|
|25|AttCon Feed Forward|
|26|AttCon Accel Limits|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Copter Sound|
|31|Motor Emergency Stop|
|32|Motor Interlock|
|33|Brake|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|37|Throw|
|38|ADSB Avoidance En|
|39|PrecLoiter|
|40|Proximity Avoidance|
|41|ArmDisarm (4.1 and lower)|
|42|SmartRTL|
|43|InvertedFlight|
|44|Winch Enable|
|46|RC Override Enable|
|47|User Function 1|
|48|User Function 2|
|49|User Function 3|
|52|Acro|
|55|Guided|
|56|Loiter|
|57|Follow|
|58|Clear Waypoints|
|60|ZigZag|
|61|ZigZag SaveWP|
|62|Compass Learn|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|68|Stabilize|
|69|PosHold|
|70|AltHold|
|71|FlowHold|
|72|Circle|
|73|Drift|
|75|SurfaceTrackingUpDown|
|76|Standby Mode|
|78|RunCam Control|
|79|RunCam OSD Control|
|80|VisOdom Align|
|81|Disarm|
|83|ZigZag Auto|
|84|Air Mode|
|85|Generator|
|90|EKF Pos Source|
|94|VTX Power|
|99|AUTO RTL|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|151|Turtle|
|152|simple heading reset|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with AirMode  (4.2 and higher)|
|158|Optflow Calibration|
|159|Force Flying|
|161|Turbine Start(heli)|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|212|Mount1 Roll|
|213|Mount1 Pitch|
|214|Mount1 Yaw|
|215|Mount2 Roll|
|216|Mount2 Pitch|
|217|Mount2 Yaw|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|

## BTN_FUNC4: Button Pin 4 RC Channel function

Auxiliary RC Options function executed on pin change

|Value|Meaning|
|:---:|:---:|
|0|Do Nothing|
|2|Flip|
|3|Simple Mode|
|4|RTL|
|5|Save Trim|
|7|Save WP|
|9|Camera Trigger|
|10|RangeFinder|
|11|Fence|
|13|Super Simple Mode|
|14|Acro Trainer|
|15|Sprayer|
|16|Auto|
|17|AutoTune|
|18|Land|
|19|Gripper|
|21|Parachute Enable|
|22|Parachute Release|
|23|Parachute 3pos|
|24|Auto Mission Reset|
|25|AttCon Feed Forward|
|26|AttCon Accel Limits|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Copter Sound|
|31|Motor Emergency Stop|
|32|Motor Interlock|
|33|Brake|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|37|Throw|
|38|ADSB Avoidance En|
|39|PrecLoiter|
|40|Proximity Avoidance|
|41|ArmDisarm (4.1 and lower)|
|42|SmartRTL|
|43|InvertedFlight|
|44|Winch Enable|
|46|RC Override Enable|
|47|User Function 1|
|48|User Function 2|
|49|User Function 3|
|52|Acro|
|55|Guided|
|56|Loiter|
|57|Follow|
|58|Clear Waypoints|
|60|ZigZag|
|61|ZigZag SaveWP|
|62|Compass Learn|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|68|Stabilize|
|69|PosHold|
|70|AltHold|
|71|FlowHold|
|72|Circle|
|73|Drift|
|75|SurfaceTrackingUpDown|
|76|Standby Mode|
|78|RunCam Control|
|79|RunCam OSD Control|
|80|VisOdom Align|
|81|Disarm|
|83|ZigZag Auto|
|84|Air Mode|
|85|Generator|
|90|EKF Pos Source|
|94|VTX Power|
|99|AUTO RTL|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|151|Turtle|
|152|simple heading reset|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with AirMode  (4.2 and higher)|
|158|Optflow Calibration|
|159|Force Flying|
|161|Turbine Start(heli)|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|212|Mount1 Roll|
|213|Mount1 Pitch|
|214|Mount1 Yaw|
|215|Mount2 Roll|
|216|Mount2 Pitch|
|217|Mount2 Yaw|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|

# CAM Parameters

## CAM_TRIGG_TYPE: Camera shutter (trigger) type

how to trigger the camera to take a picture

|Value|Meaning|
|:---:|:---:|
|0|Servo|
|1|Relay|
|2|GoPro in Solo Gimbal|
|3|Mount (Siyi)|

## CAM_DURATION: Duration that shutter is held open

How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)

- Units: ds

- Range: 0 50

## CAM_SERVO_ON: Servo ON PWM value

PWM value in microseconds to move servo to when shutter is activated

- Units: PWM

- Range: 1000 2000

## CAM_SERVO_OFF: Servo OFF PWM value

PWM value in microseconds to move servo to when shutter is deactivated

- Units: PWM

- Range: 1000 2000

## CAM_TRIGG_DIST: Camera trigger distance

Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.

- Units: m

- Range: 0 1000

## CAM_RELAY_ON: Relay ON value

This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera

|Value|Meaning|
|:---:|:---:|
|0|Low|
|1|High|

## CAM_MIN_INTERVAL: Minimum time between photos

Postpone shooting if previous picture was taken less than preset time(ms) ago.

- Units: ms

- Range: 0 10000

## CAM_MAX_ROLL: Maximum photo roll angle.

Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).

- Units: deg

- Range: 0 180

## CAM_FEEDBACK_PIN: Camera feedback pin

pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot. See also the CAM_FEEDBACK_POL option.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|

- RebootRequired: True

## CAM_FEEDBACK_POL: Camera feedback pin polarity

Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low

|Value|Meaning|
|:---:|:---:|
|0|TriggerLow|
|1|TriggerHigh|

## CAM_AUTO_ONLY: Distance-trigging in AUTO mode only

When enabled, trigging by distance is done in AUTO mode only.

|Value|Meaning|
|:---:|:---:|
|0|Always|
|1|Only when in AUTO|

## CAM_TYPE: Type of camera (0: None, 1: BMMCC)

Set the camera type that is being used, certain cameras have custom functions that need further configuration, this enables that.

|Value|Meaning|
|:---:|:---:|
|0|Default|
|1|BMMCC|

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

Specifies the allowed actions required to enter the OSD menu

- Bitmask: 0:Stick yaw right,1:Stick roll right,2:3-position switch,3:2-position switch,4:Autorecording enabled

# CAN Parameters

## CAN_LOGLEVEL: Loglevel

*Note: This parameter is for advanced users*

Loglevel for recording initialisation and debug information from CAN Interface

- Range: 0 4

|Value|Meaning|
|:---:|:---:|
|0|Log None|
|1|Log Error|
|2|Log Warning and below|
|3|Log Info and below|
|4|Log Everything|

# CAND1 Parameters

## CAN_D1_PROTOCOL: Enable use of specific protocol over virtual driver

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|DroneCAN|
|4|PiccoloCAN|
|5|CANTester|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|
|10|Scripting|
|11|Benewake|
|12|Scripting2|

- RebootRequired: True

# CAND1KDE Parameters

## CAN_D1_KDE_NPOLE: Number of motor poles

Sets the number of motor poles to calculate the correct RPM value

# CAND1PC Parameters

## CAN_D1_PC_ESC_BM: ESC channels

*Note: This parameter is for advanced users*

Bitmask defining which ESC (motor) channels are to be transmitted over Piccolo CAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D1_PC_ESC_RT: ESC output rate

*Note: This parameter is for advanced users*

Output rate of ESC command messages

- Units: Hz

- Range: 1 500

## CAN_D1_PC_SRV_BM: Servo channels

*Note: This parameter is for advanced users*

Bitmask defining which servo channels are to be transmitted over Piccolo CAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16

## CAN_D1_PC_SRV_RT: Servo command output rate

*Note: This parameter is for advanced users*

Output rate of servo command messages

- Units: Hz

- Range: 1 500

## CAN_D1_PC_ECU_ID: ECU Node ID

*Note: This parameter is for advanced users*

Node ID to send ECU throttle messages to. Set to zero to disable ECU throttle messages. Set to 255 to broadcast to all ECUs.

- Range: 0 255

## CAN_D1_PC_ECU_RT: ECU command output rate

*Note: This parameter is for advanced users*

Output rate of ECU command messages

- Units: Hz

- Range: 1 500

# CAND1TST Parameters

## CAN_D1_TST_ID: CAN Test Index

*Note: This parameter is for advanced users*

Selects the Index of Test that needs to be run recursively, this value gets reset to 0 at boot.

- Range: 0 4

|Value|Meaning|
|:---:|:---:|
|0|TEST_NONE|
|1|TEST_LOOPBACK|
|2|TEST_BUSOFF_RECOVERY|
|3|TEST_UAVCAN_DNA|
|5|TEST_KDE_CAN|
|6|TEST_UAVCAN_ESC|
|7|TEST_UAVCAN_FD_ESC|

## CAN_D1_TST_LPR8: CANTester LoopRate

*Note: This parameter is for advanced users*

Selects the Looprate of Test methods

- Units: us

# CAND1UC Parameters

## CAN_D1_UC_NODE: UAVCAN node that is used for this network

*Note: This parameter is for advanced users*

UAVCAN node should be set implicitly

- Range: 1 250

## CAN_D1_UC_SRV_BM: Output channels to be transmitted as servo over UAVCAN

Bitmask with one set for channel to be transmitted as a servo command over UAVCAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

## CAN_D1_UC_ESC_BM: Output channels to be transmitted as ESC over UAVCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D1_UC_SRV_RT: Servo output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for servo outputs

- Range: 1 200

- Units: Hz

## CAN_D1_UC_OPTION: UAVCAN options

*Note: This parameter is for advanced users*

Option flags

- Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy

## CAN_D1_UC_NTF_RT: Notify State rate

*Note: This parameter is for advanced users*

Maximum transmit rate for Notify State Message

- Range: 1 200

- Units: Hz

## CAN_D1_UC_ESC_OF: ESC Output channels offset

*Note: This parameter is for advanced users*

Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo functions 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficint usage of CAN bandwidth

- Range: 0 18

## CAN_D1_UC_POOL: CAN pool size

*Note: This parameter is for advanced users*

Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads

- Range: 1024 16384

# CAND2 Parameters

## CAN_D2_PROTOCOL: Enable use of specific protocol over virtual driver

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|DroneCAN|
|4|PiccoloCAN|
|5|CANTester|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|
|10|Scripting|
|11|Benewake|
|12|Scripting2|

- RebootRequired: True

# CAND2KDE Parameters

## CAN_D2_KDE_NPOLE: Number of motor poles

Sets the number of motor poles to calculate the correct RPM value

# CAND2PC Parameters

## CAN_D2_PC_ESC_BM: ESC channels

*Note: This parameter is for advanced users*

Bitmask defining which ESC (motor) channels are to be transmitted over Piccolo CAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D2_PC_ESC_RT: ESC output rate

*Note: This parameter is for advanced users*

Output rate of ESC command messages

- Units: Hz

- Range: 1 500

## CAN_D2_PC_SRV_BM: Servo channels

*Note: This parameter is for advanced users*

Bitmask defining which servo channels are to be transmitted over Piccolo CAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16

## CAN_D2_PC_SRV_RT: Servo command output rate

*Note: This parameter is for advanced users*

Output rate of servo command messages

- Units: Hz

- Range: 1 500

## CAN_D2_PC_ECU_ID: ECU Node ID

*Note: This parameter is for advanced users*

Node ID to send ECU throttle messages to. Set to zero to disable ECU throttle messages. Set to 255 to broadcast to all ECUs.

- Range: 0 255

## CAN_D2_PC_ECU_RT: ECU command output rate

*Note: This parameter is for advanced users*

Output rate of ECU command messages

- Units: Hz

- Range: 1 500

# CAND2TST Parameters

## CAN_D2_TST_ID: CAN Test Index

*Note: This parameter is for advanced users*

Selects the Index of Test that needs to be run recursively, this value gets reset to 0 at boot.

- Range: 0 4

|Value|Meaning|
|:---:|:---:|
|0|TEST_NONE|
|1|TEST_LOOPBACK|
|2|TEST_BUSOFF_RECOVERY|
|3|TEST_UAVCAN_DNA|
|5|TEST_KDE_CAN|
|6|TEST_UAVCAN_ESC|
|7|TEST_UAVCAN_FD_ESC|

## CAN_D2_TST_LPR8: CANTester LoopRate

*Note: This parameter is for advanced users*

Selects the Looprate of Test methods

- Units: us

# CAND2UC Parameters

## CAN_D2_UC_NODE: UAVCAN node that is used for this network

*Note: This parameter is for advanced users*

UAVCAN node should be set implicitly

- Range: 1 250

## CAN_D2_UC_SRV_BM: Output channels to be transmitted as servo over UAVCAN

Bitmask with one set for channel to be transmitted as a servo command over UAVCAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

## CAN_D2_UC_ESC_BM: Output channels to be transmitted as ESC over UAVCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D2_UC_SRV_RT: Servo output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for servo outputs

- Range: 1 200

- Units: Hz

## CAN_D2_UC_OPTION: UAVCAN options

*Note: This parameter is for advanced users*

Option flags

- Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy

## CAN_D2_UC_NTF_RT: Notify State rate

*Note: This parameter is for advanced users*

Maximum transmit rate for Notify State Message

- Range: 1 200

- Units: Hz

## CAN_D2_UC_ESC_OF: ESC Output channels offset

*Note: This parameter is for advanced users*

Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo functions 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficint usage of CAN bandwidth

- Range: 0 18

## CAN_D2_UC_POOL: CAN pool size

*Note: This parameter is for advanced users*

Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads

- Range: 1024 16384

# CAND3 Parameters

## CAN_D3_PROTOCOL: Enable use of specific protocol over virtual driver

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|DroneCAN|
|4|PiccoloCAN|
|5|CANTester|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|
|10|Scripting|
|11|Benewake|
|12|Scripting2|

- RebootRequired: True

# CAND3KDE Parameters

## CAN_D3_KDE_NPOLE: Number of motor poles

Sets the number of motor poles to calculate the correct RPM value

# CAND3PC Parameters

## CAN_D3_PC_ESC_BM: ESC channels

*Note: This parameter is for advanced users*

Bitmask defining which ESC (motor) channels are to be transmitted over Piccolo CAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D3_PC_ESC_RT: ESC output rate

*Note: This parameter is for advanced users*

Output rate of ESC command messages

- Units: Hz

- Range: 1 500

## CAN_D3_PC_SRV_BM: Servo channels

*Note: This parameter is for advanced users*

Bitmask defining which servo channels are to be transmitted over Piccolo CAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16

## CAN_D3_PC_SRV_RT: Servo command output rate

*Note: This parameter is for advanced users*

Output rate of servo command messages

- Units: Hz

- Range: 1 500

## CAN_D3_PC_ECU_ID: ECU Node ID

*Note: This parameter is for advanced users*

Node ID to send ECU throttle messages to. Set to zero to disable ECU throttle messages. Set to 255 to broadcast to all ECUs.

- Range: 0 255

## CAN_D3_PC_ECU_RT: ECU command output rate

*Note: This parameter is for advanced users*

Output rate of ECU command messages

- Units: Hz

- Range: 1 500

# CAND3TST Parameters

## CAN_D3_TST_ID: CAN Test Index

*Note: This parameter is for advanced users*

Selects the Index of Test that needs to be run recursively, this value gets reset to 0 at boot.

- Range: 0 4

|Value|Meaning|
|:---:|:---:|
|0|TEST_NONE|
|1|TEST_LOOPBACK|
|2|TEST_BUSOFF_RECOVERY|
|3|TEST_UAVCAN_DNA|
|5|TEST_KDE_CAN|
|6|TEST_UAVCAN_ESC|
|7|TEST_UAVCAN_FD_ESC|

## CAN_D3_TST_LPR8: CANTester LoopRate

*Note: This parameter is for advanced users*

Selects the Looprate of Test methods

- Units: us

# CAND3UC Parameters

## CAN_D3_UC_NODE: UAVCAN node that is used for this network

*Note: This parameter is for advanced users*

UAVCAN node should be set implicitly

- Range: 1 250

## CAN_D3_UC_SRV_BM: Output channels to be transmitted as servo over UAVCAN

Bitmask with one set for channel to be transmitted as a servo command over UAVCAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

## CAN_D3_UC_ESC_BM: Output channels to be transmitted as ESC over UAVCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D3_UC_SRV_RT: Servo output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for servo outputs

- Range: 1 200

- Units: Hz

## CAN_D3_UC_OPTION: UAVCAN options

*Note: This parameter is for advanced users*

Option flags

- Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy

## CAN_D3_UC_NTF_RT: Notify State rate

*Note: This parameter is for advanced users*

Maximum transmit rate for Notify State Message

- Range: 1 200

- Units: Hz

## CAN_D3_UC_ESC_OF: ESC Output channels offset

*Note: This parameter is for advanced users*

Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo functions 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficint usage of CAN bandwidth

- Range: 0 18

## CAN_D3_UC_POOL: CAN pool size

*Note: This parameter is for advanced users*

Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads

- Range: 1024 16384

# CANP1 Parameters

## CAN_P1_DRIVER: Index of virtual driver to be used with physical CAN interface

Enabling this option enables use of CAN buses.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|First driver|
|2|Second driver|
|3|Third driver|

- RebootRequired: True

## CAN_P1_BITRATE: Bitrate of CAN interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 10000 to 1000000

- Range: 10000 1000000

## CAN_P1_FDBITRATE: Bitrate of CANFD interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 1000000 to 8000000

|Value|Meaning|
|:---:|:---:|
|1|1M|
|2|2M|
|4|4M|
|5|5M|
|8|8M|

# CANP2 Parameters

## CAN_P2_DRIVER: Index of virtual driver to be used with physical CAN interface

Enabling this option enables use of CAN buses.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|First driver|
|2|Second driver|
|3|Third driver|

- RebootRequired: True

## CAN_P2_BITRATE: Bitrate of CAN interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 10000 to 1000000

- Range: 10000 1000000

## CAN_P2_FDBITRATE: Bitrate of CANFD interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 1000000 to 8000000

|Value|Meaning|
|:---:|:---:|
|1|1M|
|2|2M|
|4|4M|
|5|5M|
|8|8M|

# CANP3 Parameters

## CAN_P3_DRIVER: Index of virtual driver to be used with physical CAN interface

Enabling this option enables use of CAN buses.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|First driver|
|2|Second driver|
|3|Third driver|

- RebootRequired: True

## CAN_P3_BITRATE: Bitrate of CAN interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 10000 to 1000000

- Range: 10000 1000000

## CAN_P3_FDBITRATE: Bitrate of CANFD interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 1000000 to 8000000

|Value|Meaning|
|:---:|:---:|
|1|1M|
|2|2M|
|4|4M|
|5|5M|
|8|8M|

# CANSLCAN Parameters

## CAN_SLCAN_CPORT: SLCAN Route

CAN Interface ID to be routed to SLCAN, 0 means no routing

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|First interface|
|2|Second interface|

- RebootRequired: True

## CAN_SLCAN_SERNUM: SLCAN Serial Port

Serial Port ID to be used for temporary SLCAN iface, -1 means no temporary serial. This parameter is automatically reset on reboot or on timeout. See CAN_SLCAN_TIMOUT for timeout details

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

## CAN_SLCAN_TIMOUT: SLCAN Timeout

Duration of inactivity after which SLCAN is switched back to original driver in seconds.

- Range: 0 127

## CAN_SLCAN_SDELAY: SLCAN Start Delay

Duration after which slcan starts after setting SERNUM in seconds.

- Range: 0 127

# CC Parameters

## CC_TYPE: Custom control type

*Note: This parameter is for advanced users*

Custom control type to be used

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Empty|
|2|PID|

- RebootRequired: True

## CC_AXIS_MASK: Custom Controller bitmask

*Note: This parameter is for advanced users*

Custom Controller bitmask to chose which axis to run

- Bitmask: 0:Roll, 1:Pitch, 2:Yaw

# CHUTE Parameters

## CHUTE_ENABLED: Parachute release enabled or disabled

Parachute release enabled or disabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## CHUTE_TYPE: Parachute release mechanism type (relay or servo)

Parachute release mechanism type (relay or servo)

|Value|Meaning|
|:---:|:---:|
|0|First Relay|
|1|Second Relay|
|2|Third Relay|
|3|Fourth Relay|
|10|Servo|

## CHUTE_SERVO_ON: Parachute Servo ON PWM value

Parachute Servo PWM value in microseconds when parachute is released

- Range: 1000 2000

- Units: PWM

- Increment: 1

## CHUTE_SERVO_OFF: Servo OFF PWM value

Parachute Servo PWM value in microseconds when parachute is not released

- Range: 1000 2000

- Units: PWM

- Increment: 1

## CHUTE_ALT_MIN: Parachute min altitude in meters above home

Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.

- Range: 0 32000

- Units: m

- Increment: 1

## CHUTE_DELAY_MS: Parachute release delay

Delay in millseconds between motor stop and chute release

- Range: 0 5000

- Units: ms

- Increment: 1

## CHUTE_CRT_SINK: Critical sink speed rate in m/s to trigger emergency parachute

Release parachute when critical sink rate is reached

- Range: 0 15

- Units: m/s

- Increment: 1

## CHUTE_OPTIONS: Parachute options

Optional behaviour for parachute

- Bitmask: 0:hold open forever after release

# CIRCLE Parameters

## CIRCLE_RADIUS: Circle Radius

Defines the radius of the circle the vehicle will fly when in Circle flight mode

- Units: cm

- Range: 0 200000

- Increment: 100

## CIRCLE_RATE: Circle rate

Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise

- Units: deg/s

- Range: -90 90

- Increment: 1

## CIRCLE_OPTIONS: Circle options

0:Enable or disable using the pitch/roll stick control circle mode's radius and rate

- Bitmask: 0:manual control, 1:face direction of travel, 2:Start at center rather than on perimeter

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

- Bitmask: 0:CalRequireGPS

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

- Bitmask: 0:EnforceArming, 1:AllowNonGPSPosition

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

# EK2 Parameters

## EK2_ENABLE: Enable EKF2

*Note: This parameter is for advanced users*

This enables EKF2. Enabling EKF2 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=2. A reboot or restart will need to be performed after changing the value of EK2_ENABLE for it to take effect.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## EK2_GPS_TYPE: GPS mode control

*Note: This parameter is for advanced users*

This controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = Inhibit GPS use - this can be useful when flying with an optical flow sensor in an environment where GPS quality is poor and subject to large multipath errors.

|Value|Meaning|
|:---:|:---:|
|0|GPS 3D Vel and 2D Pos|
|1|GPS 2D vel and 2D pos|
|2|GPS 2D pos|
|3|No GPS|

## EK2_VELNE_M_NSE: GPS horizontal velocity measurement noise (m/s)

*Note: This parameter is for advanced users*

This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS horizontal velocity measurements.

- Range: 0.05 5.0

- Increment: 0.05

- Units: m/s

## EK2_VELD_M_NSE: GPS vertical velocity measurement noise (m/s)

*Note: This parameter is for advanced users*

This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS vertical velocity measurements.

- Range: 0.05 5.0

- Increment: 0.05

- Units: m/s

## EK2_VEL_I_GATE: GPS velocity innovation gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_POSNE_M_NSE: GPS horizontal position measurement noise (m)

*Note: This parameter is for advanced users*

This sets the GPS horizontal position observation noise. Increasing it reduces the weighting of GPS horizontal position measurements.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK2_POS_I_GATE: GPS position measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_GLITCH_RAD: GPS glitch radius gate size (m)

*Note: This parameter is for advanced users*

This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position.

- Range: 10 100

- Increment: 5

- Units: m

## EK2_ALT_SOURCE: Primary altitude sensor source

*Note: This parameter is for advanced users*

Primary height sensor used by the EKF. If a sensor other than Baro is selected and becomes unavailable, then the Baro sensor will be used as a fallback. NOTE: The EK2_RNG_USE_HGT parameter can be used to switch to range-finder when close to the ground in conjunction with EK2_ALT_SOURCE = 0 or 2 (Baro or GPS).

|Value|Meaning|
|:---:|:---:|
|0|Use Baro|
|1|Use Range Finder|
|2|Use GPS|
|3|Use Range Beacon|

- RebootRequired: True

## EK2_ALT_M_NSE: Altitude measurement noise (m)

*Note: This parameter is for advanced users*

This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK2_HGT_I_GATE: Height measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_HGT_DELAY: Height measurement delay (msec)

*Note: This parameter is for advanced users*

This is the number of msec that the Height measurements lag behind the inertial measurements.

- Range: 0 250

- Increment: 10

- Units: ms

- RebootRequired: True

## EK2_MAG_M_NSE: Magnetometer measurement noise (Gauss)

*Note: This parameter is for advanced users*

This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.

- Range: 0.01 0.5

- Increment: 0.01

- Units: Gauss

## EK2_MAG_CAL: Magnetometer default fusion mode

*Note: This parameter is for advanced users*

This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states, when it will use a simpler magnetic heading fusion model that does not use magnetic field states and when it will use an alternative method of yaw determination to the magnetometer. The 3-axis magnetometer fusion is only suitable for use when the external magnetic field environment is stable. EK2_MAG_CAL = 0 uses heading fusion on ground, 3-axis fusion in-flight, and is the default setting for Plane users. EK2_MAG_CAL = 1 uses 3-axis fusion only when manoeuvring. EK2_MAG_CAL = 2 uses heading fusion at all times, is recommended if the external magnetic field is varying and is the default for rovers. EK2_MAG_CAL = 3 uses heading fusion on the ground and 3-axis fusion after the first in-air field and yaw reset has completed, and is the default for copters. EK2_MAG_CAL = 4 uses 3-axis fusion at all times. NOTE: The fusion mode can be forced to 2 for specific EKF cores using the EK2_MAG_MASK parameter. NOTE: limited operation without a magnetometer or any other yaw sensor is possible by setting all COMPASS_USE, COMPASS_USE2, COMPASS_USE3, etc parameters to 0 with COMPASS_ENABLE set to 1. If this is done, the EK2_GSF_RUN and EK2_GSF_USE masks must be set to the same as EK2_IMU_MASK.

|Value|Meaning|
|:---:|:---:|
|0|When flying|
|1|When manoeuvring|
|2|Never|
|3|After first climb yaw reset|
|4|Always|

## EK2_MAG_I_GATE: Magnetometer measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_EAS_M_NSE: Equivalent airspeed measurement noise (m/s)

*Note: This parameter is for advanced users*

This is the RMS value of noise in equivalent airspeed measurements used by planes. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge. Increasing also increases navigation errors when dead-reckoning without GPS measurements.

- Range: 0.5 5.0

- Increment: 0.1

- Units: m/s

## EK2_EAS_I_GATE: Airspeed measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_RNG_M_NSE: Range finder measurement noise (m)

*Note: This parameter is for advanced users*

This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK2_RNG_I_GATE: Range finder measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_MAX_FLOW: Maximum valid optical flow rate

*Note: This parameter is for advanced users*

This sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter

- Range: 1.0 4.0

- Increment: 0.1

- Units: rad/s

## EK2_FLOW_M_NSE: Optical flow measurement noise (rad/s)

*Note: This parameter is for advanced users*

This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.

- Range: 0.05 1.0

- Increment: 0.05

- Units: rad/s

## EK2_FLOW_I_GATE: Optical Flow measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_FLOW_DELAY: Optical Flow measurement delay (msec)

*Note: This parameter is for advanced users*

This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.

- Range: 0 127

- Increment: 10

- Units: ms

- RebootRequired: True

## EK2_GYRO_P_NSE: Rate gyro noise (rad/s)

*Note: This parameter is for advanced users*

This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.

- Range: 0.0001 0.1

- Increment: 0.0001

- Units: rad/s

## EK2_ACC_P_NSE: Accelerometer noise (m/s^2)

*Note: This parameter is for advanced users*

This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.

- Range: 0.01 1.0

- Increment: 0.01

- Units: m/s/s

## EK2_GBIAS_P_NSE: Rate gyro bias stability (rad/s/s)

*Note: This parameter is for advanced users*

This state  process noise controls growth of the gyro delta angle bias state error estimate. Increasing it makes rate gyro bias estimation faster and noisier.

- Range: 0.00001 0.001

- Units: rad/s/s

## EK2_GSCL_P_NSE: Rate gyro scale factor stability (1/s)

*Note: This parameter is for advanced users*

This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.

- Range: 0.000001 0.001

- Units: Hz

## EK2_ABIAS_P_NSE: Accelerometer bias stability (m/s^3)

*Note: This parameter is for advanced users*

This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.

- Range: 0.00001 0.005

- Units: m/s/s/s

## EK2_WIND_P_NSE: Wind velocity process noise (m/s^2)

*Note: This parameter is for advanced users*

This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.

- Range: 0.01 1.0

- Increment: 0.1

- Units: m/s/s

## EK2_WIND_PSCALE: Height rate to wind process noise scaler

*Note: This parameter is for advanced users*

This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.

- Range: 0.0 1.0

- Increment: 0.1

## EK2_GPS_CHECK: GPS preflight check

*Note: This parameter is for advanced users*

This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.

- Bitmask: 0:NSats,1:HDoP,2:speed error,3:position error,4:yaw error,5:pos drift,6:vert speed,7:horiz speed

## EK2_IMU_MASK: Bitmask of active IMUs

*Note: This parameter is for advanced users*

1 byte bitmap of IMUs to use in EKF2. A separate instance of EKF2 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF2 will fail to start.

- Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU,3:FourthIMU,4:FifthIMU,5:SixthIMU

- RebootRequired: True

## EK2_CHECK_SCALE: GPS accuracy check scaler (%)

*Note: This parameter is for advanced users*

This scales the thresholds that are used to check GPS accuracy before it is used by the EKF. A value of 100 is the default. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept. A value of 200 will double the allowable GPS error.

- Range: 50 200

- Units: %

## EK2_NOAID_M_NSE: Non-GPS operation position uncertainty (m)

*Note: This parameter is for advanced users*

This sets the amount of position variation that the EKF allows for when operating without external measurements (eg GPS or optical flow). Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors.

- Range: 0.5 50.0

- Units: m

## EK2_YAW_M_NSE: Yaw measurement noise (rad)

*Note: This parameter is for advanced users*

This is the RMS value of noise in yaw measurements from the magnetometer. Increasing it reduces the weighting on these measurements.

- Range: 0.05 1.0

- Increment: 0.05

- Units: rad

## EK2_YAW_I_GATE: Yaw measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the magnetometer yaw measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_TAU_OUTPUT: Output complementary filter time constant (centi-sec)

*Note: This parameter is for advanced users*

Sets the time constant of the output complementary filter/predictor in centi-seconds.

- Range: 10 50

- Increment: 5

- Units: cs

## EK2_MAGE_P_NSE: Earth magnetic field process noise (gauss/s)

*Note: This parameter is for advanced users*

This state process noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field estimation faster and noisier.

- Range: 0.00001 0.01

- Units: Gauss/s

## EK2_MAGB_P_NSE: Body magnetic field process noise (gauss/s)

*Note: This parameter is for advanced users*

This state process noise controls the growth of body magnetic field state error estimates. Increasing it makes magnetometer bias error estimation faster and noisier.

- Range: 0.00001 0.01

- Units: Gauss/s

## EK2_RNG_USE_HGT: Range finder switch height percentage

*Note: This parameter is for advanced users*

Range finder can be used as the primary height source when below this percentage of its maximum range (see RNGFND_MAX_CM). This will not work unless Baro or GPS height is selected as the primary height source vis EK2_ALT_SOURCE = 0 or 2 respectively.  This feature should not be used for terrain following as it is designed  for vertical takeoff and landing with climb above  the range finder use height before commencing the mission, and with horizontal position changes below that height being limited to a flat region around the takeoff and landing point.

- Range: -1 70

- Increment: 1

- Units: %

## EK2_TERR_GRAD: Maximum terrain gradient

*Note: This parameter is for advanced users*

Specifies the maximum gradient of the terrain below the vehicle assumed when it is fusing range finder or optical flow to estimate terrain height.

- Range: 0 0.2

- Increment: 0.01

## EK2_BCN_M_NSE: Range beacon measurement noise (m)

*Note: This parameter is for advanced users*

This is the RMS value of noise in the range beacon measurement. Increasing it reduces the weighting on this measurement.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK2_BCN_I_GTE: Range beacon measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the range beacon measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK2_BCN_DELAY: Range beacon measurement delay (msec)

*Note: This parameter is for advanced users*

This is the number of msec that the range beacon measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.

- Range: 0 127

- Increment: 10

- Units: ms

- RebootRequired: True

## EK2_RNG_USE_SPD: Range finder max ground speed

*Note: This parameter is for advanced users*

The range finder will not be used as the primary height source when the horizontal ground speed is greater than this value.

- Range: 2.0 6.0

- Increment: 0.5

- Units: m/s

## EK2_MAG_MASK: Bitmask of active EKF cores that will always use heading fusion

*Note: This parameter is for advanced users*

1 byte bitmap of EKF cores that will disable magnetic field states and use simple magnetic heading fusion at all times. This parameter enables specified cores to be used as a backup for flight into an environment with high levels of external magnetic interference which may degrade the EKF attitude estimate when using 3-axis magnetometer fusion. NOTE : Use of a different magnetometer fusion algorithm on different cores makes unwanted EKF core switches due to magnetometer errors more likely.

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK2_OGN_HGT_MASK: Bitmask control of EKF reference height correction

*Note: This parameter is for advanced users*

When a height sensor other than GPS is used as the primary height source by the EKF, the position of the zero height datum is defined by that sensor and its frame of reference. If a GPS height measurement is also available, then the height of the WGS-84 height datum used by the EKF can be corrected so that the height returned by the getLLH() function is compensated for primary height sensor drift and change in datum over time. The first two bit positions control when the height datum will be corrected. Correction is performed using a Bayes filter and only operates when GPS quality permits. The third bit position controls where the corrections to the GPS reference datum are applied. Corrections can be applied to the local vertical position or to the reported EKF origin height (default).

- Bitmask: 0:Correct when using Baro height,1:Correct when using range finder height,2:Apply corrections to local position

- RebootRequired: True

## EK2_FLOW_USE: Optical flow use bitmask

*Note: This parameter is for advanced users*

Controls if the optical flow data is fused into the 24-state navigation estimator OR the 1-state terrain height estimator.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Navigation|
|2|Terrain|

- RebootRequired: True

## EK2_MAG_EF_LIM: EarthField error limit

*Note: This parameter is for advanced users*

This limits the difference between the learned earth magnetic field and the earth field from the world magnetic model tables. A value of zero means to disable the use of the WMM tables.

- Range: 0 500

- Units: mGauss

## EK2_HRT_FILT: Height rate filter crossover frequency

Specifies the crossover frequency of the complementary filter used to calculate the output predictor height rate derivative.

- Range: 0.1 30.0

- Units: Hz

## EK2_GSF_RUN_MASK: Bitmask of which EKF-GSF yaw estimators run

*Note: This parameter is for advanced users*

A bitmask of which EKF2 instances run an independant EKF-GSF yaw estimator to provide a backup yaw estimate that doesn't rely on magnetometer data. This estimator uses IMU, GPS and, if available, airspeed data. EKF-GSF yaw estimator data for the primary EKF2 instance will be logged as GSF0 and GSF1 messages. Use of the yaw estimate generated by this algorithm is controlled by the EK2_GSF_USE_MASK and EK2_GSF_RST_MAX parameters. To run the EKF-GSF yaw estimator in ride-along and logging only, set EK2_GSF_USE_MASK to 0. 

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK2_GSF_USE_MASK: Bitmask of which EKF-GSF yaw estimators are used

*Note: This parameter is for advanced users*

1 byte bitmap of which EKF2 instances will use the output from the EKF-GSF yaw estimator that has been turned on by the EK2_GSF_RUN_MASK parameter. If the inertial navigation calculation stops following the GPS, then the vehicle code can request EKF2 to attempt to resolve the issue, either by performing a yaw reset if enabled by this parameter by switching to another EKF2 instance.

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK2_GSF_RST_MAX: Maximum number of resets to the EKF-GSF yaw estimate allowed

*Note: This parameter is for advanced users*

Sets the maximum number of times the EKF2 will be allowed to reset its yaw to the estimate from the EKF-GSF yaw estimator. No resets will be allowed unless the use of the EKF-GSF yaw estimate is enabled via the EK2_GSF_USE_MASK parameter.

- Range: 1 10

- Increment: 1

- RebootRequired: True

# EK3 Parameters

## EK3_ENABLE: Enable EKF3

*Note: This parameter is for advanced users*

This enables EKF3. Enabling EKF3 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=3. A reboot or restart will need to be performed after changing the value of EK3_ENABLE for it to take effect.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## EK3_VELNE_M_NSE: GPS horizontal velocity measurement noise (m/s)

*Note: This parameter is for advanced users*

This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS horizontal velocity measurements.

- Range: 0.05 5.0

- Increment: 0.05

- Units: m/s

## EK3_VELD_M_NSE: GPS vertical velocity measurement noise (m/s)

*Note: This parameter is for advanced users*

This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS vertical velocity measurements.

- Range: 0.05 5.0

- Increment: 0.05

- Units: m/s

## EK3_VEL_I_GATE: GPS velocity innovation gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_POSNE_M_NSE: GPS horizontal position measurement noise (m)

*Note: This parameter is for advanced users*

This sets the GPS horizontal position observation noise. Increasing it reduces the weighting of GPS horizontal position measurements.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK3_POS_I_GATE: GPS position measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_GLITCH_RAD: GPS glitch radius gate size (m)

*Note: This parameter is for advanced users*

This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position.

- Range: 10 100

- Increment: 5

- Units: m

## EK3_ALT_M_NSE: Altitude measurement noise (m)

*Note: This parameter is for advanced users*

This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK3_HGT_I_GATE: Height measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_HGT_DELAY: Height measurement delay (msec)

*Note: This parameter is for advanced users*

This is the number of msec that the Height measurements lag behind the inertial measurements.

- Range: 0 250

- Increment: 10

- RebootRequired: True

- Units: ms

## EK3_MAG_M_NSE: Magnetometer measurement noise (Gauss)

*Note: This parameter is for advanced users*

This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.

- Range: 0.01 0.5

- Increment: 0.01

- Units: Gauss

## EK3_MAG_CAL: Magnetometer default fusion mode

*Note: This parameter is for advanced users*

This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states and when it will use a simpler magnetic heading fusion model that does not use magnetic field states. The 3-axis magnetometer fusion is only suitable for use when the external magnetic field environment is stable. EK3_MAG_CAL = 0 uses heading fusion on ground, 3-axis fusion in-flight, and is the default setting for Plane users. EK3_MAG_CAL = 1 uses 3-axis fusion only when manoeuvring. EK3_MAG_CAL = 2 uses heading fusion at all times, is recommended if the external magnetic field is varying and is the default for rovers. EK3_MAG_CAL = 3 uses heading fusion on the ground and 3-axis fusion after the first in-air field and yaw reset has completed, and is the default for copters. EK3_MAG_CAL = 4 uses 3-axis fusion at all times. EK3_MAG_CAL = 5 uses an external yaw sensor with simple heading fusion. NOTE : Use of simple heading magnetometer fusion makes vehicle compass calibration and alignment errors harder for the EKF to detect which reduces the sensitivity of the Copter EKF failsafe algorithm. NOTE: The fusion mode can be forced to 2 for specific EKF cores using the EK3_MAG_MASK parameter. EK3_MAG_CAL = 6 uses an external yaw sensor with fallback to compass when the external sensor is not available if we are flying. NOTE: The fusion mode can be forced to 2 for specific EKF cores using the EK3_MAG_MASK parameter. NOTE: limited operation without a magnetometer or any other yaw sensor is possible by setting all COMPASS_USE, COMPASS_USE2, COMPASS_USE3, etc parameters to 0 and setting COMPASS_ENABLE to 0. If this is done, the EK3_GSF_RUN and EK3_GSF_USE masks must be set to the same as EK3_IMU_MASK. A yaw angle derived from IMU and GPS velocity data using a Gaussian Sum Filter (GSF) will then be used to align the yaw when flight commences and there is sufficient movement.

|Value|Meaning|
|:---:|:---:|
|0|When flying|
|1|When manoeuvring|
|2|Never|
|3|After first climb yaw reset|
|4|Always|
|5|Use external yaw sensor (Deprecated in 4.1+ see EK3_SRCn_YAW)|
|6|External yaw sensor with compass fallback (Deprecated in 4.1+ see EK3_SRCn_YAW)|

- RebootRequired: True

## EK3_MAG_I_GATE: Magnetometer measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_EAS_M_NSE: Equivalent airspeed measurement noise (m/s)

*Note: This parameter is for advanced users*

This is the RMS value of noise in equivalent airspeed measurements used by planes. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge. Increasing also increases navigation errors when dead-reckoning without GPS measurements.

- Range: 0.5 5.0

- Increment: 0.1

- Units: m/s

## EK3_EAS_I_GATE: Airspeed measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_RNG_M_NSE: Range finder measurement noise (m)

*Note: This parameter is for advanced users*

This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK3_RNG_I_GATE: Range finder measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_MAX_FLOW: Maximum valid optical flow rate

*Note: This parameter is for advanced users*

This sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter

- Range: 1.0 4.0

- Increment: 0.1

- Units: rad/s

## EK3_FLOW_M_NSE: Optical flow measurement noise (rad/s)

*Note: This parameter is for advanced users*

This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.

- Range: 0.05 1.0

- Increment: 0.05

- Units: rad/s

## EK3_FLOW_I_GATE: Optical Flow measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_FLOW_DELAY: Optical Flow measurement delay (msec)

*Note: This parameter is for advanced users*

This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.

- Range: 0 250

- Increment: 10

- RebootRequired: True

- Units: ms

## EK3_GYRO_P_NSE: Rate gyro noise (rad/s)

*Note: This parameter is for advanced users*

This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.

- Range: 0.0001 0.1

- Increment: 0.0001

- Units: rad/s

## EK3_ACC_P_NSE: Accelerometer noise (m/s^2)

*Note: This parameter is for advanced users*

This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.

- Range: 0.01 1.0

- Increment: 0.01

- Units: m/s/s

## EK3_GBIAS_P_NSE: Rate gyro bias stability (rad/s/s)

*Note: This parameter is for advanced users*

This state  process noise controls growth of the gyro delta angle bias state error estimate. Increasing it makes rate gyro bias estimation faster and noisier.

- Range: 0.00001 0.001

- Units: rad/s/s

## EK3_ABIAS_P_NSE: Accelerometer bias stability (m/s^3)

*Note: This parameter is for advanced users*

This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.

- Range: 0.00001 0.005

- Units: m/s/s/s

## EK3_WIND_P_NSE: Wind velocity process noise (m/s^2)

*Note: This parameter is for advanced users*

This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.

- Range: 0.01 2.0

- Increment: 0.1

- Units: m/s/s

## EK3_WIND_PSCALE: Height rate to wind process noise scaler

*Note: This parameter is for advanced users*

This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.

- Range: 0.0 2.0

- Increment: 0.1

## EK3_GPS_CHECK: GPS preflight check

*Note: This parameter is for advanced users*

This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.

- Bitmask: 0:NSats,1:HDoP,2:speed error,3:position error,4:yaw error,5:pos drift,6:vert speed,7:horiz speed

## EK3_IMU_MASK: Bitmask of active IMUs

*Note: This parameter is for advanced users*

1 byte bitmap of IMUs to use in EKF3. A separate instance of EKF3 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF3 will fail to start.

- Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU,3:FourthIMU,4:FifthIMU,5:SixthIMU

- RebootRequired: True

## EK3_CHECK_SCALE: GPS accuracy check scaler (%)

*Note: This parameter is for advanced users*

This scales the thresholds that are used to check GPS accuracy before it is used by the EKF. A value of 100 is the default. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept. A value of 200 will double the allowable GPS error.

- Range: 50 200

- Units: %

## EK3_NOAID_M_NSE: Non-GPS operation position uncertainty (m)

*Note: This parameter is for advanced users*

This sets the amount of position variation that the EKF allows for when operating without external measurements (eg GPS or optical flow). Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors.

- Range: 0.5 50.0

- Units: m

## EK3_BETA_MASK: Bitmask controlling sidelip angle fusion

*Note: This parameter is for advanced users*

1 byte bitmap controlling use of sideslip angle fusion for estimation of non wind states during operation of 'fly forward' vehicle types such as fixed wing planes. By assuming that the angle of sideslip is small, the wind velocity state estimates are corrected  whenever the EKF is not dead reckoning (e.g. has an independent velocity or position sensor such as GPS). This behaviour is on by default and cannot be disabled. When the EKF is dead reckoning, the wind states are used as a reference, enabling use of the small angle of sideslip assumption to correct non wind velocity states (eg attitude, velocity, position, etc) and improve navigation accuracy. This behaviour is on by default and cannot be disabled. The behaviour controlled by this parameter is the use of the small angle of sideslip assumption to correct non wind velocity states when the EKF is NOT dead reckoning. This is primarily of benefit to reduce the buildup of yaw angle errors during straight and level flight without a yaw sensor (e.g. magnetometer or dual antenna GPS yaw) provided aerobatic flight maneuvers with large sideslip angles are not performed. The 'always' option might be used where the yaw sensor is intentionally not fitted or disabled. The 'WhenNoYawSensor' option might be used if a yaw sensor is fitted, but protection against in-flight failure and continual rejection by the EKF is desired. For vehicles operated within visual range of the operator performing frequent turning maneuvers, setting this parameter is unnecessary.

- Bitmask: 0:Always,1:WhenNoYawSensor

- RebootRequired: True

## EK3_YAW_M_NSE: Yaw measurement noise (rad)

*Note: This parameter is for advanced users*

This is the RMS value of noise in yaw measurements from the magnetometer. Increasing it reduces the weighting on these measurements.

- Range: 0.05 1.0

- Increment: 0.05

- Units: rad

## EK3_YAW_I_GATE: Yaw measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the magnetometer yaw measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_TAU_OUTPUT: Output complementary filter time constant (centi-sec)

*Note: This parameter is for advanced users*

Sets the time constant of the output complementary filter/predictor in centi-seconds.

- Range: 10 50

- Increment: 5

- Units: cs

## EK3_MAGE_P_NSE: Earth magnetic field process noise (gauss/s)

*Note: This parameter is for advanced users*

This state process noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field estimation faster and noisier.

- Range: 0.00001 0.01

- Units: Gauss/s

## EK3_MAGB_P_NSE: Body magnetic field process noise (gauss/s)

*Note: This parameter is for advanced users*

This state process noise controls the growth of body magnetic field state error estimates. Increasing it makes magnetometer bias error estimation faster and noisier.

- Range: 0.00001 0.01

- Units: Gauss/s

## EK3_RNG_USE_HGT: Range finder switch height percentage

*Note: This parameter is for advanced users*

Range finder can be used as the primary height source when below this percentage of its maximum range (see RNGFNDx_MAX_CM) and the primary height source is Baro or GPS (see EK3_SRCx_POSZ).  This feature should not be used for terrain following as it is designed for vertical takeoff and landing with climb above the range finder use height before commencing the mission, and with horizontal position changes below that height being limited to a flat region around the takeoff and landing point.

- Range: -1 70

- Increment: 1

- Units: %

## EK3_TERR_GRAD: Maximum terrain gradient

*Note: This parameter is for advanced users*

Specifies the maximum gradient of the terrain below the vehicle when it is using range finder as a height reference

- Range: 0 0.2

- Increment: 0.01

## EK3_BCN_M_NSE: Range beacon measurement noise (m)

*Note: This parameter is for advanced users*

This is the RMS value of noise in the range beacon measurement. Increasing it reduces the weighting on this measurement.

- Range: 0.1 10.0

- Increment: 0.1

- Units: m

## EK3_BCN_I_GTE: Range beacon measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the range beacon measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.

- Range: 100 1000

- Increment: 25

## EK3_BCN_DELAY: Range beacon measurement delay (msec)

*Note: This parameter is for advanced users*

This is the number of msec that the range beacon measurements lag behind the inertial measurements.

- Range: 0 250

- Increment: 10

- RebootRequired: True

- Units: ms

## EK3_RNG_USE_SPD: Range finder max ground speed

*Note: This parameter is for advanced users*

The range finder will not be used as the primary height source when the horizontal ground speed is greater than this value.

- Range: 2.0 6.0

- Increment: 0.5

- Units: m/s

## EK3_ACC_BIAS_LIM: Accelerometer bias limit

*Note: This parameter is for advanced users*

The accelerometer bias state will be limited to +- this value

- Range: 0.5 2.5

- Increment: 0.1

- Units: m/s/s

## EK3_MAG_MASK: Bitmask of active EKF cores that will always use heading fusion

*Note: This parameter is for advanced users*

1 byte bitmap of EKF cores that will disable magnetic field states and use simple magnetic heading fusion at all times. This parameter enables specified cores to be used as a backup for flight into an environment with high levels of external magnetic interference which may degrade the EKF attitude estimate when using 3-axis magnetometer fusion. NOTE : Use of a different magnetometer fusion algorithm on different cores makes unwanted EKF core switches due to magnetometer errors more likely.

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK3_OGN_HGT_MASK: Bitmask control of EKF reference height correction

*Note: This parameter is for advanced users*

When a height sensor other than GPS is used as the primary height source by the EKF, the position of the zero height datum is defined by that sensor and its frame of reference. If a GPS height measurement is also available, then the height of the WGS-84 height datum used by the EKF can be corrected so that the height returned by the getLLH() function is compensated for primary height sensor drift and change in datum over time. The first two bit positions control when the height datum will be corrected. Correction is performed using a Bayes filter and only operates when GPS quality permits. The third bit position controls where the corrections to the GPS reference datum are applied. Corrections can be applied to the local vertical position or to the reported EKF origin height (default).

- Bitmask: 0:Correct when using Baro height,1:Correct when using range finder height,2:Apply corrections to local position

- RebootRequired: True

## EK3_VIS_VERR_MIN: Visual odometry minimum velocity error

*Note: This parameter is for advanced users*

This is the 1-STD odometry velocity observation error that will be assumed when maximum quality is reported by the sensor. When quality is between max and min, the error will be calculated using linear interpolation between VIS_VERR_MIN and VIS_VERR_MAX.

- Range: 0.05 0.5

- Increment: 0.05

- Units: m/s

## EK3_VIS_VERR_MAX: Visual odometry maximum velocity error

*Note: This parameter is for advanced users*

This is the 1-STD odometry velocity observation error that will be assumed when minimum quality is reported by the sensor. When quality is between max and min, the error will be calculated using linear interpolation between VIS_VERR_MIN and VIS_VERR_MAX.

- Range: 0.5 5.0

- Increment: 0.1

- Units: m/s

## EK3_WENC_VERR: Wheel odometry velocity error

*Note: This parameter is for advanced users*

This is the 1-STD odometry velocity observation error that will be assumed when wheel encoder data is being fused.

- Range: 0.01 1.0

- Increment: 0.1

- Units: m/s

## EK3_FLOW_USE: Optical flow use bitmask

*Note: This parameter is for advanced users*

Controls if the optical flow data is fused into the 24-state navigation estimator OR the 1-state terrain height estimator.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Navigation|
|2|Terrain|

- RebootRequired: True

## EK3_HRT_FILT: Height rate filter crossover frequency

Specifies the crossover frequency of the complementary filter used to calculate the output predictor height rate derivative.

- Range: 0.1 30.0

- Units: Hz

## EK3_MAG_EF_LIM: EarthField error limit

*Note: This parameter is for advanced users*

This limits the difference between the learned earth magnetic field and the earth field from the world magnetic model tables. A value of zero means to disable the use of the WMM tables.

- Range: 0 500

- Units: mGauss

## EK3_GSF_RUN_MASK: Bitmask of which EKF-GSF yaw estimators run

*Note: This parameter is for advanced users*

1 byte bitmap of which EKF3 instances run an independant EKF-GSF yaw estimator to provide a backup yaw estimate that doesn't rely on magnetometer data. This estimator uses IMU, GPS and, if available, airspeed data. EKF-GSF yaw estimator data for the primary EKF3 instance will be logged as GSF0 and GSF1 messages. Use of the yaw estimate generated by this algorithm is controlled by the EK3_GSF_USE_MASK and EK3_GSF_RST_MAX parameters. To run the EKF-GSF yaw estimator in ride-along and logging only, set EK3_GSF_USE to 0. 

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK3_GSF_USE_MASK: Bitmask of which EKF-GSF yaw estimators are used

*Note: This parameter is for advanced users*

A bitmask of which EKF3 instances will use the output from the EKF-GSF yaw estimator that has been turned on by the EK3_GSF_RUN_MASK parameter. If the inertial navigation calculation stops following the GPS, then the vehicle code can request EKF3 to attempt to resolve the issue, either by performing a yaw reset if enabled by this parameter by switching to another EKF3 instance.

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK3_GSF_RST_MAX: Maximum number of resets to the EKF-GSF yaw estimate allowed

*Note: This parameter is for advanced users*

Sets the maximum number of times the EKF3 will be allowed to reset its yaw to the estimate from the EKF-GSF yaw estimator. No resets will be allowed unless the use of the EKF-GSF yaw estimate is enabled via the EK3_GSF_USE_MASK parameter.

- Range: 1 10

- Increment: 1

- RebootRequired: True

## EK3_ERR_THRESH: EKF3 Lane Relative Error Sensitivity Threshold

*Note: This parameter is for advanced users*

lanes have to be consistently better than the primary by at least this threshold to reduce their overall relativeCoreError, lowering this makes lane switching more sensitive to smaller error differences

- Range: 0.05 1

- Increment: 0.05

## EK3_AFFINITY: EKF3 Sensor Affinity Options

*Note: This parameter is for advanced users*

These options control the affinity between sensor instances and EKF cores

- Bitmask: 0:EnableGPSAffinity,1:EnableBaroAffinity,2:EnableCompassAffinity,3:EnableAirspeedAffinity

- RebootRequired: True

## EK3_DRAG_BCOEF_X: Ballistic coefficient for X axis drag

*Note: This parameter is for advanced users*

Ratio of mass to drag coefficient measured along the X body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared.  Set to a postive value > 1.0 to enable. A starting value is the mass in Kg divided by the frontal area. The predicted drag from the rotors is specified separately by the EK3_DRAG_MCOEF parameter.

- Range: 0.0 1000.0

- Units: kg/m/m

## EK3_DRAG_BCOEF_Y: Ballistic coefficient for Y axis drag

*Note: This parameter is for advanced users*

Ratio of mass to drag coefficient measured along the Y body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared.  Set to a postive value > 1.0 to enable. A starting value is the mass in Kg divided by the side area. The predicted drag from the rotors is specified separately by the EK3_DRAG_MCOEF parameter.

- Range: 50.0 1000.0

- Units: kg/m/m

## EK3_DRAG_M_NSE: Observation noise for drag acceleration

*Note: This parameter is for advanced users*

This sets the amount of noise used when fusing X and Y acceleration as an observation that enables esitmation of wind velocity for multi-rotor vehicles. This feature is enabled by the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters

- Range: 0.1 2.0

- Increment: 0.1

- Units: m/s/s

## EK3_DRAG_MCOEF: Momentum coefficient for propeller drag

*Note: This parameter is for advanced users*

This parameter is used to predict the drag produced by the rotors when flying a multi-copter, enabling estimation of wind drift. The drag produced by this effect scales with speed not speed squared and is produced because some of the air velocity normal to the rotors axis of rotation is lost when passing through the rotor disc which changes the momentum of the airflow causing drag. For unducted rotors the effect is roughly proportional to the area of the propeller blades when viewed side on and changes with different propellers. It is higher for ducted rotors. For example if flying at 15 m/s at sea level conditions produces a rotor induced drag acceleration of 1.5 m/s/s, then EK3_DRAG_MCOEF would be set to 0.1 = (1.5/15.0). Set EK3_MCOEF to a postive value to enable wind estimation using this drag effect. To account for the drag produced by the body which scales with speed squared, see documentation for the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters.

- Range: 0.0 1.0

- Increment: 0.01

- Units: 1/s

## EK3_OGNM_TEST_SF: On ground not moving test scale factor

*Note: This parameter is for advanced users*

This parameter is adjust the sensitivity of the on ground not moving test which is used to assist with learning the yaw gyro bias and stopping yaw drift before flight when operating without a yaw sensor. Bigger values allow the detection of a not moving condition with noiser IMU data. Check the XKFM data logged when the vehicle is on ground not moving and adjust the value of OGNM_TEST_SF to be slightly higher than the maximum value of the XKFM.ADR, XKFM.ALR, XKFM.GDR and XKFM.GLR test levels.

- Range: 1.0 10.0

- Increment: 0.5

## EK3_GND_EFF_DZ: Baro height ground effect dead zone

*Note: This parameter is for advanced users*

This parameter sets the size of the dead zone that is applied to negative baro height spikes that can occur when taking off or landing when a vehicle with lift rotors is operating in ground effect ground effect. Set to about 0.5m less than the amount of negative offset in baro height that occurs just prior to takeoff when lift motors are spooling up. Set to 0 if no ground effect is present.

- Range: 0.0 10.0

- Increment: 0.5

## EK3_PRIMARY: Primary core number

*Note: This parameter is for advanced users*

The core number (index in IMU mask) that will be used as the primary EKF core on startup. While disarmed the EKF will force the use of this core. A value of 0 corresponds to the first IMU in EK3_IMU_MASK.

- Range: 0 2

- Increment: 1

## EK3_LOG_LEVEL: Logging Level

*Note: This parameter is for advanced users*

Determines how verbose the EKF3 streaming logging is. A value of 0 provides full logging(default), a value of 1 only XKF4 scaled innovations are logged, a value of 2 both XKF4 and GSF are logged, and a value of 3 disables all streaming logging of EKF3.

- Range: 0 3

- Increment: 1

## EK3_GPS_VACC_MAX: GPS vertical accuracy threshold

*Note: This parameter is for advanced users*

Vertical accuracy threshold for GPS as the altitude source. The GPS will not be used as an altitude source if the reported vertical accuracy of the GPS is larger than this threshold, falling back to baro instead. Set to zero to deactivate the threshold check.

- Range: 0.0 10.0

- Increment: 0.1

- Units: m

# EK3SRC Parameters

## EK3_SRC1_POSXY: Position Horizontal Source (Primary)

*Note: This parameter is for advanced users*

Position Horizontal Source (Primary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC1_VELXY: Velocity Horizontal Source

*Note: This parameter is for advanced users*

Velocity Horizontal Source

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|5|OpticalFlow|
|6|ExternalNav|
|7|WheelEncoder|

## EK3_SRC1_POSZ: Position Vertical Source

*Note: This parameter is for advanced users*

Position Vertical Source

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Baro|
|2|RangeFinder|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC1_VELZ: Velocity Vertical Source

*Note: This parameter is for advanced users*

Velocity Vertical Source

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC1_YAW: Yaw Source

*Note: This parameter is for advanced users*

Yaw Source

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Compass|
|2|GPS|
|3|GPS with Compass Fallback|
|6|ExternalNav|
|8|GSF|

## EK3_SRC2_POSXY: Position Horizontal Source (Secondary)

*Note: This parameter is for advanced users*

Position Horizontal Source (Secondary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC2_VELXY: Velocity Horizontal Source (Secondary)

*Note: This parameter is for advanced users*

Velocity Horizontal Source (Secondary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|5|OpticalFlow|
|6|ExternalNav|
|7|WheelEncoder|

## EK3_SRC2_POSZ: Position Vertical Source (Secondary)

*Note: This parameter is for advanced users*

Position Vertical Source (Secondary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Baro|
|2|RangeFinder|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC2_VELZ: Velocity Vertical Source (Secondary)

*Note: This parameter is for advanced users*

Velocity Vertical Source (Secondary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC2_YAW: Yaw Source (Secondary)

*Note: This parameter is for advanced users*

Yaw Source (Secondary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Compass|
|2|GPS|
|3|GPS with Compass Fallback|
|6|ExternalNav|
|8|GSF|

## EK3_SRC3_POSXY: Position Horizontal Source (Tertiary)

*Note: This parameter is for advanced users*

Position Horizontal Source (Tertiary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC3_VELXY: Velocity Horizontal Source (Tertiary)

*Note: This parameter is for advanced users*

Velocity Horizontal Source (Tertiary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|5|OpticalFlow|
|6|ExternalNav|
|7|WheelEncoder|

## EK3_SRC3_POSZ: Position Vertical Source (Tertiary)

*Note: This parameter is for advanced users*

Position Vertical Source (Tertiary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Baro|
|2|RangeFinder|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC3_VELZ: Velocity Vertical Source (Tertiary)

*Note: This parameter is for advanced users*

Velocity Vertical Source (Tertiary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|3|GPS|
|4|Beacon|
|6|ExternalNav|

## EK3_SRC3_YAW: Yaw Source (Tertiary)

*Note: This parameter is for advanced users*

Yaw Source (Tertiary)

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Compass|
|2|GPS|
|3|GPS with Compass Fallback|
|6|ExternalNav|
|8|GSF|

## EK3_SRC_OPTIONS: EKF Source Options

*Note: This parameter is for advanced users*

EKF Source Options

- Bitmask: 0:FuseAllVelocities

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

## FENCE_TYPE: Fence Type

Enabled fence types held as bitmask

- Bitmask: 0:Max altitude,1:Circle,2:Polygon,3:Min altitude

## FENCE_ACTION: Fence Action

What action should be taken when fence is breached

|Value|Meaning|
|:---:|:---:|
|0|Report Only|
|1|RTL or Land|
|2|Always Land|
|3|SmartRTL or RTL or Land|
|4|Brake or Land|
|5|SmartRTL or Land|

## FENCE_ALT_MAX: Fence Maximum Altitude

Maximum altitude allowed before geofence triggers

- Units: m

- Range: 10 1000

- Increment: 1

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

## FENCE_ALT_MIN: Fence Minimum Altitude

Minimum altitude allowed before geofence triggers

- Units: m

- Range: -100 100

- Increment: 1

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

# FHLD Parameters

## FHLD_XY_P: FlowHold P gain

*Note: This parameter is for advanced users*

FlowHold (horizontal) P gain.

- Range: 0.1 6.0

- Increment: 0.1

## FHLD_XY_I: FlowHold I gain

*Note: This parameter is for advanced users*

FlowHold (horizontal) I gain

- Range: 0.02 1.00

- Increment: 0.01

## FHLD_XY_IMAX: FlowHold Integrator Max

*Note: This parameter is for advanced users*

FlowHold (horizontal) integrator maximum

- Range: 0 4500

- Increment: 10

- Units: cdeg

## FHLD_XY_FILT_HZ: FlowHold filter on input to control

*Note: This parameter is for advanced users*

FlowHold (horizontal) filter on input to control

- Range: 0 100

- Units: Hz

## FHLD_FLOW_MAX: FlowHold Flow Rate Max

Controls maximum apparent flow rate in flowhold

- Range: 0.1 2.5

## FHLD_FILT_HZ: FlowHold Filter Frequency

Filter frequency for flow data

- Range: 1 100

- Units: Hz

## FHLD_QUAL_MIN: FlowHold Flow quality minimum

Minimum flow quality to use flow position hold

- Range: 0 255

## FHLD_BRAKE_RATE: FlowHold Braking rate

Controls deceleration rate on stick release

- Range: 1 30

- Units: deg/s

# FLOW Parameters

## FLOW_TYPE: Optical flow sensor type

Optical flow sensor type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|PX4Flow|
|2|Pixart|
|3|Bebop|
|4|CXOF|
|5|MAVLink|
|6|DroneCAN|
|7|MSP|
|8|UPFLOW|

- RebootRequired: True

## FLOW_FXSCALER: X axis optical flow scale factor correction

This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.

- Range: -200 +200

- Increment: 1

## FLOW_FYSCALER: Y axis optical flow scale factor correction

This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.

- Range: -200 +200

- Increment: 1

## FLOW_ORIENT_YAW: Flow sensor yaw alignment

Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.

- Units: cdeg

- Range: -17999 +18000

- Increment: 10

## FLOW_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the optical flow sensor focal point in body frame. Positive X is forward of the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## FLOW_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the optical flow sensor focal point in body frame. Positive Y is to the right of the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## FLOW_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the optical flow sensor focal point in body frame. Positive Z is down from the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## FLOW_ADDR: Address on the bus

*Note: This parameter is for advanced users*

This is used to select between multiple possible I2C addresses for some sensor types. For PX4Flow you can choose 0 to 7 for the 8 possible addresses on the I2C bus.

- Range: 0 127

# FOLL Parameters

## FOLL_ENABLE: Follow enable/disable

Enabled/disable following a target

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## FOLL_SYSID: Follow target's mavlink system id

Follow target's mavlink system id

- Range: 0 255

## FOLL_DIST_MAX: Follow distance maximum

Follow distance maximum.  targets further than this will be ignored

- Units: m

- Range: 1 1000

## FOLL_OFS_TYPE: Follow offset type

Follow offset type

|Value|Meaning|
|:---:|:---:|
|0|North-East-Down|
|1|Relative to lead vehicle heading|

## FOLL_OFS_X: Follow offsets in meters north/forward

Follow offsets in meters north/forward.  If positive, this vehicle fly ahead or north of lead vehicle.  Depends on FOLL_OFS_TYPE

- Range: -100 100

- Units: m

- Increment: 1

## FOLL_OFS_Y: Follow offsets in meters east/right

Follow offsets in meters east/right.  If positive, this vehicle will fly to the right or east of lead vehicle.  Depends on FOLL_OFS_TYPE

- Range: -100 100

- Units: m

- Increment: 1

## FOLL_OFS_Z: Follow offsets in meters down

Follow offsets in meters down.  If positive, this vehicle will fly below the lead vehicle

- Range: -100 100

- Units: m

- Increment: 1

## FOLL_YAW_BEHAVE: Follow yaw behaviour

Follow yaw behaviour

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Face Lead Vehicle|
|2|Same as Lead vehicle|
|3|Direction of Flight|

## FOLL_POS_P: Follow position error P gain

Follow position error P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller

- Range: 0.01 1.00

- Increment: 0.01

## FOLL_ALT_TYPE: Follow altitude type

Follow altitude type

|Value|Meaning|
|:---:|:---:|
|0|absolute|
|1|relative|

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

X position of the base GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB1_OFS_Y: Base antenna Y position offset    

*Note: This parameter is for advanced users*

Y position of the base GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB1_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.

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

X position of the base GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB2_OFS_Y: Base antenna Y position offset    

*Note: This parameter is for advanced users*

Y position of the base GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB2_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

# GRIP Parameters

## GRIP_ENABLE: Gripper Enable/Disable

Gripper enable/disable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## GRIP_TYPE: Gripper Type

Gripper enable/disable

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Servo|
|2|EPM|

## GRIP_GRAB: Gripper Grab PWM

*Note: This parameter is for advanced users*

PWM value in microseconds sent to Gripper to initiate grabbing the cargo

- Range: 1000 2000

- Units: PWM

## GRIP_RELEASE: Gripper Release PWM

*Note: This parameter is for advanced users*

PWM value in microseconds sent to Gripper to release the cargo

- Range: 1000 2000

- Units: PWM

## GRIP_NEUTRAL: Neutral PWM

*Note: This parameter is for advanced users*

PWM value in microseconds sent to grabber when not grabbing or releasing

- Range: 1000 2000

- Units: PWM

## GRIP_REGRAB: EPM Gripper Regrab interval

*Note: This parameter is for advanced users*

Time in seconds that EPM gripper will regrab the cargo to ensure grip has not weakened; 0 to disable

- Range: 0 255

- Units: s

## GRIP_CAN_ID: EPM UAVCAN Hardpoint ID

Refer to https://docs.zubax.com/opengrab_epm_v3#UAVCAN_interface

- Range: 0 255

## GRIP_AUTOCLOSE: Gripper Autoclose time

*Note: This parameter is for advanced users*

Time in seconds that gripper close the gripper after opening; 0 to disable

- Range: 0.25 255

- Units: s

# H Parameters

## H_TAIL_TYPE: Tail Type

Tail type selection.  Simpler yaw controller used if external gyro is selected. Direct Drive Variable Pitch is used for tails that have a motor that is governed at constant speed by an ESC.  Tail pitch is still accomplished with a servo.  Direct Drive Fixed Pitch (DDFP) CW is used for helicopters with a rotor that spins clockwise when viewed from above. Direct Drive Fixed Pitch (DDFP) CCW is used for helicopters with a rotor that spins counter clockwise when viewed from above. In both DDFP cases, no servo is used for the tail and the tail motor esc is controlled by the yaw axis.

|Value|Meaning|
|:---:|:---:|
|0|Servo only|
|1|Servo with ExtGyro|
|2|DirectDrive VarPitch|
|3|DirectDrive FixedPitch CW|
|4|DirectDrive FixedPitch CCW|
|5|DDVP with external governor|

## H_GYR_GAIN: External Gyro Gain

PWM in microseconds sent to external gyro on ch7 when tail type is Servo w/ ExtGyro

- Range: 0 1000

- Units: PWM

- Increment: 1

## H_COLYAW: Collective-Yaw Mixing

Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.

- Range: -10 10

- Increment: 0.1

## H_FLYBAR_MODE: Flybar Mode Selector

Flybar present or not.  Affects attitude controller used during ACRO flight mode

|Value|Meaning|
|:---:|:---:|
|0|NoFlybar|
|1|Flybar|

## H_TAIL_SPEED: DDVP Tail ESC speed

Direct drive, variable pitch tail ESC speed in percent output to the tail motor esc (HeliTailRSC Servo) when motor interlock enabled (throttle hold off).

- Range: 0 100

- Units: %

- Increment: 1

## H_GYR_GAIN_ACRO: ACRO External Gyro Gain

PWM in microseconds sent to external gyro on ch7 when tail type is Servo w/ ExtGyro. A value of zero means to use H_GYR_GAIN

- Range: 0 1000

- Units: PWM

- Increment: 1

## H_SW_TYPE: Swashplate Type

H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 

|Value|Meaning|
|:---:|:---:|
|0|H3 Generic|
|1|H1 non-CPPM|
|2|H3_140|
|3|H3_120|
|4|H4_90|
|5|H4_45|

## H_SW_COL_DIR: Collective Direction

Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## H_SW_LIN_SVO: Linearize Swash Servos

This linearizes the swashplate servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## H_SW_H3_ENABLE: H3 Generic Enable

*Note: This parameter is for advanced users*

Automatically set when H3 generic swash type is selected for swashplate. Do not set manually.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## H_SW_H3_SV1_POS: H3 Generic Servo 1 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW_H3_SV2_POS: H3 Generic Servo 2 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate for servo 2 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW_H3_SV3_POS: H3 Generic Servo 3 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate for servo 3 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW_H3_PHANG: H3 Generic Phase Angle Comp

*Note: This parameter is for advanced users*

Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem

- Range: -30 30

- Units: deg

- Increment: 1

## H_DUAL_MODE: Dual Mode

Sets the dual mode of the heli, either as tandem or as transverse.

|Value|Meaning|
|:---:|:---:|
|0|Longitudinal|
|1|Transverse|
|2|Intermeshing|

## H_DCP_SCALER: Differential-Collective-Pitch Scaler

Scaling factor applied to the differential-collective-pitch

- Range: 0 1

## H_DCP_YAW: Differential-Collective-Pitch Yaw Mixing

Feed-forward compensation to automatically add yaw input when differential collective pitch is applied.  Disabled for intermeshing mode.

- Range: -10 10

- Increment: 0.1

## H_YAW_SCALER: Scaler for yaw mixing

Scaler for mixing yaw into roll or pitch.

- Range: -10 10

- Increment: 0.1

## H_COL2_MIN: Swash 2 Minimum Collective Pitch

Lowest possible servo position in PWM microseconds for swashplate 2

- Range: 1000 2000

- Units: PWM

- Increment: 1

## H_COL2_MAX: Swash 2 Maximum Collective Pitch

Highest possible servo position in PWM microseconds for swashplate 2

- Range: 1000 2000

- Units: PWM

- Increment: 1

## H_SW_TYPE: Swash 1 Type

H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 

|Value|Meaning|
|:---:|:---:|
|0|H3 Generic|
|1|H1 non-CPPM|
|2|H3_140|
|3|H3_120|
|4|H4_90|
|5|H4_45|

## H_SW_COL_DIR: Swash 1 Collective Direction

Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## H_SW_LIN_SVO: Linearize Swash 1 Servos

This linearizes the swashplate 1 servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## H_SW_H3_ENABLE: Swash 1 H3 Generic Enable

*Note: This parameter is for advanced users*

Automatically set when H3 generic swash type is selected for swashplate 1. Do not set manually.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## H_SW_H3_SV1_POS: Swash 1 H3 Generic Servo 1 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW_H3_SV2_POS: Swash 1 H3 Generic Servo 2 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate 1 for servo 2 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW_H3_SV3_POS: Swash 1 H3 Generic Servo 3 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate 1 for servo 3 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW_H3_PHANG: Swash 1 H3 Generic Phase Angle Comp

*Note: This parameter is for advanced users*

Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem

- Range: -30 30

- Units: deg

- Increment: 1

## H_SW2_TYPE: Swash 2 Type

H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 

|Value|Meaning|
|:---:|:---:|
|0|H3 Generic|
|1|H1 non-CPPM|
|2|H3_140|
|3|H3_120|
|4|H4_90|
|5|H4_45|

## H_SW2_COL_DIR: Swash 2 Collective Direction

Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## H_SW2_LIN_SVO: Linearize Swash 2 Servos

This linearizes the swashplate 2 servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## H_SW2_H3_ENABLE: Swash 2 H3 Generic Enable

*Note: This parameter is for advanced users*

Automatically set when H3 generic swash type is selected for swashplate 2. Do not set manually.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## H_SW2_H3_SV1_POS: Swash 2 H3 Generic Servo 1 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW2_H3_SV2_POS: Swash 2 H3 Generic Servo 2 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate 2 for servo 2 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW2_H3_SV3_POS: Swash 2 H3 Generic Servo 3 Position

*Note: This parameter is for advanced users*

Azimuth position on swashplate 2 for servo 3 with the front of the heli being 0 deg

- Range: -180 180

- Units: deg

## H_SW2_H3_PHANG: Swash 2 H3 Generic Phase Angle Comp

*Note: This parameter is for advanced users*

Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem

- Range: -30 30

- Units: deg

- Increment: 1

## H_DCP_TRIM: Differential Collective Pitch Trim

Removes I term bias due to center of gravity offsets or discrepancies between rotors in swashplate setup. If DCP axis has I term bias while hovering in calm winds, use value of bias in DCP_TRIM to re-center I term.

- Range: -0.2 0.2

- Increment: 0.01

## H_YAW_REV_EXPO: Yaw reverser expo

For intermeshing mode only. Yaw revereser smoothing exponent, smoothen transition near zero collective region. Increase this parameter to shink smoothing range. Set to -1 to disable reverser. 

- Range: -1 1000

- Increment: 1.0

## H_COL_MIN: Minimum Collective Pitch

Lowest possible servo position in PWM microseconds for the swashplate

- Range: 1000 2000

- Units: PWM

- Increment: 1

## H_COL_MAX: Maximum Collective Pitch

Highest possible servo position in PWM microseconds for the swashplate

- Range: 1000 2000

- Units: PWM

- Increment: 1

## H_SV_MAN: Manual Servo Mode

Manual servo override for swash set-up. Must be 0 (Disabled) for flight!

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Passthrough|
|2|Max collective|
|3|Zero thrust collective|
|4|Min collective|

## H_CYC_MAX: Maximum Cyclic Pitch Angle

Maximum cyclic pitch angle of the swash plate.  There are no units to this parameter.  This should be adjusted to get the desired cyclic blade pitch for the pitch and roll axes.  Typically this should be 6-7 deg (measured blade pitch angle difference between stick centered and stick max deflection.

- Range: 0 4500

- Increment: 100

## H_SV_TEST: Boot-up Servo Test Cycles

Number of cycles to run servo test on boot-up

- Range: 0 10

- Increment: 1

## H_COL_HOVER: Collective Hover Value

*Note: This parameter is for advanced users*

Collective needed to hover expressed as a number from 0 to 1 where 0 is H_COL_MIN and 1 is H_COL_MAX

- Range: 0.3 0.8

## H_HOVER_LEARN: Hover Value Learning

*Note: This parameter is for advanced users*

Enable/Disable automatic learning of hover collective

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Learn|
|2|Learn and Save|

## H_OPTIONS: Heli_Options

Bitmask of heli options.  Bit 0 changes how the pitch, roll, and yaw axis integrator term is managed for low speed and takeoff/landing. In AC 4.0 and earlier, scheme uses a leaky integrator for ground speeds less than 5 m/s and won't let the steady state integrator build above ILMI. The integrator is allowed to build to the ILMI value when it is landed.  The other integrator management scheme bases integrator limiting on takeoff and landing.  Whenever the aircraft is landed the integrator is set to zero.  When the aicraft is airborne, the integrator is only limited by IMAX. 

- Bitmask: 0:Use Leaky I

## H_COL_ANG_MIN: Collective Blade Pitch Angle Minimum

Minimum collective blade pitch angle in deg that corresponds to the PWM set for minimum collective pitch (H_COL_MIN).

- Range: -20 0

- Units: deg

- Increment: 0.1

## H_COL_ANG_MAX: Collective Blade Pitch Angle Maximum

Maximum collective blade pitch angle in deg that corresponds to the PWM set for maximum collective pitch (H_COL_MAX).

- Range: 5 20

- Units: deg

- Increment: 0.1

## H_COL_ZERO_THRST: Collective Blade Pitch at Zero Thrust

Collective blade pitch angle at zero thrust in degrees. For symetric airfoil blades this value is zero deg. For chambered airfoil blades this value is typically negative.

- Range: -5 0

- Units: deg

- Increment: 0.1

## H_COL_LAND_MIN: Collective Blade Pitch Minimum when Landed

Minimum collective blade pitch angle when landed in degrees for non-manual collective modes (i.e. modes that use altitude hold).

- Range: -5 0

- Units: deg

- Increment: 0.1

# HRSC Parameters

## H_RSC_SETPOINT: External Motor Governor Setpoint

Throttle (HeliRSC Servo) output in percent to the external motor governor when motor interlock enabled (throttle hold off).

- Range: 0 100

- Units: %

- Increment: 1

## H_RSC_MODE: Rotor Speed Control Mode

Selects the type of rotor speed control used to determine throttle output to the HeliRSC servo channel when motor interlock is enabled (throttle hold off). RC Passthrough sends the input from the RC Motor Interlock channel as throttle output.  External Gov SetPoint sends the RSC SetPoint parameter value as throttle output.  Throttle Curve uses the 5 point throttle curve to determine throttle output based on the collective output.  AutoThrottle requires a rotor speed sensor, contains an advanced autothrottle governor and is primarily for piston and turbine engines. WARNING: Throttle ramp time and throttle curve MUST be tuned properly using Throttle Curve mode before using AutoThrottle

|Value|Meaning|
|:---:|:---:|
|1|RC Passthrough|
|2|External Gov SetPoint|
|3|Throttle Curve|
|4|AutoThrottle|

## H_RSC_RAMP_TIME: Throttle Ramp Time

Time in seconds for throttle output (HeliRSC servo) to ramp from ground idle (RSC_IDLE) to flight idle throttle setting when motor interlock is enabled (throttle hold off).

- Range: 0 60

- Units: s

## H_RSC_RUNUP_TIME: Rotor Runup Time

Actual time in seconds for the main rotor to reach full speed after motor interlock is enabled (throttle hold off). Must be at least one second longer than the Throttle Ramp Time that is set with RSC_RAMP_TIME. WARNING: For AutoThrottle users with piston and turbine engines it is VERY important to know how long it takes to warm up your engine and reach full rotor speed when throttle switch is turned ON. This timer should be set for at least the amount of time it takes to get your helicopter to full flight power, ready for takeoff. Failure to heed this warning could result in the auto-takeoff mode attempting to lift up into hover before the engine has reached full power, and subsequent loss of control

- Range: 0 60

- Units: s

## H_RSC_CRITICAL: Critical Rotor Speed

Percentage of normal rotor speed where flight is no longer possible. However currently the rotor runup/rundown is estimated using the RSC_RUNUP_TIME parameter.   Estimated rotor speed increases/decreases between 0 (rotor stopped) to 1 (rotor at normal speed) in the RSC_RUNUP_TIME in seconds. This parameter should be set so that the estimated rotor speed goes below critical in approximately 3 seconds.  So if you had a 10 second runup time then set RSC_CRITICAL to 70%.

- Range: 0 100

- Units: %

- Increment: 1

## H_RSC_IDLE: Throttle Output at Idle

Throttle output (HeliRSC Servo) in percent while armed but motor interlock is disabled (throttle hold on). FOR COMBUSTION ENGINES. Sets the engine ground idle throttle percentage with clutch disengaged. This must be set to zero for electric helicopters under most situations. If the ESC has an autorotation window this can be set to keep the autorotation window open in the ESC. Consult the operating manual for your ESC to set it properly for this purpose

- Range: 0 50

- Units: %

- Increment: 1

## H_RSC_SLEWRATE: Throttle Slew Rate

This controls the maximum rate at which the throttle output (HeliRSC servo) can change, as a percentage per second. A value of 100 means the throttle can change over its full range in one second. A value of zero gives unlimited slew rate.

- Range: 0 500

- Increment: 10

## H_RSC_THRCRV_0: Throttle Curve at 0% Coll

Sets the throttle output (HeliRSC servo) in percent for the throttle curve at the minimum collective pitch position. The 0 percent collective is defined by H_COL_MIN. Example: if the setup has -2 degree to +10 degree collective pitch setup, this setting would correspond to -2 degree of pitch.

- Range: 0 100

- Units: %

- Increment: 1

## H_RSC_THRCRV_25: Throttle Curve at 25% Coll

Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 25% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 25% of 12 degrees is 3 degrees, so this setting would correspond to +1 degree of pitch.

- Range: 0 100

- Units: %

- Increment: 1

## H_RSC_THRCRV_50: Throttle Curve at 50% Coll

Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 50% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 50% of 12 degrees is 6 degrees, so this setting would correspond to +4 degree of pitch.

- Range: 0 100

- Units: %

- Increment: 1

## H_RSC_THRCRV_75: Throttle Curve at 75% Coll

Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 75% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 75% of 12 degrees is 9 degrees, so this setting would correspond to +7 degree of pitch.

- Range: 0 100

- Units: %

- Increment: 1

## H_RSC_THRCRV_100: Throttle Curve at 100% Coll

Sets the throttle output (HeliRSC servo) in percent for the throttle curve at the minimum collective pitch position. The 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, this setting would correspond to +10 degree of pitch.

- Range: 0 100

- Units: %

- Increment: 1

## H_RSC_GOV_RANGE: Governor Operational Range

RPM range +/- governor rpm reference setting where governor is operational. If speed sensor fails or rpm falls outside of this range, the governor will disengage and return to throttle curve. Recommended range is 100

- Range: 50 200

- Units: RPM

- Increment: 10

## H_RSC_AROT_PCT: Autorotation Throttle Percentage for External Governor

The throttle percentage sent to external governors, signaling to enable fast spool-up, when bailing out of an autorotation.  Set 0 to disable. If also using a tail rotor of type DDVP with external governor then this value must lie within the autorotation window of both governors.

- Range: 0 40

- Units: %

- Increment: 1

## H_RSC_CLDWN_TIME: Cooldown Time

Will provide a fast idle for engine cooldown by raising the Ground Idle speed setting by 50% for the number of seconds the timer is set for. A setting of zero disables the fast idle. This feature will only apply after the runup complete has been declared. This will not extend the time before ground idle is declared, which triggers engine shutdown for autonomous landings.

- Range: 0 120

- Units: s

- Increment: 1

## H_RSC_GOV_COMP: Governor Torque Compensator

Adjusts the autothrottle governor torque compensator that determines how fast the governor will adjust the base torque reference to compensate for changes in density altitude. If RPM is low or high by more than 2-5 RPM, increase this setting by 1% at a time until the governor speed matches your RPM setting. Setting the compensator too high can result in surging and throttle "hunting". Do not make large adjustments at one time

- Range: 0 70

- Units: %

- Increment: 0.1

## H_RSC_GOV_DROOP: Governor Droop Compensator

AutoThrottle governor droop response under load, normal settings of 0-50%. Higher value is quicker response to large speed changes due to load but may cause surging. Adjust this to be as aggressive as possible without getting surging or RPM over-run when the governor responds to large load changes on the rotor system

- Range: 0 100

- Units: %

- Increment: 0.1

## H_RSC_GOV_FF: Governor Feedforward

Feedforward governor gain to throttle response during sudden loading/unloading of the rotor system. If RPM drops excessively during full collective climb with the droop response set correctly, increase the governor feedforward.

- Range: 0 100

- Units: %

- Increment: 0.1

## H_RSC_GOV_RPM: Rotor RPM Setting

Main rotor RPM that governor maintains when engaged

- Range: 800 3500

- Units: RPM

- Increment: 10

## H_RSC_GOV_TORQUE: Governor Torque Limiter

Adjusts the engine's percentage of torque rise on autothrottle during ramp-up to governor speed. The torque rise will determine how fast the rotor speed will ramp up when rotor speed reaches 50% of the rotor RPM setting. The sequence of events engaging the governor is as follows: Throttle ramp time will engage the clutch and start the main rotor turning. The collective should be at flat pitch and the throttle curve set to provide at least 50% of normal RPM at flat pitch. The autothrottle torque limiter will automatically activate and start accelerating the main rotor. If the autothrottle consistently fails to accelerate the main rotor during ramp-in due to engine tune or other factors, then increase the torque limiter setting. NOTE: throttle ramp time and throttle curve should be tuned using RSC_MODE Throttle Curve before using RSC_MODE AutoThrottle

- Range: 10 60

- Units: %

- Increment: 1

# IM Parameters

## IM_ACRO_COL_EXP: Acro Mode Collective Expo

*Note: This parameter is for advanced users*

Used to soften collective pitch inputs near center point in Acro mode.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|0.1|Very Low|
|0.2|Low|
|0.3|Medium|
|0.4|High|
|0.5|Very High|

## IM_STB_COL_1: Stabilize Collective Low

Helicopter's minimum collective pitch setting at zero collective stick input in Stabilize mode.  Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.

- Range: 0 100

- Units: %

- Increment: 1

## IM_STB_COL_2: Stabilize Collective Mid-Low

Helicopter's collective pitch setting at mid-low (40%) collective stick input in Stabilize mode. Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.

- Range: 0 100

- Units: %

- Increment: 1

## IM_STB_COL_3: Stabilize Collective Mid-High

Helicopter's collective pitch setting at mid-high (60%) collective stick input in Stabilize mode. Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.

- Range: 0 100

- Units: %

- Increment: 1

## IM_STB_COL_4: Stabilize Collective High

Helicopter's maximum collective pitch setting at full collective stick input in Stabilize mode. Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.

- Range: 0 100

- Units: %

- Increment: 1

# INS Parameters

## INS_GYROFFS_X: Gyro offsets of X axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYROFFS_Y: Gyro offsets of Y axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYROFFS_Z: Gyro offsets of Z axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYR2OFFS_X: Gyro2 offsets of X axis

*Note: This parameter is for advanced users*

Gyro2 sensor offsets of X axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYR2OFFS_Y: Gyro2 offsets of Y axis

*Note: This parameter is for advanced users*

Gyro2 sensor offsets of Y axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYR2OFFS_Z: Gyro2 offsets of Z axis

*Note: This parameter is for advanced users*

Gyro2 sensor offsets of Z axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYR3OFFS_X: Gyro3 offsets of X axis

*Note: This parameter is for advanced users*

Gyro3 sensor offsets of X axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYR3OFFS_Y: Gyro3 offsets of Y axis

*Note: This parameter is for advanced users*

Gyro3 sensor offsets of Y axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_GYR3OFFS_Z: Gyro3 offsets of Z axis

*Note: This parameter is for advanced users*

Gyro3 sensor offsets of Z axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS_ACCSCAL_X: Accelerometer scaling of X axis

*Note: This parameter is for advanced users*

Accelerometer scaling of X axis.  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACCSCAL_Y: Accelerometer scaling of Y axis

*Note: This parameter is for advanced users*

Accelerometer scaling of Y axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACCSCAL_Z: Accelerometer scaling of Z axis

*Note: This parameter is for advanced users*

Accelerometer scaling of Z axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACCOFFS_X: Accelerometer offsets of X axis

*Note: This parameter is for advanced users*

Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACCOFFS_Y: Accelerometer offsets of Y axis

*Note: This parameter is for advanced users*

Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACCOFFS_Z: Accelerometer offsets of Z axis

*Note: This parameter is for advanced users*

Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACC2SCAL_X: Accelerometer2 scaling of X axis

*Note: This parameter is for advanced users*

Accelerometer2 scaling of X axis.  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACC2SCAL_Y: Accelerometer2 scaling of Y axis

*Note: This parameter is for advanced users*

Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACC2SCAL_Z: Accelerometer2 scaling of Z axis

*Note: This parameter is for advanced users*

Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACC2OFFS_X: Accelerometer2 offsets of X axis

*Note: This parameter is for advanced users*

Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACC2OFFS_Y: Accelerometer2 offsets of Y axis

*Note: This parameter is for advanced users*

Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACC2OFFS_Z: Accelerometer2 offsets of Z axis

*Note: This parameter is for advanced users*

Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACC3SCAL_X: Accelerometer3 scaling of X axis

*Note: This parameter is for advanced users*

Accelerometer3 scaling of X axis.  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACC3SCAL_Y: Accelerometer3 scaling of Y axis

*Note: This parameter is for advanced users*

Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACC3SCAL_Z: Accelerometer3 scaling of Z axis

*Note: This parameter is for advanced users*

Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS_ACC3OFFS_X: Accelerometer3 offsets of X axis

*Note: This parameter is for advanced users*

Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACC3OFFS_Y: Accelerometer3 offsets of Y axis

*Note: This parameter is for advanced users*

Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_ACC3OFFS_Z: Accelerometer3 offsets of Z axis

*Note: This parameter is for advanced users*

Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS_GYRO_FILTER: Gyro filter cutoff frequency

*Note: This parameter is for advanced users*

Filter cutoff frequency for gyroscopes. This can be set to a lower value to try to cope with very high vibration levels in aircraft. A value of zero means no filtering (not recommended!)

- Units: Hz

- Range: 0 256

## INS_ACCEL_FILTER: Accel filter cutoff frequency

*Note: This parameter is for advanced users*

Filter cutoff frequency for accelerometers. This can be set to a lower value to try to cope with very high vibration levels in aircraft. A value of zero means no filtering (not recommended!)

- Units: Hz

- Range: 0 256

## INS_USE: Use first IMU for attitude, velocity and position estimates

*Note: This parameter is for advanced users*

Use first IMU for attitude, velocity and position estimates

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_USE2: Use second IMU for attitude, velocity and position estimates

*Note: This parameter is for advanced users*

Use second IMU for attitude, velocity and position estimates

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_USE3: Use third IMU for attitude, velocity and position estimates

*Note: This parameter is for advanced users*

Use third IMU for attitude, velocity and position estimates

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_STILL_THRESH: Stillness threshold for detecting if we are moving

*Note: This parameter is for advanced users*

Threshold to tolerate vibration to determine if vehicle is motionless. This depends on the frame type and if there is a constant vibration due to motors before launch or after landing. Total motionless is about 0.05. Suggested values: Planes/rover use 0.1, multirotors use 1, tradHeli uses 5

- Range: 0.05 50

## INS_GYR_CAL: Gyro Calibration scheme

*Note: This parameter is for advanced users*

Conrols when automatic gyro calibration is performed

|Value|Meaning|
|:---:|:---:|
|0|Never|
|1|Start-up only|

## INS_TRIM_OPTION: Accel cal trim option

*Note: This parameter is for advanced users*

Specifies how the accel cal routine determines the trims

|Value|Meaning|
|:---:|:---:|
|0|Don't adjust the trims|
|1|Assume first orientation was level|
|2|Assume ACC_BODYFIX is perfectly aligned to the vehicle|

## INS_ACC_BODYFIX: Body-fixed accelerometer

*Note: This parameter is for advanced users*

The body-fixed accelerometer to be used for trim calculation

|Value|Meaning|
|:---:|:---:|
|1|IMU 1|
|2|IMU 2|
|3|IMU 3|

## INS_POS1_X: IMU accelerometer X position

*Note: This parameter is for advanced users*

X position of the first IMU Accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_POS1_Y: IMU accelerometer Y position

*Note: This parameter is for advanced users*

Y position of the first IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_POS1_Z: IMU accelerometer Z position

*Note: This parameter is for advanced users*

Z position of the first IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_POS2_X: IMU accelerometer X position

*Note: This parameter is for advanced users*

X position of the second IMU accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_POS2_Y: IMU accelerometer Y position

*Note: This parameter is for advanced users*

Y position of the second IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_POS2_Z: IMU accelerometer Z position

*Note: This parameter is for advanced users*

Z position of the second IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_POS3_X: IMU accelerometer X position

*Note: This parameter is for advanced users*

X position of the third IMU accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -10 10

## INS_POS3_Y: IMU accelerometer Y position

*Note: This parameter is for advanced users*

Y position of the third IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_POS3_Z: IMU accelerometer Z position

*Note: This parameter is for advanced users*

Z position of the third IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS_GYR_ID: Gyro ID

*Note: This parameter is for advanced users*

Gyro sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS_GYR2_ID: Gyro2 ID

*Note: This parameter is for advanced users*

Gyro2 sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS_GYR3_ID: Gyro3 ID

*Note: This parameter is for advanced users*

Gyro3 sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS_ACC_ID: Accelerometer ID

*Note: This parameter is for advanced users*

Accelerometer sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS_ACC2_ID: Accelerometer2 ID

*Note: This parameter is for advanced users*

Accelerometer2 sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS_ACC3_ID: Accelerometer3 ID

*Note: This parameter is for advanced users*

Accelerometer3 sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS_FAST_SAMPLE: Fast sampling mask

*Note: This parameter is for advanced users*

Mask of IMUs to enable fast sampling on, if available

- Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU

## INS_ENABLE_MASK: IMU enable mask

*Note: This parameter is for advanced users*

Bitmask of IMUs to enable. It can be used to prevent startup of specific detected IMUs

- Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU,3:FourthIMU,4:FifthIMU,5:SixthIMU,6:SeventhIMU

## INS_GYRO_RATE: Gyro rate for IMUs with Fast Sampling enabled

*Note: This parameter is for advanced users*

Gyro rate for IMUs with fast sampling enabled. The gyro rate is the sample rate at which the IMU filters operate and needs to be at least double the maximum filter frequency. If the sensor does not support the selected rate the next highest supported rate will be used. For IMUs which do not support fast sampling this setting is ignored and the default gyro rate of 1Khz is used.

|Value|Meaning|
|:---:|:---:|
|0|1kHz|
|1|2kHz|
|2|4kHz|
|3|8kHz|

- RebootRequired: True

## INS_ACC1_CALTEMP: Calibration temperature for 1st accelerometer

*Note: This parameter is for advanced users*

Temperature that the 1st accelerometer was calibrated at

- Units: degC

- Calibration: 1

## INS_GYR1_CALTEMP: Calibration temperature for 1st gyroscope

*Note: This parameter is for advanced users*

Temperature that the 1st gyroscope was calibrated at

- Units: degC

- Calibration: 1

## INS_ACC2_CALTEMP: Calibration temperature for 2nd accelerometer

*Note: This parameter is for advanced users*

Temperature that the 2nd accelerometer was calibrated at

- Units: degC

- Calibration: 1

## INS_GYR2_CALTEMP: Calibration temperature for 2nd gyroscope

*Note: This parameter is for advanced users*

Temperature that the 2nd gyroscope was calibrated at

- Units: degC

- Calibration: 1

## INS_ACC3_CALTEMP: Calibration temperature for 3rd accelerometer

*Note: This parameter is for advanced users*

Temperature that the 3rd accelerometer was calibrated at

- Units: degC

- Calibration: 1

## INS_GYR3_CALTEMP: Calibration temperature for 3rd gyroscope

*Note: This parameter is for advanced users*

Temperature that the 3rd gyroscope was calibrated at

- Units: degC

- Calibration: 1

## INS_TCAL_OPTIONS: Options for temperature calibration

*Note: This parameter is for advanced users*

This enables optional temperature calibration features. Setting PersistParams will save the accelerometer and temperature calibration parameters in the bootloader sector on the next update of the bootloader.

- Bitmask: 0:PersistParams

# INSHNTC2 Parameters

## INS_HNTC2_ENABLE: Harmonic Notch Filter enable

*Note: This parameter is for advanced users*

Harmonic Notch Filter enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_HNTC2_FREQ: Harmonic Notch Filter base frequency

*Note: This parameter is for advanced users*

Harmonic Notch Filter base center frequency in Hz. This should be set at most half the backend gyro rate (which is typically 1Khz). For helicopters using RPM sensor to dynamically set the notch frequency, use this parameter to provide a lower limit to the dynamic notch filter.  Recommend setting it to half the operating rotor speed in Hz.

- Range: 10 495

- Units: Hz

## INS_HNTC2_BW: Harmonic Notch Filter bandwidth

*Note: This parameter is for advanced users*

Harmonic Notch Filter bandwidth in Hz. This is typically set to half the base frequency. The ratio of base frequency to bandwidth determines the notch quality factor and is fixed across harmonics.

- Range: 5 250

- Units: Hz

## INS_HNTC2_ATT: Harmonic Notch Filter attenuation

*Note: This parameter is for advanced users*

Harmonic Notch Filter attenuation in dB. Values greater than 40dB will typically produce a hard notch rather than a modest attenuation of motor noise.

- Range: 5 50

- Units: dB

## INS_HNTC2_HMNCS: Harmonic Notch Filter harmonics

*Note: This parameter is for advanced users*

Bitmask of harmonic frequencies to apply Harmonic Notch Filter to. This option takes effect on the next reboot. A value of 0 disables this filter. The first harmonic refers to the base frequency.

- Bitmask: 0:1st harmonic,1:2nd harmonic,2:3rd harmonic,3:4th hamronic,4:5th harmonic,5:6th harmonic,6:7th harmonic,7:8th harmonic

- RebootRequired: True

## INS_HNTC2_REF: Harmonic Notch Filter reference value

*Note: This parameter is for advanced users*

A reference value of zero disables dynamic updates on the Harmonic Notch Filter and a positive value enables dynamic updates on the Harmonic Notch Filter.  For throttle-based scaling, this parameter is the reference value associated with the specified frequency to facilitate frequency scaling of the Harmonic Notch Filter. For RPM and ESC telemetry based tracking, this parameter is set to 1 to enable the Harmonic Notch Filter using the RPM sensor or ESC telemetry set to measure rotor speed.  The sensor data is converted to Hz automatically for use in the Harmonic Notch Filter.  This reference value may also be used to scale the sensor data, if required.  For example, rpm sensor data is required to measure heli motor RPM. Therefore the reference value can be used to scale the RPM sensor to the rotor RPM.

- Range: 0.0 1.0

- RebootRequired: True

## INS_HNTC2_MODE: Harmonic Notch Filter dynamic frequency tracking mode

*Note: This parameter is for advanced users*

Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based updates should only be used with multicopters.

- Range: 0 4

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Throttle|
|2|RPM Sensor|
|3|ESC Telemetry|
|4|Dynamic FFT|
|5|Second RPM Sensor|

## INS_HNTC2_OPTS: Harmonic Notch Filter options

*Note: This parameter is for advanced users*

Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Dynamic harmonics attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.

- Bitmask: 0:Double notch,1:Dynamic harmonic,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch

- RebootRequired: True

## INS_HNTC2_FM_RAT: Throttle notch min freqency ratio

*Note: This parameter is for advanced users*

The minimum ratio below the configured frequency to take throttle based notch filters when flying at a throttle level below the reference throttle. Note that lower frequency notch filters will have more phase lag. If you want throttle based notch filtering to be effective at a throttle up to 30% below the configured notch frequency then set this parameter to 0.7. The default of 1.0 means the notch will not go below the frequency in the FREQ parameter.

- Range: 0.1 1.0

# INSHNTCH Parameters

## INS_HNTCH_ENABLE: Harmonic Notch Filter enable

*Note: This parameter is for advanced users*

Harmonic Notch Filter enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_HNTCH_FREQ: Harmonic Notch Filter base frequency

*Note: This parameter is for advanced users*

Harmonic Notch Filter base center frequency in Hz. This should be set at most half the backend gyro rate (which is typically 1Khz). For helicopters using RPM sensor to dynamically set the notch frequency, use this parameter to provide a lower limit to the dynamic notch filter.  Recommend setting it to half the operating rotor speed in Hz.

- Range: 10 495

- Units: Hz

## INS_HNTCH_BW: Harmonic Notch Filter bandwidth

*Note: This parameter is for advanced users*

Harmonic Notch Filter bandwidth in Hz. This is typically set to half the base frequency. The ratio of base frequency to bandwidth determines the notch quality factor and is fixed across harmonics.

- Range: 5 250

- Units: Hz

## INS_HNTCH_ATT: Harmonic Notch Filter attenuation

*Note: This parameter is for advanced users*

Harmonic Notch Filter attenuation in dB. Values greater than 40dB will typically produce a hard notch rather than a modest attenuation of motor noise.

- Range: 5 50

- Units: dB

## INS_HNTCH_HMNCS: Harmonic Notch Filter harmonics

*Note: This parameter is for advanced users*

Bitmask of harmonic frequencies to apply Harmonic Notch Filter to. This option takes effect on the next reboot. A value of 0 disables this filter. The first harmonic refers to the base frequency.

- Bitmask: 0:1st harmonic,1:2nd harmonic,2:3rd harmonic,3:4th hamronic,4:5th harmonic,5:6th harmonic,6:7th harmonic,7:8th harmonic

- RebootRequired: True

## INS_HNTCH_REF: Harmonic Notch Filter reference value

*Note: This parameter is for advanced users*

A reference value of zero disables dynamic updates on the Harmonic Notch Filter and a positive value enables dynamic updates on the Harmonic Notch Filter.  For throttle-based scaling, this parameter is the reference value associated with the specified frequency to facilitate frequency scaling of the Harmonic Notch Filter. For RPM and ESC telemetry based tracking, this parameter is set to 1 to enable the Harmonic Notch Filter using the RPM sensor or ESC telemetry set to measure rotor speed.  The sensor data is converted to Hz automatically for use in the Harmonic Notch Filter.  This reference value may also be used to scale the sensor data, if required.  For example, rpm sensor data is required to measure heli motor RPM. Therefore the reference value can be used to scale the RPM sensor to the rotor RPM.

- Range: 0.0 1.0

- RebootRequired: True

## INS_HNTCH_MODE: Harmonic Notch Filter dynamic frequency tracking mode

*Note: This parameter is for advanced users*

Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based updates should only be used with multicopters.

- Range: 0 4

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Throttle|
|2|RPM Sensor|
|3|ESC Telemetry|
|4|Dynamic FFT|
|5|Second RPM Sensor|

## INS_HNTCH_OPTS: Harmonic Notch Filter options

*Note: This parameter is for advanced users*

Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Dynamic harmonics attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.

- Bitmask: 0:Double notch,1:Dynamic harmonic,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch

- RebootRequired: True

## INS_HNTCH_FM_RAT: Throttle notch min freqency ratio

*Note: This parameter is for advanced users*

The minimum ratio below the configured frequency to take throttle based notch filters when flying at a throttle level below the reference throttle. Note that lower frequency notch filters will have more phase lag. If you want throttle based notch filtering to be effective at a throttle up to 30% below the configured notch frequency then set this parameter to 0.7. The default of 1.0 means the notch will not go below the frequency in the FREQ parameter.

- Range: 0.1 1.0

# INSLOG Parameters

## INS_LOG_BAT_CNT: sample count per batch

*Note: This parameter is for advanced users*

Number of samples to take when logging streams of IMU sensor readings.  Will be rounded down to a multiple of 32. This option takes effect on the next reboot.

- Increment: 32

- RebootRequired: True

## INS_LOG_BAT_MASK: Sensor Bitmask

*Note: This parameter is for advanced users*

Bitmap of which IMUs to log batch data for. This option takes effect on the next reboot.

- Bitmask: 0:IMU1,1:IMU2,2:IMU3

- RebootRequired: True

## INS_LOG_BAT_OPT: Batch Logging Options Mask

*Note: This parameter is for advanced users*

Options for the BatchSampler.

- Bitmask: 0:Sensor-Rate Logging (sample at full sensor rate seen by AP), 1: Sample post-filtering, 2: Sample pre- and post-filter

## INS_LOG_BAT_LGIN: logging interval

Interval between pushing samples to the AP_Logger log

- Units: ms

- Increment: 10

## INS_LOG_BAT_LGCT: logging count

Number of samples to push to count every INS_LOG_BAT_LGIN

- Increment: 1

# INSTCAL1 Parameters

## INS_TCAL1_ENABLE: Enable temperature calibration

*Note: This parameter is for advanced users*

Enable the use of temperature calibration parameters for this IMU. For automatic learning set to 2 and also set the INS_TCALn_TMAX to the target temperature, then reboot

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|LearnCalibration|

- RebootRequired: True

## INS_TCAL1_TMIN: Temperature calibration min

*Note: This parameter is for advanced users*

The minimum temperature that the calibration is valid for

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCAL1_TMAX: Temperature calibration max

*Note: This parameter is for advanced users*

The maximum temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCAL1_ACC1_X: Accelerometer 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC1_Y: Accelerometer 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC1_Z: Accelerometer 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC2_X: Accelerometer 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC2_Y: Accelerometer 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC2_Z: Accelerometer 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC3_X: Accelerometer 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC3_Y: Accelerometer 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_ACC3_Z: Accelerometer 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR1_X: Gyroscope 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR1_Y: Gyroscope 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR1_Z: Gyroscope 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR2_X: Gyroscope 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR2_Y: Gyroscope 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR2_Z: Gyroscope 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR3_X: Gyroscope 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR3_Y: Gyroscope 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL1_GYR3_Z: Gyroscope 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

# INSTCAL2 Parameters

## INS_TCAL2_ENABLE: Enable temperature calibration

*Note: This parameter is for advanced users*

Enable the use of temperature calibration parameters for this IMU. For automatic learning set to 2 and also set the INS_TCALn_TMAX to the target temperature, then reboot

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|LearnCalibration|

- RebootRequired: True

## INS_TCAL2_TMIN: Temperature calibration min

*Note: This parameter is for advanced users*

The minimum temperature that the calibration is valid for

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCAL2_TMAX: Temperature calibration max

*Note: This parameter is for advanced users*

The maximum temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCAL2_ACC1_X: Accelerometer 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC1_Y: Accelerometer 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC1_Z: Accelerometer 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC2_X: Accelerometer 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC2_Y: Accelerometer 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC2_Z: Accelerometer 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC3_X: Accelerometer 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC3_Y: Accelerometer 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_ACC3_Z: Accelerometer 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR1_X: Gyroscope 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR1_Y: Gyroscope 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR1_Z: Gyroscope 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR2_X: Gyroscope 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR2_Y: Gyroscope 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR2_Z: Gyroscope 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR3_X: Gyroscope 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR3_Y: Gyroscope 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL2_GYR3_Z: Gyroscope 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

# INSTCAL3 Parameters

## INS_TCAL3_ENABLE: Enable temperature calibration

*Note: This parameter is for advanced users*

Enable the use of temperature calibration parameters for this IMU. For automatic learning set to 2 and also set the INS_TCALn_TMAX to the target temperature, then reboot

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|LearnCalibration|

- RebootRequired: True

## INS_TCAL3_TMIN: Temperature calibration min

*Note: This parameter is for advanced users*

The minimum temperature that the calibration is valid for

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCAL3_TMAX: Temperature calibration max

*Note: This parameter is for advanced users*

The maximum temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCAL3_ACC1_X: Accelerometer 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC1_Y: Accelerometer 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC1_Z: Accelerometer 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC2_X: Accelerometer 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC2_Y: Accelerometer 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC2_Z: Accelerometer 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC3_X: Accelerometer 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC3_Y: Accelerometer 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_ACC3_Z: Accelerometer 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR1_X: Gyroscope 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR1_Y: Gyroscope 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR1_Z: Gyroscope 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR2_X: Gyroscope 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR2_Y: Gyroscope 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR2_Z: Gyroscope 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR3_X: Gyroscope 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR3_Y: Gyroscope 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCAL3_GYR3_Z: Gyroscope 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

# LGR Parameters

## LGR_ENABLE: Enable landing gear

Enable landing gear control

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## LGR_STARTUP: Landing Gear Startup position

Landing Gear Startup behaviour control

|Value|Meaning|
|:---:|:---:|
|0|WaitForPilotInput|
|1|Retract|
|2|Deploy|

## LGR_DEPLOY_PIN: Chassis deployment feedback pin

Pin number to use for detection of gear deployment. If set to -1 feedback is disabled. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|

- RebootRequired: True

## LGR_DEPLOY_POL: Chassis deployment feedback pin polarity

Polarity for feedback pin. If this is 1 then the pin should be high when gear are deployed. If set to 0 then then deployed gear level is low.

|Value|Meaning|
|:---:|:---:|
|0|Low|
|1|High|

## LGR_WOW_PIN: Weight on wheels feedback pin

Pin number to use for feedback of weight on wheels condition. If set to -1 feedback is disabled. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|

- RebootRequired: True

## LGR_WOW_POL: Weight on wheels feedback pin polarity

Polarity for feedback pin. If this is 1 then the pin should be high when there is weight on wheels. If set to 0 then then weight on wheels level is low.

|Value|Meaning|
|:---:|:---:|
|0|Low|
|1|High|

## LGR_DEPLOY_ALT: Landing gear deployment altitude

Altitude where the landing gear will be deployed. This should be lower than the RETRACT_ALT. If zero then altitude is not used for deploying landing gear. Only applies when vehicle is armed.

- Units: m

- Range: 0 1000

- Increment: 1

## LGR_RETRACT_ALT: Landing gear retract altitude

Altitude where the landing gear will be retracted. This should be higher than the DEPLOY_ALT. If zero then altitude is not used for retracting landing gear. Only applies when vehicle is armed.

- Units: m

- Range: 0 1000

- Increment: 1

## LGR_OPTIONS: Landing gear auto retract/deploy options

Options to retract or deploy landing gear in Auto or Guided mode

- Bitmask: 0:Retract after Takeoff,1:Deploy during Land

# LOG Parameters

## LOG_BACKEND_TYPE: AP_Logger Backend Storage type

Bitmap of what Logger backend types to enable. Block-based logging is available on SITL and boards with dataflash chips. Multiple backends can be selected.

- Bitmask: 0:File,1:MAVLink,2:Block

## LOG_FILE_BUFSIZE: Maximum AP_Logger File and Block Backend buffer size (in kilobytes)

The File and Block backends use a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.

## LOG_DISARMED: Enable logging while disarmed

If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## LOG_REPLAY: Enable logging of information needed for Replay

If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost

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

This sets the maximum rate that streaming log messages will be logged to the mavlink backend. A value of zero means that rate limiting is disabled.

- Units: Hz

- Range: 0 1000

- Increment: 0.1

# LOIT Parameters

## LOIT_ANG_MAX: Loiter pilot angle max

*Note: This parameter is for advanced users*

Loiter maximum pilot requested lean angle. Set to zero for 2/3 of PSC_ANGLE_MAX/ANGLE_MAX. The maximum vehicle lean angle is still limited by PSC_ANGLE_MAX/ANGLE_MAX

- Units: deg

- Range: 0 45

- Increment: 1

## LOIT_SPEED: Loiter Horizontal Maximum Speed

Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode

- Units: cm/s

- Range: 20 3500

- Increment: 50

## LOIT_ACC_MAX: Loiter maximum correction acceleration

*Note: This parameter is for advanced users*

Loiter maximum correction acceleration in cm/s/s.  Higher values cause the copter to correct position errors more aggressively.

- Units: cm/s/s

- Range: 100 981

- Increment: 1

## LOIT_BRK_ACCEL: Loiter braking acceleration

*Note: This parameter is for advanced users*

Loiter braking acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered.

- Units: cm/s/s

- Range: 25 250

- Increment: 1

## LOIT_BRK_JERK: Loiter braking jerk

*Note: This parameter is for advanced users*

Loiter braking jerk in cm/s/s/s. Higher values will remove braking faster if the pilot moves the sticks during a braking maneuver.

- Units: cm/s/s/s

- Range: 500 5000

- Increment: 1

## LOIT_BRK_DELAY: Loiter brake start delay (in seconds)

*Note: This parameter is for advanced users*

Loiter brake start delay (in seconds)

- Units: s

- Range: 0 2

- Increment: 0.1

# MIS Parameters

## MIS_TOTAL: Total mission commands

*Note: This parameter is for advanced users*

The number of mission mission items that has been loaded by the ground station. Do not change this manually.

- Range: 0 32766

- Increment: 1

- ReadOnly: True

## MIS_RESTART: Mission Restart when entering Auto mode

*Note: This parameter is for advanced users*

Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)

|Value|Meaning|
|:---:|:---:|
|0|Resume Mission|
|1|Restart Mission|

## MIS_OPTIONS: Mission options bitmask

*Note: This parameter is for advanced users*

Bitmask of what options to use in missions.

- Bitmask: 0:Clear Mission on reboot, 2:ContinueAfterLand

# MNT1 Parameters

## MNT1_TYPE: Mount Type

Mount Type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Servo|
|2|3DR Solo|
|3|Alexmos Serial|
|4|SToRM32 MAVLink|
|5|SToRM32 Serial|
|6|Gremsy|
|7|BrushlessPWM|
|8|Siyi|

- RebootRequired: True

## MNT1_DEFLT_MODE: Mount default operating mode

Mount default operating mode on startup and after control is returned from autopilot

|Value|Meaning|
|:---:|:---:|
|0|Retracted|
|1|Neutral|
|2|MavLink Targeting|
|3|RC Targeting|
|4|GPS Point|
|6|Home Location|

## MNT1_RC_RATE: Mount RC Rate

Pilot rate control's maximum rate.  Set to zero to use angle control

- Units: deg/s

- Range: 0 90

- Increment: 1

## MNT1_ROLL_MIN: Mount Roll angle minimum

Mount Roll angle minimum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT1_ROLL_MAX: Mount Roll angle maximum

Mount Roll angle maximum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT1_PITCH_MIN: Mount Pitch angle minimum

Mount Pitch angle minimum

- Units: deg

- Range: -90 90

- Increment: 1

## MNT1_PITCH_MAX: Mount Pitch angle maximum

Mount Pitch angle maximum

- Units: deg

- Range: -90 90

- Increment: 1

## MNT1_YAW_MIN: Mount Yaw angle minimum

Mount Yaw angle minimum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT1_YAW_MAX: Mount Yaw angle maximum

Mount Yaw angle maximum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT1_RETRACT_X: Mount roll angle when in retracted position

Mount roll angle when in retracted position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT1_RETRACT_Y: Mount pitch angle when in retracted position

Mount pitch angle when in retracted position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT1_RETRACT_Z: Mount yaw angle when in retracted position

Mount yaw angle when in retracted position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT1_NEUTRAL_X: Mount roll angle when in neutral position

Mount roll angle when in neutral position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT1_NEUTRAL_Y: Mount pitch angle when in neutral position

Mount pitch angle when in neutral position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT1_NEUTRAL_Z: Mount yaw angle when in neutral position

Mount yaw angle when in neutral position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT1_LEAD_RLL: Mount Roll stabilization lead time

Servo mount roll angle output leads the vehicle angle by this amount of time based on current roll rate. Increase until the servo is responsive but does not overshoot

- Units: s

- Range: 0.0 0.2

- Increment: .005

## MNT1_LEAD_PTCH: Mount Pitch stabilization lead time

Servo mount pitch angle output leads the vehicle angle by this amount of time based on current pitch rate. Increase until the servo is responsive but does not overshoot

- Units: s

- Range: 0.0 0.2

- Increment: .005

# MNT2 Parameters

## MNT2_TYPE: Mount Type

Mount Type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Servo|
|2|3DR Solo|
|3|Alexmos Serial|
|4|SToRM32 MAVLink|
|5|SToRM32 Serial|
|6|Gremsy|
|7|BrushlessPWM|
|8|Siyi|

- RebootRequired: True

## MNT2_DEFLT_MODE: Mount default operating mode

Mount default operating mode on startup and after control is returned from autopilot

|Value|Meaning|
|:---:|:---:|
|0|Retracted|
|1|Neutral|
|2|MavLink Targeting|
|3|RC Targeting|
|4|GPS Point|
|6|Home Location|

## MNT2_RC_RATE: Mount RC Rate

Pilot rate control's maximum rate.  Set to zero to use angle control

- Units: deg/s

- Range: 0 90

- Increment: 1

## MNT2_ROLL_MIN: Mount Roll angle minimum

Mount Roll angle minimum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT2_ROLL_MAX: Mount Roll angle maximum

Mount Roll angle maximum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT2_PITCH_MIN: Mount Pitch angle minimum

Mount Pitch angle minimum

- Units: deg

- Range: -90 90

- Increment: 1

## MNT2_PITCH_MAX: Mount Pitch angle maximum

Mount Pitch angle maximum

- Units: deg

- Range: -90 90

- Increment: 1

## MNT2_YAW_MIN: Mount Yaw angle minimum

Mount Yaw angle minimum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT2_YAW_MAX: Mount Yaw angle maximum

Mount Yaw angle maximum

- Units: deg

- Range: -180 180

- Increment: 1

## MNT2_RETRACT_X: Mount roll angle when in retracted position

Mount roll angle when in retracted position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT2_RETRACT_Y: Mount pitch angle when in retracted position

Mount pitch angle when in retracted position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT2_RETRACT_Z: Mount yaw angle when in retracted position

Mount yaw angle when in retracted position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT2_NEUTRAL_X: Mount roll angle when in neutral position

Mount roll angle when in neutral position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT2_NEUTRAL_Y: Mount pitch angle when in neutral position

Mount pitch angle when in neutral position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT2_NEUTRAL_Z: Mount yaw angle when in neutral position

Mount yaw angle when in neutral position

- Units: deg

- Range: -180.0 180.0

- Increment: 1

## MNT2_LEAD_RLL: Mount Roll stabilization lead time

Servo mount roll angle output leads the vehicle angle by this amount of time based on current roll rate. Increase until the servo is responsive but does not overshoot

- Units: s

- Range: 0.0 0.2

- Increment: .005

## MNT2_LEAD_PTCH: Mount Pitch stabilization lead time

Servo mount pitch angle output leads the vehicle angle by this amount of time based on current pitch rate. Increase until the servo is responsive but does not overshoot

- Units: s

- Range: 0.0 0.2

- Increment: .005

# MOT Parameters

## MOT_YAW_HEADROOM: Matrix Yaw Min

*Note: This parameter is for advanced users*

Yaw control is given at least this pwm in microseconds range

- Range: 0 500

- Units: PWM

## MOT_THST_EXPO: Thrust Curve Expo

*Note: This parameter is for advanced users*

Motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)

- Range: -1.0 1.0

## MOT_SPIN_MAX: Motor Spin maximum

*Note: This parameter is for advanced users*

Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range

|Value|Meaning|
|:---:|:---:|
|0.9|Low|
|0.95|Default|
|1.0|High|

## MOT_BAT_VOLT_MAX: Battery voltage compensation maximum voltage

*Note: This parameter is for advanced users*

Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.2 * cell count, 0 = Disabled

- Range: 6 53

- Units: V

## MOT_BAT_VOLT_MIN: Battery voltage compensation minimum voltage

*Note: This parameter is for advanced users*

Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.3 * cell count, 0 = Disabled

- Range: 6 42

- Units: V

## MOT_BAT_CURR_MAX: Motor Current Max

*Note: This parameter is for advanced users*

Maximum current over which maximum throttle is limited (0 = Disabled)

- Range: 0 200

- Units: A

## MOT_PWM_TYPE: Output PWM type

*Note: This parameter is for advanced users*

This selects the output PWM type, allowing for normal PWM continuous output, OneShot, brushed or DShot motor output

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|OneShot|
|2|OneShot125|
|3|Brushed|
|4|DShot150|
|5|DShot300|
|6|DShot600|
|7|DShot1200|
|8|PWMRange|

- RebootRequired: True

## MOT_PWM_MIN: PWM output minimum

*Note: This parameter is for advanced users*

This sets the min PWM output value in microseconds that will ever be output to the motors

- Units: PWM

- Range: 0 2000

## MOT_PWM_MAX: PWM output maximum

*Note: This parameter is for advanced users*

This sets the max PWM value in microseconds that will ever be output to the motors

- Units: PWM

- Range: 0 2000

## MOT_SPIN_MIN: Motor Spin minimum

*Note: This parameter is for advanced users*

Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.  Should be higher than MOT_SPIN_ARM.

|Value|Meaning|
|:---:|:---:|
|0.0|Low|
|0.15|Default|
|0.25|High|

## MOT_SPIN_ARM: Motor Spin armed

*Note: This parameter is for advanced users*

Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range.  Should be lower than MOT_SPIN_MIN.

|Value|Meaning|
|:---:|:---:|
|0.0|Low|
|0.1|Default|
|0.2|High|

## MOT_BAT_CURR_TC: Motor Current Max Time Constant

*Note: This parameter is for advanced users*

Time constant used to limit the maximum current

- Range: 0 10

- Units: s

## MOT_THST_HOVER: Thrust Hover Value

*Note: This parameter is for advanced users*

Motor thrust needed to hover expressed as a number from 0 to 1

- Range: 0.2 0.8

## MOT_HOVER_LEARN: Hover Value Learning

*Note: This parameter is for advanced users*

Enable/Disable automatic learning of hover throttle

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Learn|
|2|Learn and Save|

## MOT_SAFE_DISARM: Motor PWM output disabled when disarmed

*Note: This parameter is for advanced users*

Disables motor PWM output when disarmed

|Value|Meaning|
|:---:|:---:|
|0|PWM enabled while disarmed|
|1|PWM disabled while disarmed|

## MOT_YAW_SV_ANGLE: Yaw Servo Max Lean Angle

Yaw servo's maximum lean angle (Tricopter only)

- Range: 5 80

- Units: deg

- Increment: 1

## MOT_SPOOL_TIME: Spool up time

*Note: This parameter is for advanced users*

Time in seconds to spool up the motors from zero to min throttle. 

- Range: 0 2

- Units: s

- Increment: 0.1

## MOT_BOOST_SCALE: Motor boost scale

*Note: This parameter is for advanced users*

Booster motor output scaling factor vs main throttle.  The output to the BoostThrottle servo will be the main throttle times this scaling factor. A higher scaling factor will put more of the load on the booster motor. A value of 1 will set the BoostThrottle equal to the main throttle.

- Range: 0 5

- Increment: 0.1

## MOT_BAT_IDX: Battery compensation index

*Note: This parameter is for advanced users*

Which battery monitor should be used for doing compensation

|Value|Meaning|
|:---:|:---:|
|0|First battery|
|1|Second battery|

## MOT_SLEW_UP_TIME: Output slew time for increasing throttle

*Note: This parameter is for advanced users*

Time in seconds to slew output from zero to full. This is used to limit the rate at which output can change. Range is constrained between 0 and 0.5.

- Range: 0 .5

- Units: s

- Increment: 0.001

## MOT_SLEW_DN_TIME: Output slew time for decreasing throttle

*Note: This parameter is for advanced users*

Time in seconds to slew output from full to zero. This is used to limit the rate at which output can change.  Range is constrained between 0 and 0.5.

- Range: 0 .5

- Units: s

- Increment: 0.001

## MOT_SAFE_TIME: Time taken to disable and enable the motor PWM output when disarmed and armed.

*Note: This parameter is for advanced users*

Time taken to disable and enable the motor PWM output when disarmed and armed.

- Range: 0 5

- Units: s

- Increment: 0.001

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
|0|Disabled|

## NTF_LED_TYPES: LED Driver Types

*Note: This parameter is for advanced users*

Controls what types of LEDs will be enabled

- Bitmask: 0:Built-in LED, 1:Internal ToshibaLED, 2:External ToshibaLED, 3:External PCA9685, 4:Oreo LED, 5:DroneCAN, 6:NCP5623 External, 7:NCP5623 Internal, 8:NeoPixel, 9:ProfiLED, 10:Scripting, 11:DShot, 12:ProfiLED_SPI

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

# OA Parameters

## OA_TYPE: Object Avoidance Path Planning algorithm to use

Enabled/disable path planning around obstacles

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|BendyRuler|
|2|Dijkstra|
|3|Dijkstra with BendyRuler|

## OA_MARGIN_MAX: Object Avoidance wide margin distance

Object Avoidance will ignore objects more than this many meters from vehicle

- Units: m

- Range: 0.1 100

- Increment: 1

## OA_OPTIONS: Options while recovering from Object Avoidance

Bitmask which will govern vehicles behaviour while recovering from Obstacle Avoidance (i.e Avoidance is turned off after the path ahead is clear).   

- Bitmask: 1: log Dijkstra points

# OABR Parameters

## OA_BR_LOOKAHEAD: Object Avoidance look ahead distance maximum

Object Avoidance will look this many meters ahead of vehicle

- Units: m

- Range: 1 100

- Increment: 1

## OA_BR_CONT_RATIO: Obstacle Avoidance margin ratio for BendyRuler to change bearing significantly 

 BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is atleast this much.

- Range: 1.1 2

- Increment: 0.1

## OA_BR_CONT_ANGLE: BendyRuler's bearing change resistance threshold angle   

 BendyRuler will resist changing current bearing if the change in bearing is over this angle

- Range: 20 180

- Increment: 5

## OA_BR_TYPE: Type of BendyRuler

BendyRuler will search for clear path along the direction defined by this parameter

|Value|Meaning|
|:---:|:---:|
|1|Horizontal search|
|2|Vertical search|

# OADB Parameters

## OA_DB_SIZE: OADatabase maximum number of points

*Note: This parameter is for advanced users*

OADatabase maximum number of points. Set to 0 to disable the OA Database. Larger means more points but is more cpu intensive to process

- Range: 0 10000

- RebootRequired: True

## OA_DB_EXPIRE: OADatabase item timeout

*Note: This parameter is for advanced users*

OADatabase item timeout. The time an item will linger without any updates before it expires. Zero means never expires which is useful for a sent-once static environment but terrible for dynamic ones.

- Units: s

- Range: 0 127

- Increment: 1

## OA_DB_QUEUE_SIZE: OADatabase queue maximum number of points

*Note: This parameter is for advanced users*

OADatabase queue maximum number of points. This in an input buffer size. Larger means it can handle larger bursts of incoming data points to filter into the database. No impact on cpu, only RAM. Recommend larger for faster datalinks or for sensors that generate a lot of data.

- Range: 1 200

- RebootRequired: True

## OA_DB_OUTPUT: OADatabase output level

*Note: This parameter is for advanced users*

OADatabase output level to configure which database objects are sent to the ground station. All data is always available internally for avoidance algorithms.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Send only HIGH importance items|
|2|Send HIGH and NORMAL importance items|
|3|Send all items|

## OA_DB_BEAM_WIDTH: OADatabase beam width

*Note: This parameter is for advanced users*

Beam width of incoming lidar data

- Units: deg

- Range: 1 10

- RebootRequired: True

## OA_DB_RADIUS_MIN: OADatabase Minimum  radius

*Note: This parameter is for advanced users*

Minimum radius of objects held in database

- Units: m

- Range: 0 10

## OA_DB_DIST_MAX: OADatabase Distance Maximum

*Note: This parameter is for advanced users*

Maximum distance of objects held in database.  Set to zero to disable the limits

- Units: m

- Range: 0 10

## OA_DB_ALT_MIN: OADatabase minimum altitude above home before storing obstacles

*Note: This parameter is for advanced users*

OADatabase will reject obstacle's if vehicle's altitude above home is below this parameter, in a 3 meter radius around home. Set 0 to disable this feature.

- Units: m

- Range: 0 4

# OSD Parameters

## OSD_TYPE: OSD type

OSD type. TXONLY makes the OSD parameter selection available to other modules even if there is no native OSD support on the board, for instance CRSF.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|MAX7456|
|2|SITL|
|3|MSP|
|4|TXONLY|

- RebootRequired: True

## OSD_CHAN: Screen switch transmitter channel

This sets the channel used to switch different OSD screens.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|5|Chan5|
|6|Chan6|
|7|Chan7|
|8|Chan8|
|9|Chan9|
|10|Chan10|
|11|Chan11|
|12|Chan12|
|13|Chan13|
|14|Chan14|
|15|Chan15|
|16|Chan16|

## OSD_SW_METHOD: Screen switch method

This sets the method used to switch different OSD screens.

|Value|Meaning|
|:---:|:---:|
|0|switch to next screen if channel value was changed|
|1|select screen based on pwm ranges specified for each screen|
|2|switch to next screen after low to high transition and every 1s while channel value is high|

## OSD_OPTIONS: OSD Options

This sets options that change the display

- Bitmask: 0:UseDecimalPack, 1:InvertedWindPointer, 2:InvertedAHRoll, 3:Convert feet to miles at 5280ft instead of 10000ft, 4:DisableCrosshair

## OSD_FONT: OSD Font

This sets which OSD font to use. It is an integer from 0 to the number of fonts available

- RebootRequired: True

## OSD_V_OFFSET: OSD vertical offset

Sets vertical offset of the osd inside image

- Range: 0 31

- RebootRequired: True

## OSD_H_OFFSET: OSD horizontal offset

Sets horizontal offset of the osd inside image

- Range: 0 63

- RebootRequired: True

## OSD_W_RSSI: RSSI warn level (in %)

Set level at which RSSI item will flash

- Range: 0 99

## OSD_W_NSAT: NSAT warn level

Set level at which NSAT item will flash

- Range: 1 30

## OSD_W_BATVOLT: BAT_VOLT warn level

Set level at which BAT_VOLT item will flash

- Range: 0 100

## OSD_UNITS: Display Units

Sets the units to use in displaying items

|Value|Meaning|
|:---:|:---:|
|0|Metric|
|1|Imperial|
|2|SI|
|3|Aviation|

## OSD_MSG_TIME: Message display duration in seconds

Sets message duration seconds

- Range: 1 20

## OSD_ARM_SCR: Arm screen

Screen to be shown on Arm event. Zero to disable the feature.

- Range: 0 4

## OSD_DSARM_SCR: Disarm screen

Screen to be shown on disarm event. Zero to disable the feature.

- Range: 0 4

## OSD_FS_SCR: Failsafe screen

Screen to be shown on failsafe event. Zero to disable the feature.

- Range: 0 4

## OSD_BTN_DELAY: Button delay

*Note: This parameter is for advanced users*

Debounce time in ms for stick commanded parameter navigation.

- Range: 0 3000

## OSD_W_TERR: Terrain warn level

Set level below which TER_HGT item will flash. -1 disables.

- Range: -1 3000

- Units: m

## OSD_W_AVGCELLV: AVGCELLV warn level

Set level at which AVGCELLV item will flash

- Range: 0 100

## OSD_CELL_COUNT: Battery cell count

*Note: This parameter is for advanced users*

Used for average cell voltage display. -1 disables, 0 uses cell count autodetection for well charged LIPO/LIION batteries at connection, other values manually select cell count used.

- Increment: 1

## OSD_W_RESTVOLT: RESTVOLT warn level

Set level at which RESTVOLT item will flash

- Range: 0 100

# OSD1 Parameters

## OSD1_ENABLE: Enable screen

Enable this screen

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CHAN_MIN: Transmitter switch screen minimum pwm

This sets the PWM lower limit for this screen

- Range: 900 2100

## OSD1_CHAN_MAX: Transmitter switch screen maximum pwm

This sets the PWM upper limit for this screen

- Range: 900 2100

## OSD1_ALTITUDE_EN: ALTITUDE_EN

Enables display of altitude AGL

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ALTITUDE_X: ALTITUDE_X

Horizontal position on screen

- Range: 0 29

## OSD1_ALTITUDE_Y: ALTITUDE_Y

Vertical position on screen

- Range: 0 15

## OSD1_BAT_VOLT_EN: BATVOLT_EN

Displays main battery voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_BAT_VOLT_X: BATVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD1_BAT_VOLT_Y: BATVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD1_RSSI_EN: RSSI_EN

Displays RC signal strength

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_RSSI_X: RSSI_X

Horizontal position on screen

- Range: 0 29

## OSD1_RSSI_Y: RSSI_Y

Vertical position on screen

- Range: 0 15

## OSD1_CURRENT_EN: CURRENT_EN

Displays main battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CURRENT_X: CURRENT_X

Horizontal position on screen

- Range: 0 29

## OSD1_CURRENT_Y: CURRENT_Y

Vertical position on screen

- Range: 0 15

## OSD1_BATUSED_EN: BATUSED_EN

Displays primary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_BATUSED_X: BATUSED_X

Horizontal position on screen

- Range: 0 29

## OSD1_BATUSED_Y: BATUSED_Y

Vertical position on screen

- Range: 0 15

## OSD1_SATS_EN: SATS_EN

Displays number of acquired satellites

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_SATS_X: SATS_X

Horizontal position on screen

- Range: 0 29

## OSD1_SATS_Y: SATS_Y

Vertical position on screen

- Range: 0 15

## OSD1_FLTMODE_EN: FLTMODE_EN

Displays flight mode

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_FLTMODE_X: FLTMODE_X

Horizontal position on screen

- Range: 0 29

## OSD1_FLTMODE_Y: FLTMODE_Y

Vertical position on screen

- Range: 0 15

## OSD1_MESSAGE_EN: MESSAGE_EN

Displays Mavlink messages

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_MESSAGE_X: MESSAGE_X

Horizontal position on screen

- Range: 0 29

## OSD1_MESSAGE_Y: MESSAGE_Y

Vertical position on screen

- Range: 0 15

## OSD1_GSPEED_EN: GSPEED_EN

Displays GPS ground speed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_GSPEED_X: GSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD1_GSPEED_Y: GSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD1_HORIZON_EN: HORIZON_EN

Displays artificial horizon

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_HORIZON_X: HORIZON_X

Horizontal position on screen

- Range: 0 29

## OSD1_HORIZON_Y: HORIZON_Y

Vertical position on screen

- Range: 0 15

## OSD1_HOME_EN: HOME_EN

Displays distance and relative direction to HOME

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_HOME_X: HOME_X

Horizontal position on screen

- Range: 0 29

## OSD1_HOME_Y: HOME_Y

Vertical position on screen

- Range: 0 15

## OSD1_HEADING_EN: HEADING_EN

Displays heading

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_HEADING_X: HEADING_X

Horizontal position on screen

- Range: 0 29

## OSD1_HEADING_Y: HEADING_Y

Vertical position on screen

- Range: 0 15

## OSD1_THROTTLE_EN: THROTTLE_EN

Displays actual throttle percentage being sent to motor(s)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_THROTTLE_X: THROTTLE_X

Horizontal position on screen

- Range: 0 29

## OSD1_THROTTLE_Y: THROTTLE_Y

Vertical position on screen

- Range: 0 15

## OSD1_COMPASS_EN: COMPASS_EN

Enables display of compass rose

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_COMPASS_X: COMPASS_X

Horizontal position on screen

- Range: 0 29

## OSD1_COMPASS_Y: COMPASS_Y

Vertical position on screen

- Range: 0 15

## OSD1_WIND_EN: WIND_EN

Displays wind speed and relative direction, on Rover this is the apparent wind speed and direction from the windvane, if fitted

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_WIND_X: WIND_X

Horizontal position on screen

- Range: 0 29

## OSD1_WIND_Y: WIND_Y

Vertical position on screen

- Range: 0 15

## OSD1_ASPEED_EN: ASPEED_EN

Displays airspeed value being used by TECS (fused value)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ASPEED_X: ASPEED_X

Horizontal position on screen

- Range: 0 29

## OSD1_ASPEED_Y: ASPEED_Y

Vertical position on screen

- Range: 0 15

## OSD1_VSPEED_EN: VSPEED_EN

Displays climb rate

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_VSPEED_X: VSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD1_VSPEED_Y: VSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD1_ESCTEMP_EN: ESCTEMP_EN

Displays first esc's temp

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ESCTEMP_X: ESCTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD1_ESCTEMP_Y: ESCTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD1_ESCRPM_EN: ESCRPM_EN

Displays first esc's rpm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ESCRPM_X: ESCRPM_X

Horizontal position on screen

- Range: 0 29

## OSD1_ESCRPM_Y: ESCRPM_Y

Vertical position on screen

- Range: 0 15

## OSD1_ESCAMPS_EN: ESCAMPS_EN

Displays first esc's current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ESCAMPS_X: ESCAMPS_X

Horizontal position on screen

- Range: 0 29

## OSD1_ESCAMPS_Y: ESCAMPS_Y

Vertical position on screen

- Range: 0 15

## OSD1_GPSLAT_EN: GPSLAT_EN

Displays GPS latitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_GPSLAT_X: GPSLAT_X

Horizontal position on screen

- Range: 0 29

## OSD1_GPSLAT_Y: GPSLAT_Y

Vertical position on screen

- Range: 0 15

## OSD1_GPSLONG_EN: GPSLONG_EN

Displays GPS longitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_GPSLONG_X: GPSLONG_X

Horizontal position on screen

- Range: 0 29

## OSD1_GPSLONG_Y: GPSLONG_Y

Vertical position on screen

- Range: 0 15

## OSD1_ROLL_EN: ROLL_EN

Displays degrees of roll from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ROLL_X: ROLL_X

Horizontal position on screen

- Range: 0 29

## OSD1_ROLL_Y: ROLL_Y

Vertical position on screen

- Range: 0 15

## OSD1_PITCH_EN: PITCH_EN

Displays degrees of pitch from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_PITCH_X: PITCH_X

Horizontal position on screen

- Range: 0 29

## OSD1_PITCH_Y: PITCH_Y

Vertical position on screen

- Range: 0 15

## OSD1_TEMP_EN: TEMP_EN

Displays temperature reported by primary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_TEMP_X: TEMP_X

Horizontal position on screen

- Range: 0 29

## OSD1_TEMP_Y: TEMP_Y

Vertical position on screen

- Range: 0 15

## OSD1_HDOP_EN: HDOP_EN

Displays Horizontal Dilution Of Position

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_HDOP_X: HDOP_X

Horizontal position on screen

- Range: 0 29

## OSD1_HDOP_Y: HDOP_Y

Vertical position on screen

- Range: 0 15

## OSD1_WAYPOINT_EN: WAYPOINT_EN

Displays bearing and distance to next waypoint

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_WAYPOINT_X: WAYPOINT_X

Horizontal position on screen

- Range: 0 29

## OSD1_WAYPOINT_Y: WAYPOINT_Y

Vertical position on screen

- Range: 0 15

## OSD1_XTRACK_EN: XTRACK_EN

Displays crosstrack error

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_XTRACK_X: XTRACK_X

Horizontal position on screen

- Range: 0 29

## OSD1_XTRACK_Y: XTRACK_Y

Vertical position on screen

- Range: 0 15

## OSD1_DIST_EN: DIST_EN

Displays total distance flown

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_DIST_X: DIST_X

Horizontal position on screen

- Range: 0 29

## OSD1_DIST_Y: DIST_Y

Vertical position on screen

- Range: 0 15

## OSD1_STATS_EN: STATS_EN

Displays flight stats

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_STATS_X: STATS_X

Horizontal position on screen

- Range: 0 29

## OSD1_STATS_Y: STATS_Y

Vertical position on screen

- Range: 0 15

## OSD1_FLTIME_EN: FLTIME_EN

Displays total flight time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_FLTIME_X: FLTIME_X

Horizontal position on screen

- Range: 0 29

## OSD1_FLTIME_Y: FLTIME_Y

Vertical position on screen

- Range: 0 15

## OSD1_CLIMBEFF_EN: CLIMBEFF_EN

Displays climb efficiency (climb rate/current)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CLIMBEFF_X: CLIMBEFF_X

Horizontal position on screen

- Range: 0 29

## OSD1_CLIMBEFF_Y: CLIMBEFF_Y

Vertical position on screen

- Range: 0 15

## OSD1_EFF_EN: EFF_EN

Displays flight efficiency (mAh/km or /mi)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_EFF_X: EFF_X

Horizontal position on screen

- Range: 0 29

## OSD1_EFF_Y: EFF_Y

Vertical position on screen

- Range: 0 15

## OSD1_BTEMP_EN: BTEMP_EN

Displays temperature reported by secondary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_BTEMP_X: BTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD1_BTEMP_Y: BTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD1_ATEMP_EN: ATEMP_EN

Displays temperature reported by primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ATEMP_X: ATEMP_X

Horizontal position on screen

- Range: 0 29

## OSD1_ATEMP_Y: ATEMP_Y

Vertical position on screen

- Range: 0 15

## OSD1_BAT2_VLT_EN: BAT2VLT_EN

Displays battery2 voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_BAT2_VLT_X: BAT2VLT_X

Horizontal position on screen

- Range: 0 29

## OSD1_BAT2_VLT_Y: BAT2VLT_Y

Vertical position on screen

- Range: 0 15

## OSD1_BAT2USED_EN: BAT2USED_EN

Displays secondary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_BAT2USED_X: BAT2USED_X

Horizontal position on screen

- Range: 0 29

## OSD1_BAT2USED_Y: BAT2USED_Y

Vertical position on screen

- Range: 0 15

## OSD1_ASPD2_EN: ASPD2_EN

Displays airspeed reported directly from secondary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ASPD2_X: ASPD2_X

Horizontal position on screen

- Range: 0 29

## OSD1_ASPD2_Y: ASPD2_Y

Vertical position on screen

- Range: 0 15

## OSD1_ASPD1_EN: ASPD1_EN

Displays airspeed reported directly from primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ASPD1_X: ASPD1_X

Horizontal position on screen

- Range: 0 29

## OSD1_ASPD1_Y: ASPD1_Y

Vertical position on screen

- Range: 0 15

## OSD1_CLK_EN: CLK_EN

Displays a clock panel based on AP_RTC local time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CLK_X: CLK_X

Horizontal position on screen

- Range: 0 29

## OSD1_CLK_Y: CLK_Y

Vertical position on screen

- Range: 0 15

## OSD1_SIDEBARS_EN: SIDEBARS_EN

Displays artificial horizon side bars

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_SIDEBARS_X: SIDEBARS_X

Horizontal position on screen

- Range: 0 29

## OSD1_SIDEBARS_Y: SIDEBARS_Y

Vertical position on screen

- Range: 0 15

## OSD1_CRSSHAIR_EN: CRSSHAIR_EN

Displays artificial horizon crosshair (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CRSSHAIR_X: CRSSHAIR_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD1_CRSSHAIR_Y: CRSSHAIR_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD1_HOMEDIST_EN: HOMEDIST_EN

Displays distance from HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_HOMEDIST_X: HOMEDIST_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD1_HOMEDIST_Y: HOMEDIST_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD1_HOMEDIR_EN: HOMEDIR_EN

Displays relative direction to HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_HOMEDIR_X: HOMEDIR_X

Horizontal position on screen

- Range: 0 29

## OSD1_HOMEDIR_Y: HOMEDIR_Y

Vertical position on screen

- Range: 0 15

## OSD1_POWER_EN: POWER_EN

Displays power (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_POWER_X: POWER_X

Horizontal position on screen

- Range: 0 29

## OSD1_POWER_Y: POWER_Y

Vertical position on screen

- Range: 0 15

## OSD1_CELLVOLT_EN: CELL_VOLT_EN

Displays average cell voltage (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CELLVOLT_X: CELL_VOLT_X

Horizontal position on screen

- Range: 0 29

## OSD1_CELLVOLT_Y: CELL_VOLT_Y

Vertical position on screen

- Range: 0 15

## OSD1_BATTBAR_EN: BATT_BAR_EN

Displays battery usage bar (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_BATTBAR_X: BATT_BAR_X

Horizontal position on screen

- Range: 0 29

## OSD1_BATTBAR_Y: BATT_BAR_Y

Vertical position on screen

- Range: 0 15

## OSD1_ARMING_EN: ARMING_EN

Displays arming status (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_ARMING_X: ARMING_X

Horizontal position on screen

- Range: 0 29

## OSD1_ARMING_Y: ARMING_Y

Vertical position on screen

- Range: 0 15

## OSD1_PLUSCODE_EN: PLUSCODE_EN

Displays pluscode (OLC) element

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_PLUSCODE_X: PLUSCODE_X

Horizontal position on screen

- Range: 0 29

## OSD1_PLUSCODE_Y: PLUSCODE_Y

Vertical position on screen

- Range: 0 15

## OSD1_CALLSIGN_EN: CALLSIGN_EN

Displays callsign from callsign.txt on microSD card

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CALLSIGN_X: CALLSIGN_X

Horizontal position on screen

- Range: 0 29

## OSD1_CALLSIGN_Y: CALLSIGN_Y

Vertical position on screen

- Range: 0 15

## OSD1_CURRENT2_EN: CURRENT2_EN

Displays 2nd battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_CURRENT2_X: CURRENT2_X

Horizontal position on screen

- Range: 0 29

## OSD1_CURRENT2_Y: CURRENT2_Y

Vertical position on screen

- Range: 0 15

## OSD1_VTX_PWR_EN: VTX_PWR_EN

Displays VTX Power

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_VTX_PWR_X: VTX_PWR_X

Horizontal position on screen

- Range: 0 29

## OSD1_VTX_PWR_Y: VTX_PWR_Y

Vertical position on screen

- Range: 0 15

## OSD1_TER_HGT_EN: TER_HGT_EN

Displays Height above terrain

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_TER_HGT_X: TER_HGT_X

Horizontal position on screen

- Range: 0 29

## OSD1_TER_HGT_Y: TER_HGT_Y

Vertical position on screen

- Range: 0 15

## OSD1_AVGCELLV_EN: AVGCELLV_EN

Displays average cell voltage. WARNING: this can be inaccurate if the cell count is not detected or set properly. If the  the battery is far from fully charged the detected cell count might not be accurate if auto cell count detection is used (OSD_CELL_COUNT=0).

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_AVGCELLV_X: AVGCELLV_X

Horizontal position on screen

- Range: 0 29

## OSD1_AVGCELLV_Y: AVGCELLV_Y

Vertical position on screen

- Range: 0 15

## OSD1_RESTVOLT_EN: RESTVOLT_EN

Displays main battery resting voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_RESTVOLT_X: RESTVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD1_RESTVOLT_Y: RESTVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD1_FENCE_EN: FENCE_EN

Displays indication of fence enable and breach

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_FENCE_X: FENCE_X

Horizontal position on screen

- Range: 0 29

## OSD1_FENCE_Y: FENCE_Y

Vertical position on screen

- Range: 0 15

## OSD1_RNGF_EN: RNGF_EN

Displays a rangefinder's distance in cm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_RNGF_X: RNGF_X

Horizontal position on screen

- Range: 0 29

## OSD1_RNGF_Y: RNGF_Y

Vertical position on screen

- Range: 0 15

## OSD1_LINK_Q_EN: LINK_Q_EN

Displays Receiver link quality

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD1_LINK_Q_X: LINK_Q_X

Horizontal position on screen

- Range: 0 29

## OSD1_LINK_Q_Y: LINK_Q_Y

Vertical position on screen

- Range: 0 15

## OSD1_TXT_RES: Sets the overlay text resolution (MSP DisplayPort only)

Sets the overlay text resolution for this screen to either LD 30x16 or HD 50x18 (MSP DisplayPort only)

|Value|Meaning|
|:---:|:---:|
|0|30x16|
|1|50x18|

## OSD1_FONT: Sets the font index for this screen (MSP DisplayPort only)

Sets the font index for this screen (MSP DisplayPort only)

- Range: 0 15

# OSD2 Parameters

## OSD2_ENABLE: Enable screen

Enable this screen

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CHAN_MIN: Transmitter switch screen minimum pwm

This sets the PWM lower limit for this screen

- Range: 900 2100

## OSD2_CHAN_MAX: Transmitter switch screen maximum pwm

This sets the PWM upper limit for this screen

- Range: 900 2100

## OSD2_ALTITUDE_EN: ALTITUDE_EN

Enables display of altitude AGL

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ALTITUDE_X: ALTITUDE_X

Horizontal position on screen

- Range: 0 29

## OSD2_ALTITUDE_Y: ALTITUDE_Y

Vertical position on screen

- Range: 0 15

## OSD2_BAT_VOLT_EN: BATVOLT_EN

Displays main battery voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_BAT_VOLT_X: BATVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD2_BAT_VOLT_Y: BATVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD2_RSSI_EN: RSSI_EN

Displays RC signal strength

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_RSSI_X: RSSI_X

Horizontal position on screen

- Range: 0 29

## OSD2_RSSI_Y: RSSI_Y

Vertical position on screen

- Range: 0 15

## OSD2_CURRENT_EN: CURRENT_EN

Displays main battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CURRENT_X: CURRENT_X

Horizontal position on screen

- Range: 0 29

## OSD2_CURRENT_Y: CURRENT_Y

Vertical position on screen

- Range: 0 15

## OSD2_BATUSED_EN: BATUSED_EN

Displays primary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_BATUSED_X: BATUSED_X

Horizontal position on screen

- Range: 0 29

## OSD2_BATUSED_Y: BATUSED_Y

Vertical position on screen

- Range: 0 15

## OSD2_SATS_EN: SATS_EN

Displays number of acquired satellites

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_SATS_X: SATS_X

Horizontal position on screen

- Range: 0 29

## OSD2_SATS_Y: SATS_Y

Vertical position on screen

- Range: 0 15

## OSD2_FLTMODE_EN: FLTMODE_EN

Displays flight mode

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_FLTMODE_X: FLTMODE_X

Horizontal position on screen

- Range: 0 29

## OSD2_FLTMODE_Y: FLTMODE_Y

Vertical position on screen

- Range: 0 15

## OSD2_MESSAGE_EN: MESSAGE_EN

Displays Mavlink messages

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_MESSAGE_X: MESSAGE_X

Horizontal position on screen

- Range: 0 29

## OSD2_MESSAGE_Y: MESSAGE_Y

Vertical position on screen

- Range: 0 15

## OSD2_GSPEED_EN: GSPEED_EN

Displays GPS ground speed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_GSPEED_X: GSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD2_GSPEED_Y: GSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD2_HORIZON_EN: HORIZON_EN

Displays artificial horizon

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_HORIZON_X: HORIZON_X

Horizontal position on screen

- Range: 0 29

## OSD2_HORIZON_Y: HORIZON_Y

Vertical position on screen

- Range: 0 15

## OSD2_HOME_EN: HOME_EN

Displays distance and relative direction to HOME

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_HOME_X: HOME_X

Horizontal position on screen

- Range: 0 29

## OSD2_HOME_Y: HOME_Y

Vertical position on screen

- Range: 0 15

## OSD2_HEADING_EN: HEADING_EN

Displays heading

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_HEADING_X: HEADING_X

Horizontal position on screen

- Range: 0 29

## OSD2_HEADING_Y: HEADING_Y

Vertical position on screen

- Range: 0 15

## OSD2_THROTTLE_EN: THROTTLE_EN

Displays actual throttle percentage being sent to motor(s)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_THROTTLE_X: THROTTLE_X

Horizontal position on screen

- Range: 0 29

## OSD2_THROTTLE_Y: THROTTLE_Y

Vertical position on screen

- Range: 0 15

## OSD2_COMPASS_EN: COMPASS_EN

Enables display of compass rose

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_COMPASS_X: COMPASS_X

Horizontal position on screen

- Range: 0 29

## OSD2_COMPASS_Y: COMPASS_Y

Vertical position on screen

- Range: 0 15

## OSD2_WIND_EN: WIND_EN

Displays wind speed and relative direction, on Rover this is the apparent wind speed and direction from the windvane, if fitted

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_WIND_X: WIND_X

Horizontal position on screen

- Range: 0 29

## OSD2_WIND_Y: WIND_Y

Vertical position on screen

- Range: 0 15

## OSD2_ASPEED_EN: ASPEED_EN

Displays airspeed value being used by TECS (fused value)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ASPEED_X: ASPEED_X

Horizontal position on screen

- Range: 0 29

## OSD2_ASPEED_Y: ASPEED_Y

Vertical position on screen

- Range: 0 15

## OSD2_VSPEED_EN: VSPEED_EN

Displays climb rate

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_VSPEED_X: VSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD2_VSPEED_Y: VSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD2_ESCTEMP_EN: ESCTEMP_EN

Displays first esc's temp

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ESCTEMP_X: ESCTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD2_ESCTEMP_Y: ESCTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD2_ESCRPM_EN: ESCRPM_EN

Displays first esc's rpm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ESCRPM_X: ESCRPM_X

Horizontal position on screen

- Range: 0 29

## OSD2_ESCRPM_Y: ESCRPM_Y

Vertical position on screen

- Range: 0 15

## OSD2_ESCAMPS_EN: ESCAMPS_EN

Displays first esc's current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ESCAMPS_X: ESCAMPS_X

Horizontal position on screen

- Range: 0 29

## OSD2_ESCAMPS_Y: ESCAMPS_Y

Vertical position on screen

- Range: 0 15

## OSD2_GPSLAT_EN: GPSLAT_EN

Displays GPS latitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_GPSLAT_X: GPSLAT_X

Horizontal position on screen

- Range: 0 29

## OSD2_GPSLAT_Y: GPSLAT_Y

Vertical position on screen

- Range: 0 15

## OSD2_GPSLONG_EN: GPSLONG_EN

Displays GPS longitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_GPSLONG_X: GPSLONG_X

Horizontal position on screen

- Range: 0 29

## OSD2_GPSLONG_Y: GPSLONG_Y

Vertical position on screen

- Range: 0 15

## OSD2_ROLL_EN: ROLL_EN

Displays degrees of roll from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ROLL_X: ROLL_X

Horizontal position on screen

- Range: 0 29

## OSD2_ROLL_Y: ROLL_Y

Vertical position on screen

- Range: 0 15

## OSD2_PITCH_EN: PITCH_EN

Displays degrees of pitch from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_PITCH_X: PITCH_X

Horizontal position on screen

- Range: 0 29

## OSD2_PITCH_Y: PITCH_Y

Vertical position on screen

- Range: 0 15

## OSD2_TEMP_EN: TEMP_EN

Displays temperature reported by primary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_TEMP_X: TEMP_X

Horizontal position on screen

- Range: 0 29

## OSD2_TEMP_Y: TEMP_Y

Vertical position on screen

- Range: 0 15

## OSD2_HDOP_EN: HDOP_EN

Displays Horizontal Dilution Of Position

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_HDOP_X: HDOP_X

Horizontal position on screen

- Range: 0 29

## OSD2_HDOP_Y: HDOP_Y

Vertical position on screen

- Range: 0 15

## OSD2_WAYPOINT_EN: WAYPOINT_EN

Displays bearing and distance to next waypoint

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_WAYPOINT_X: WAYPOINT_X

Horizontal position on screen

- Range: 0 29

## OSD2_WAYPOINT_Y: WAYPOINT_Y

Vertical position on screen

- Range: 0 15

## OSD2_XTRACK_EN: XTRACK_EN

Displays crosstrack error

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_XTRACK_X: XTRACK_X

Horizontal position on screen

- Range: 0 29

## OSD2_XTRACK_Y: XTRACK_Y

Vertical position on screen

- Range: 0 15

## OSD2_DIST_EN: DIST_EN

Displays total distance flown

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_DIST_X: DIST_X

Horizontal position on screen

- Range: 0 29

## OSD2_DIST_Y: DIST_Y

Vertical position on screen

- Range: 0 15

## OSD2_STATS_EN: STATS_EN

Displays flight stats

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_STATS_X: STATS_X

Horizontal position on screen

- Range: 0 29

## OSD2_STATS_Y: STATS_Y

Vertical position on screen

- Range: 0 15

## OSD2_FLTIME_EN: FLTIME_EN

Displays total flight time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_FLTIME_X: FLTIME_X

Horizontal position on screen

- Range: 0 29

## OSD2_FLTIME_Y: FLTIME_Y

Vertical position on screen

- Range: 0 15

## OSD2_CLIMBEFF_EN: CLIMBEFF_EN

Displays climb efficiency (climb rate/current)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CLIMBEFF_X: CLIMBEFF_X

Horizontal position on screen

- Range: 0 29

## OSD2_CLIMBEFF_Y: CLIMBEFF_Y

Vertical position on screen

- Range: 0 15

## OSD2_EFF_EN: EFF_EN

Displays flight efficiency (mAh/km or /mi)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_EFF_X: EFF_X

Horizontal position on screen

- Range: 0 29

## OSD2_EFF_Y: EFF_Y

Vertical position on screen

- Range: 0 15

## OSD2_BTEMP_EN: BTEMP_EN

Displays temperature reported by secondary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_BTEMP_X: BTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD2_BTEMP_Y: BTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD2_ATEMP_EN: ATEMP_EN

Displays temperature reported by primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ATEMP_X: ATEMP_X

Horizontal position on screen

- Range: 0 29

## OSD2_ATEMP_Y: ATEMP_Y

Vertical position on screen

- Range: 0 15

## OSD2_BAT2_VLT_EN: BAT2VLT_EN

Displays battery2 voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_BAT2_VLT_X: BAT2VLT_X

Horizontal position on screen

- Range: 0 29

## OSD2_BAT2_VLT_Y: BAT2VLT_Y

Vertical position on screen

- Range: 0 15

## OSD2_BAT2USED_EN: BAT2USED_EN

Displays secondary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_BAT2USED_X: BAT2USED_X

Horizontal position on screen

- Range: 0 29

## OSD2_BAT2USED_Y: BAT2USED_Y

Vertical position on screen

- Range: 0 15

## OSD2_ASPD2_EN: ASPD2_EN

Displays airspeed reported directly from secondary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ASPD2_X: ASPD2_X

Horizontal position on screen

- Range: 0 29

## OSD2_ASPD2_Y: ASPD2_Y

Vertical position on screen

- Range: 0 15

## OSD2_ASPD1_EN: ASPD1_EN

Displays airspeed reported directly from primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ASPD1_X: ASPD1_X

Horizontal position on screen

- Range: 0 29

## OSD2_ASPD1_Y: ASPD1_Y

Vertical position on screen

- Range: 0 15

## OSD2_CLK_EN: CLK_EN

Displays a clock panel based on AP_RTC local time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CLK_X: CLK_X

Horizontal position on screen

- Range: 0 29

## OSD2_CLK_Y: CLK_Y

Vertical position on screen

- Range: 0 15

## OSD2_SIDEBARS_EN: SIDEBARS_EN

Displays artificial horizon side bars

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_SIDEBARS_X: SIDEBARS_X

Horizontal position on screen

- Range: 0 29

## OSD2_SIDEBARS_Y: SIDEBARS_Y

Vertical position on screen

- Range: 0 15

## OSD2_CRSSHAIR_EN: CRSSHAIR_EN

Displays artificial horizon crosshair (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CRSSHAIR_X: CRSSHAIR_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD2_CRSSHAIR_Y: CRSSHAIR_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD2_HOMEDIST_EN: HOMEDIST_EN

Displays distance from HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_HOMEDIST_X: HOMEDIST_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD2_HOMEDIST_Y: HOMEDIST_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD2_HOMEDIR_EN: HOMEDIR_EN

Displays relative direction to HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_HOMEDIR_X: HOMEDIR_X

Horizontal position on screen

- Range: 0 29

## OSD2_HOMEDIR_Y: HOMEDIR_Y

Vertical position on screen

- Range: 0 15

## OSD2_POWER_EN: POWER_EN

Displays power (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_POWER_X: POWER_X

Horizontal position on screen

- Range: 0 29

## OSD2_POWER_Y: POWER_Y

Vertical position on screen

- Range: 0 15

## OSD2_CELLVOLT_EN: CELL_VOLT_EN

Displays average cell voltage (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CELLVOLT_X: CELL_VOLT_X

Horizontal position on screen

- Range: 0 29

## OSD2_CELLVOLT_Y: CELL_VOLT_Y

Vertical position on screen

- Range: 0 15

## OSD2_BATTBAR_EN: BATT_BAR_EN

Displays battery usage bar (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_BATTBAR_X: BATT_BAR_X

Horizontal position on screen

- Range: 0 29

## OSD2_BATTBAR_Y: BATT_BAR_Y

Vertical position on screen

- Range: 0 15

## OSD2_ARMING_EN: ARMING_EN

Displays arming status (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_ARMING_X: ARMING_X

Horizontal position on screen

- Range: 0 29

## OSD2_ARMING_Y: ARMING_Y

Vertical position on screen

- Range: 0 15

## OSD2_PLUSCODE_EN: PLUSCODE_EN

Displays pluscode (OLC) element

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_PLUSCODE_X: PLUSCODE_X

Horizontal position on screen

- Range: 0 29

## OSD2_PLUSCODE_Y: PLUSCODE_Y

Vertical position on screen

- Range: 0 15

## OSD2_CALLSIGN_EN: CALLSIGN_EN

Displays callsign from callsign.txt on microSD card

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CALLSIGN_X: CALLSIGN_X

Horizontal position on screen

- Range: 0 29

## OSD2_CALLSIGN_Y: CALLSIGN_Y

Vertical position on screen

- Range: 0 15

## OSD2_CURRENT2_EN: CURRENT2_EN

Displays 2nd battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_CURRENT2_X: CURRENT2_X

Horizontal position on screen

- Range: 0 29

## OSD2_CURRENT2_Y: CURRENT2_Y

Vertical position on screen

- Range: 0 15

## OSD2_VTX_PWR_EN: VTX_PWR_EN

Displays VTX Power

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_VTX_PWR_X: VTX_PWR_X

Horizontal position on screen

- Range: 0 29

## OSD2_VTX_PWR_Y: VTX_PWR_Y

Vertical position on screen

- Range: 0 15

## OSD2_TER_HGT_EN: TER_HGT_EN

Displays Height above terrain

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_TER_HGT_X: TER_HGT_X

Horizontal position on screen

- Range: 0 29

## OSD2_TER_HGT_Y: TER_HGT_Y

Vertical position on screen

- Range: 0 15

## OSD2_AVGCELLV_EN: AVGCELLV_EN

Displays average cell voltage. WARNING: this can be inaccurate if the cell count is not detected or set properly. If the  the battery is far from fully charged the detected cell count might not be accurate if auto cell count detection is used (OSD_CELL_COUNT=0).

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_AVGCELLV_X: AVGCELLV_X

Horizontal position on screen

- Range: 0 29

## OSD2_AVGCELLV_Y: AVGCELLV_Y

Vertical position on screen

- Range: 0 15

## OSD2_RESTVOLT_EN: RESTVOLT_EN

Displays main battery resting voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_RESTVOLT_X: RESTVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD2_RESTVOLT_Y: RESTVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD2_FENCE_EN: FENCE_EN

Displays indication of fence enable and breach

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_FENCE_X: FENCE_X

Horizontal position on screen

- Range: 0 29

## OSD2_FENCE_Y: FENCE_Y

Vertical position on screen

- Range: 0 15

## OSD2_RNGF_EN: RNGF_EN

Displays a rangefinder's distance in cm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_RNGF_X: RNGF_X

Horizontal position on screen

- Range: 0 29

## OSD2_RNGF_Y: RNGF_Y

Vertical position on screen

- Range: 0 15

## OSD2_LINK_Q_EN: LINK_Q_EN

Displays Receiver link quality

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD2_LINK_Q_X: LINK_Q_X

Horizontal position on screen

- Range: 0 29

## OSD2_LINK_Q_Y: LINK_Q_Y

Vertical position on screen

- Range: 0 15

## OSD2_TXT_RES: Sets the overlay text resolution (MSP DisplayPort only)

Sets the overlay text resolution for this screen to either LD 30x16 or HD 50x18 (MSP DisplayPort only)

|Value|Meaning|
|:---:|:---:|
|0|30x16|
|1|50x18|

## OSD2_FONT: Sets the font index for this screen (MSP DisplayPort only)

Sets the font index for this screen (MSP DisplayPort only)

- Range: 0 15

# OSD3 Parameters

## OSD3_ENABLE: Enable screen

Enable this screen

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CHAN_MIN: Transmitter switch screen minimum pwm

This sets the PWM lower limit for this screen

- Range: 900 2100

## OSD3_CHAN_MAX: Transmitter switch screen maximum pwm

This sets the PWM upper limit for this screen

- Range: 900 2100

## OSD3_ALTITUDE_EN: ALTITUDE_EN

Enables display of altitude AGL

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ALTITUDE_X: ALTITUDE_X

Horizontal position on screen

- Range: 0 29

## OSD3_ALTITUDE_Y: ALTITUDE_Y

Vertical position on screen

- Range: 0 15

## OSD3_BAT_VOLT_EN: BATVOLT_EN

Displays main battery voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_BAT_VOLT_X: BATVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD3_BAT_VOLT_Y: BATVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD3_RSSI_EN: RSSI_EN

Displays RC signal strength

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_RSSI_X: RSSI_X

Horizontal position on screen

- Range: 0 29

## OSD3_RSSI_Y: RSSI_Y

Vertical position on screen

- Range: 0 15

## OSD3_CURRENT_EN: CURRENT_EN

Displays main battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CURRENT_X: CURRENT_X

Horizontal position on screen

- Range: 0 29

## OSD3_CURRENT_Y: CURRENT_Y

Vertical position on screen

- Range: 0 15

## OSD3_BATUSED_EN: BATUSED_EN

Displays primary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_BATUSED_X: BATUSED_X

Horizontal position on screen

- Range: 0 29

## OSD3_BATUSED_Y: BATUSED_Y

Vertical position on screen

- Range: 0 15

## OSD3_SATS_EN: SATS_EN

Displays number of acquired satellites

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_SATS_X: SATS_X

Horizontal position on screen

- Range: 0 29

## OSD3_SATS_Y: SATS_Y

Vertical position on screen

- Range: 0 15

## OSD3_FLTMODE_EN: FLTMODE_EN

Displays flight mode

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_FLTMODE_X: FLTMODE_X

Horizontal position on screen

- Range: 0 29

## OSD3_FLTMODE_Y: FLTMODE_Y

Vertical position on screen

- Range: 0 15

## OSD3_MESSAGE_EN: MESSAGE_EN

Displays Mavlink messages

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_MESSAGE_X: MESSAGE_X

Horizontal position on screen

- Range: 0 29

## OSD3_MESSAGE_Y: MESSAGE_Y

Vertical position on screen

- Range: 0 15

## OSD3_GSPEED_EN: GSPEED_EN

Displays GPS ground speed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_GSPEED_X: GSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD3_GSPEED_Y: GSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD3_HORIZON_EN: HORIZON_EN

Displays artificial horizon

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_HORIZON_X: HORIZON_X

Horizontal position on screen

- Range: 0 29

## OSD3_HORIZON_Y: HORIZON_Y

Vertical position on screen

- Range: 0 15

## OSD3_HOME_EN: HOME_EN

Displays distance and relative direction to HOME

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_HOME_X: HOME_X

Horizontal position on screen

- Range: 0 29

## OSD3_HOME_Y: HOME_Y

Vertical position on screen

- Range: 0 15

## OSD3_HEADING_EN: HEADING_EN

Displays heading

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_HEADING_X: HEADING_X

Horizontal position on screen

- Range: 0 29

## OSD3_HEADING_Y: HEADING_Y

Vertical position on screen

- Range: 0 15

## OSD3_THROTTLE_EN: THROTTLE_EN

Displays actual throttle percentage being sent to motor(s)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_THROTTLE_X: THROTTLE_X

Horizontal position on screen

- Range: 0 29

## OSD3_THROTTLE_Y: THROTTLE_Y

Vertical position on screen

- Range: 0 15

## OSD3_COMPASS_EN: COMPASS_EN

Enables display of compass rose

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_COMPASS_X: COMPASS_X

Horizontal position on screen

- Range: 0 29

## OSD3_COMPASS_Y: COMPASS_Y

Vertical position on screen

- Range: 0 15

## OSD3_WIND_EN: WIND_EN

Displays wind speed and relative direction, on Rover this is the apparent wind speed and direction from the windvane, if fitted

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_WIND_X: WIND_X

Horizontal position on screen

- Range: 0 29

## OSD3_WIND_Y: WIND_Y

Vertical position on screen

- Range: 0 15

## OSD3_ASPEED_EN: ASPEED_EN

Displays airspeed value being used by TECS (fused value)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ASPEED_X: ASPEED_X

Horizontal position on screen

- Range: 0 29

## OSD3_ASPEED_Y: ASPEED_Y

Vertical position on screen

- Range: 0 15

## OSD3_VSPEED_EN: VSPEED_EN

Displays climb rate

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_VSPEED_X: VSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD3_VSPEED_Y: VSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD3_ESCTEMP_EN: ESCTEMP_EN

Displays first esc's temp

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ESCTEMP_X: ESCTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD3_ESCTEMP_Y: ESCTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD3_ESCRPM_EN: ESCRPM_EN

Displays first esc's rpm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ESCRPM_X: ESCRPM_X

Horizontal position on screen

- Range: 0 29

## OSD3_ESCRPM_Y: ESCRPM_Y

Vertical position on screen

- Range: 0 15

## OSD3_ESCAMPS_EN: ESCAMPS_EN

Displays first esc's current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ESCAMPS_X: ESCAMPS_X

Horizontal position on screen

- Range: 0 29

## OSD3_ESCAMPS_Y: ESCAMPS_Y

Vertical position on screen

- Range: 0 15

## OSD3_GPSLAT_EN: GPSLAT_EN

Displays GPS latitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_GPSLAT_X: GPSLAT_X

Horizontal position on screen

- Range: 0 29

## OSD3_GPSLAT_Y: GPSLAT_Y

Vertical position on screen

- Range: 0 15

## OSD3_GPSLONG_EN: GPSLONG_EN

Displays GPS longitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_GPSLONG_X: GPSLONG_X

Horizontal position on screen

- Range: 0 29

## OSD3_GPSLONG_Y: GPSLONG_Y

Vertical position on screen

- Range: 0 15

## OSD3_ROLL_EN: ROLL_EN

Displays degrees of roll from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ROLL_X: ROLL_X

Horizontal position on screen

- Range: 0 29

## OSD3_ROLL_Y: ROLL_Y

Vertical position on screen

- Range: 0 15

## OSD3_PITCH_EN: PITCH_EN

Displays degrees of pitch from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_PITCH_X: PITCH_X

Horizontal position on screen

- Range: 0 29

## OSD3_PITCH_Y: PITCH_Y

Vertical position on screen

- Range: 0 15

## OSD3_TEMP_EN: TEMP_EN

Displays temperature reported by primary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_TEMP_X: TEMP_X

Horizontal position on screen

- Range: 0 29

## OSD3_TEMP_Y: TEMP_Y

Vertical position on screen

- Range: 0 15

## OSD3_HDOP_EN: HDOP_EN

Displays Horizontal Dilution Of Position

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_HDOP_X: HDOP_X

Horizontal position on screen

- Range: 0 29

## OSD3_HDOP_Y: HDOP_Y

Vertical position on screen

- Range: 0 15

## OSD3_WAYPOINT_EN: WAYPOINT_EN

Displays bearing and distance to next waypoint

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_WAYPOINT_X: WAYPOINT_X

Horizontal position on screen

- Range: 0 29

## OSD3_WAYPOINT_Y: WAYPOINT_Y

Vertical position on screen

- Range: 0 15

## OSD3_XTRACK_EN: XTRACK_EN

Displays crosstrack error

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_XTRACK_X: XTRACK_X

Horizontal position on screen

- Range: 0 29

## OSD3_XTRACK_Y: XTRACK_Y

Vertical position on screen

- Range: 0 15

## OSD3_DIST_EN: DIST_EN

Displays total distance flown

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_DIST_X: DIST_X

Horizontal position on screen

- Range: 0 29

## OSD3_DIST_Y: DIST_Y

Vertical position on screen

- Range: 0 15

## OSD3_STATS_EN: STATS_EN

Displays flight stats

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_STATS_X: STATS_X

Horizontal position on screen

- Range: 0 29

## OSD3_STATS_Y: STATS_Y

Vertical position on screen

- Range: 0 15

## OSD3_FLTIME_EN: FLTIME_EN

Displays total flight time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_FLTIME_X: FLTIME_X

Horizontal position on screen

- Range: 0 29

## OSD3_FLTIME_Y: FLTIME_Y

Vertical position on screen

- Range: 0 15

## OSD3_CLIMBEFF_EN: CLIMBEFF_EN

Displays climb efficiency (climb rate/current)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CLIMBEFF_X: CLIMBEFF_X

Horizontal position on screen

- Range: 0 29

## OSD3_CLIMBEFF_Y: CLIMBEFF_Y

Vertical position on screen

- Range: 0 15

## OSD3_EFF_EN: EFF_EN

Displays flight efficiency (mAh/km or /mi)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_EFF_X: EFF_X

Horizontal position on screen

- Range: 0 29

## OSD3_EFF_Y: EFF_Y

Vertical position on screen

- Range: 0 15

## OSD3_BTEMP_EN: BTEMP_EN

Displays temperature reported by secondary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_BTEMP_X: BTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD3_BTEMP_Y: BTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD3_ATEMP_EN: ATEMP_EN

Displays temperature reported by primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ATEMP_X: ATEMP_X

Horizontal position on screen

- Range: 0 29

## OSD3_ATEMP_Y: ATEMP_Y

Vertical position on screen

- Range: 0 15

## OSD3_BAT2_VLT_EN: BAT2VLT_EN

Displays battery2 voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_BAT2_VLT_X: BAT2VLT_X

Horizontal position on screen

- Range: 0 29

## OSD3_BAT2_VLT_Y: BAT2VLT_Y

Vertical position on screen

- Range: 0 15

## OSD3_BAT2USED_EN: BAT2USED_EN

Displays secondary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_BAT2USED_X: BAT2USED_X

Horizontal position on screen

- Range: 0 29

## OSD3_BAT2USED_Y: BAT2USED_Y

Vertical position on screen

- Range: 0 15

## OSD3_ASPD2_EN: ASPD2_EN

Displays airspeed reported directly from secondary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ASPD2_X: ASPD2_X

Horizontal position on screen

- Range: 0 29

## OSD3_ASPD2_Y: ASPD2_Y

Vertical position on screen

- Range: 0 15

## OSD3_ASPD1_EN: ASPD1_EN

Displays airspeed reported directly from primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ASPD1_X: ASPD1_X

Horizontal position on screen

- Range: 0 29

## OSD3_ASPD1_Y: ASPD1_Y

Vertical position on screen

- Range: 0 15

## OSD3_CLK_EN: CLK_EN

Displays a clock panel based on AP_RTC local time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CLK_X: CLK_X

Horizontal position on screen

- Range: 0 29

## OSD3_CLK_Y: CLK_Y

Vertical position on screen

- Range: 0 15

## OSD3_SIDEBARS_EN: SIDEBARS_EN

Displays artificial horizon side bars

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_SIDEBARS_X: SIDEBARS_X

Horizontal position on screen

- Range: 0 29

## OSD3_SIDEBARS_Y: SIDEBARS_Y

Vertical position on screen

- Range: 0 15

## OSD3_CRSSHAIR_EN: CRSSHAIR_EN

Displays artificial horizon crosshair (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CRSSHAIR_X: CRSSHAIR_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD3_CRSSHAIR_Y: CRSSHAIR_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD3_HOMEDIST_EN: HOMEDIST_EN

Displays distance from HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_HOMEDIST_X: HOMEDIST_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD3_HOMEDIST_Y: HOMEDIST_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD3_HOMEDIR_EN: HOMEDIR_EN

Displays relative direction to HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_HOMEDIR_X: HOMEDIR_X

Horizontal position on screen

- Range: 0 29

## OSD3_HOMEDIR_Y: HOMEDIR_Y

Vertical position on screen

- Range: 0 15

## OSD3_POWER_EN: POWER_EN

Displays power (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_POWER_X: POWER_X

Horizontal position on screen

- Range: 0 29

## OSD3_POWER_Y: POWER_Y

Vertical position on screen

- Range: 0 15

## OSD3_CELLVOLT_EN: CELL_VOLT_EN

Displays average cell voltage (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CELLVOLT_X: CELL_VOLT_X

Horizontal position on screen

- Range: 0 29

## OSD3_CELLVOLT_Y: CELL_VOLT_Y

Vertical position on screen

- Range: 0 15

## OSD3_BATTBAR_EN: BATT_BAR_EN

Displays battery usage bar (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_BATTBAR_X: BATT_BAR_X

Horizontal position on screen

- Range: 0 29

## OSD3_BATTBAR_Y: BATT_BAR_Y

Vertical position on screen

- Range: 0 15

## OSD3_ARMING_EN: ARMING_EN

Displays arming status (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_ARMING_X: ARMING_X

Horizontal position on screen

- Range: 0 29

## OSD3_ARMING_Y: ARMING_Y

Vertical position on screen

- Range: 0 15

## OSD3_PLUSCODE_EN: PLUSCODE_EN

Displays pluscode (OLC) element

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_PLUSCODE_X: PLUSCODE_X

Horizontal position on screen

- Range: 0 29

## OSD3_PLUSCODE_Y: PLUSCODE_Y

Vertical position on screen

- Range: 0 15

## OSD3_CALLSIGN_EN: CALLSIGN_EN

Displays callsign from callsign.txt on microSD card

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CALLSIGN_X: CALLSIGN_X

Horizontal position on screen

- Range: 0 29

## OSD3_CALLSIGN_Y: CALLSIGN_Y

Vertical position on screen

- Range: 0 15

## OSD3_CURRENT2_EN: CURRENT2_EN

Displays 2nd battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_CURRENT2_X: CURRENT2_X

Horizontal position on screen

- Range: 0 29

## OSD3_CURRENT2_Y: CURRENT2_Y

Vertical position on screen

- Range: 0 15

## OSD3_VTX_PWR_EN: VTX_PWR_EN

Displays VTX Power

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_VTX_PWR_X: VTX_PWR_X

Horizontal position on screen

- Range: 0 29

## OSD3_VTX_PWR_Y: VTX_PWR_Y

Vertical position on screen

- Range: 0 15

## OSD3_TER_HGT_EN: TER_HGT_EN

Displays Height above terrain

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_TER_HGT_X: TER_HGT_X

Horizontal position on screen

- Range: 0 29

## OSD3_TER_HGT_Y: TER_HGT_Y

Vertical position on screen

- Range: 0 15

## OSD3_AVGCELLV_EN: AVGCELLV_EN

Displays average cell voltage. WARNING: this can be inaccurate if the cell count is not detected or set properly. If the  the battery is far from fully charged the detected cell count might not be accurate if auto cell count detection is used (OSD_CELL_COUNT=0).

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_AVGCELLV_X: AVGCELLV_X

Horizontal position on screen

- Range: 0 29

## OSD3_AVGCELLV_Y: AVGCELLV_Y

Vertical position on screen

- Range: 0 15

## OSD3_RESTVOLT_EN: RESTVOLT_EN

Displays main battery resting voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_RESTVOLT_X: RESTVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD3_RESTVOLT_Y: RESTVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD3_FENCE_EN: FENCE_EN

Displays indication of fence enable and breach

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_FENCE_X: FENCE_X

Horizontal position on screen

- Range: 0 29

## OSD3_FENCE_Y: FENCE_Y

Vertical position on screen

- Range: 0 15

## OSD3_RNGF_EN: RNGF_EN

Displays a rangefinder's distance in cm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_RNGF_X: RNGF_X

Horizontal position on screen

- Range: 0 29

## OSD3_RNGF_Y: RNGF_Y

Vertical position on screen

- Range: 0 15

## OSD3_LINK_Q_EN: LINK_Q_EN

Displays Receiver link quality

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD3_LINK_Q_X: LINK_Q_X

Horizontal position on screen

- Range: 0 29

## OSD3_LINK_Q_Y: LINK_Q_Y

Vertical position on screen

- Range: 0 15

## OSD3_TXT_RES: Sets the overlay text resolution (MSP DisplayPort only)

Sets the overlay text resolution for this screen to either LD 30x16 or HD 50x18 (MSP DisplayPort only)

|Value|Meaning|
|:---:|:---:|
|0|30x16|
|1|50x18|

## OSD3_FONT: Sets the font index for this screen (MSP DisplayPort only)

Sets the font index for this screen (MSP DisplayPort only)

- Range: 0 15

# OSD4 Parameters

## OSD4_ENABLE: Enable screen

Enable this screen

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CHAN_MIN: Transmitter switch screen minimum pwm

This sets the PWM lower limit for this screen

- Range: 900 2100

## OSD4_CHAN_MAX: Transmitter switch screen maximum pwm

This sets the PWM upper limit for this screen

- Range: 900 2100

## OSD4_ALTITUDE_EN: ALTITUDE_EN

Enables display of altitude AGL

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ALTITUDE_X: ALTITUDE_X

Horizontal position on screen

- Range: 0 29

## OSD4_ALTITUDE_Y: ALTITUDE_Y

Vertical position on screen

- Range: 0 15

## OSD4_BAT_VOLT_EN: BATVOLT_EN

Displays main battery voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_BAT_VOLT_X: BATVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD4_BAT_VOLT_Y: BATVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD4_RSSI_EN: RSSI_EN

Displays RC signal strength

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_RSSI_X: RSSI_X

Horizontal position on screen

- Range: 0 29

## OSD4_RSSI_Y: RSSI_Y

Vertical position on screen

- Range: 0 15

## OSD4_CURRENT_EN: CURRENT_EN

Displays main battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CURRENT_X: CURRENT_X

Horizontal position on screen

- Range: 0 29

## OSD4_CURRENT_Y: CURRENT_Y

Vertical position on screen

- Range: 0 15

## OSD4_BATUSED_EN: BATUSED_EN

Displays primary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_BATUSED_X: BATUSED_X

Horizontal position on screen

- Range: 0 29

## OSD4_BATUSED_Y: BATUSED_Y

Vertical position on screen

- Range: 0 15

## OSD4_SATS_EN: SATS_EN

Displays number of acquired satellites

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_SATS_X: SATS_X

Horizontal position on screen

- Range: 0 29

## OSD4_SATS_Y: SATS_Y

Vertical position on screen

- Range: 0 15

## OSD4_FLTMODE_EN: FLTMODE_EN

Displays flight mode

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_FLTMODE_X: FLTMODE_X

Horizontal position on screen

- Range: 0 29

## OSD4_FLTMODE_Y: FLTMODE_Y

Vertical position on screen

- Range: 0 15

## OSD4_MESSAGE_EN: MESSAGE_EN

Displays Mavlink messages

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_MESSAGE_X: MESSAGE_X

Horizontal position on screen

- Range: 0 29

## OSD4_MESSAGE_Y: MESSAGE_Y

Vertical position on screen

- Range: 0 15

## OSD4_GSPEED_EN: GSPEED_EN

Displays GPS ground speed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_GSPEED_X: GSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD4_GSPEED_Y: GSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD4_HORIZON_EN: HORIZON_EN

Displays artificial horizon

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_HORIZON_X: HORIZON_X

Horizontal position on screen

- Range: 0 29

## OSD4_HORIZON_Y: HORIZON_Y

Vertical position on screen

- Range: 0 15

## OSD4_HOME_EN: HOME_EN

Displays distance and relative direction to HOME

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_HOME_X: HOME_X

Horizontal position on screen

- Range: 0 29

## OSD4_HOME_Y: HOME_Y

Vertical position on screen

- Range: 0 15

## OSD4_HEADING_EN: HEADING_EN

Displays heading

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_HEADING_X: HEADING_X

Horizontal position on screen

- Range: 0 29

## OSD4_HEADING_Y: HEADING_Y

Vertical position on screen

- Range: 0 15

## OSD4_THROTTLE_EN: THROTTLE_EN

Displays actual throttle percentage being sent to motor(s)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_THROTTLE_X: THROTTLE_X

Horizontal position on screen

- Range: 0 29

## OSD4_THROTTLE_Y: THROTTLE_Y

Vertical position on screen

- Range: 0 15

## OSD4_COMPASS_EN: COMPASS_EN

Enables display of compass rose

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_COMPASS_X: COMPASS_X

Horizontal position on screen

- Range: 0 29

## OSD4_COMPASS_Y: COMPASS_Y

Vertical position on screen

- Range: 0 15

## OSD4_WIND_EN: WIND_EN

Displays wind speed and relative direction, on Rover this is the apparent wind speed and direction from the windvane, if fitted

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_WIND_X: WIND_X

Horizontal position on screen

- Range: 0 29

## OSD4_WIND_Y: WIND_Y

Vertical position on screen

- Range: 0 15

## OSD4_ASPEED_EN: ASPEED_EN

Displays airspeed value being used by TECS (fused value)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ASPEED_X: ASPEED_X

Horizontal position on screen

- Range: 0 29

## OSD4_ASPEED_Y: ASPEED_Y

Vertical position on screen

- Range: 0 15

## OSD4_VSPEED_EN: VSPEED_EN

Displays climb rate

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_VSPEED_X: VSPEED_X

Horizontal position on screen

- Range: 0 29

## OSD4_VSPEED_Y: VSPEED_Y

Vertical position on screen

- Range: 0 15

## OSD4_ESCTEMP_EN: ESCTEMP_EN

Displays first esc's temp

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ESCTEMP_X: ESCTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD4_ESCTEMP_Y: ESCTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD4_ESCRPM_EN: ESCRPM_EN

Displays first esc's rpm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ESCRPM_X: ESCRPM_X

Horizontal position on screen

- Range: 0 29

## OSD4_ESCRPM_Y: ESCRPM_Y

Vertical position on screen

- Range: 0 15

## OSD4_ESCAMPS_EN: ESCAMPS_EN

Displays first esc's current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ESCAMPS_X: ESCAMPS_X

Horizontal position on screen

- Range: 0 29

## OSD4_ESCAMPS_Y: ESCAMPS_Y

Vertical position on screen

- Range: 0 15

## OSD4_GPSLAT_EN: GPSLAT_EN

Displays GPS latitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_GPSLAT_X: GPSLAT_X

Horizontal position on screen

- Range: 0 29

## OSD4_GPSLAT_Y: GPSLAT_Y

Vertical position on screen

- Range: 0 15

## OSD4_GPSLONG_EN: GPSLONG_EN

Displays GPS longitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_GPSLONG_X: GPSLONG_X

Horizontal position on screen

- Range: 0 29

## OSD4_GPSLONG_Y: GPSLONG_Y

Vertical position on screen

- Range: 0 15

## OSD4_ROLL_EN: ROLL_EN

Displays degrees of roll from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ROLL_X: ROLL_X

Horizontal position on screen

- Range: 0 29

## OSD4_ROLL_Y: ROLL_Y

Vertical position on screen

- Range: 0 15

## OSD4_PITCH_EN: PITCH_EN

Displays degrees of pitch from level

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_PITCH_X: PITCH_X

Horizontal position on screen

- Range: 0 29

## OSD4_PITCH_Y: PITCH_Y

Vertical position on screen

- Range: 0 15

## OSD4_TEMP_EN: TEMP_EN

Displays temperature reported by primary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_TEMP_X: TEMP_X

Horizontal position on screen

- Range: 0 29

## OSD4_TEMP_Y: TEMP_Y

Vertical position on screen

- Range: 0 15

## OSD4_HDOP_EN: HDOP_EN

Displays Horizontal Dilution Of Position

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_HDOP_X: HDOP_X

Horizontal position on screen

- Range: 0 29

## OSD4_HDOP_Y: HDOP_Y

Vertical position on screen

- Range: 0 15

## OSD4_WAYPOINT_EN: WAYPOINT_EN

Displays bearing and distance to next waypoint

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_WAYPOINT_X: WAYPOINT_X

Horizontal position on screen

- Range: 0 29

## OSD4_WAYPOINT_Y: WAYPOINT_Y

Vertical position on screen

- Range: 0 15

## OSD4_XTRACK_EN: XTRACK_EN

Displays crosstrack error

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_XTRACK_X: XTRACK_X

Horizontal position on screen

- Range: 0 29

## OSD4_XTRACK_Y: XTRACK_Y

Vertical position on screen

- Range: 0 15

## OSD4_DIST_EN: DIST_EN

Displays total distance flown

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_DIST_X: DIST_X

Horizontal position on screen

- Range: 0 29

## OSD4_DIST_Y: DIST_Y

Vertical position on screen

- Range: 0 15

## OSD4_STATS_EN: STATS_EN

Displays flight stats

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_STATS_X: STATS_X

Horizontal position on screen

- Range: 0 29

## OSD4_STATS_Y: STATS_Y

Vertical position on screen

- Range: 0 15

## OSD4_FLTIME_EN: FLTIME_EN

Displays total flight time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_FLTIME_X: FLTIME_X

Horizontal position on screen

- Range: 0 29

## OSD4_FLTIME_Y: FLTIME_Y

Vertical position on screen

- Range: 0 15

## OSD4_CLIMBEFF_EN: CLIMBEFF_EN

Displays climb efficiency (climb rate/current)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CLIMBEFF_X: CLIMBEFF_X

Horizontal position on screen

- Range: 0 29

## OSD4_CLIMBEFF_Y: CLIMBEFF_Y

Vertical position on screen

- Range: 0 15

## OSD4_EFF_EN: EFF_EN

Displays flight efficiency (mAh/km or /mi)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_EFF_X: EFF_X

Horizontal position on screen

- Range: 0 29

## OSD4_EFF_Y: EFF_Y

Vertical position on screen

- Range: 0 15

## OSD4_BTEMP_EN: BTEMP_EN

Displays temperature reported by secondary barometer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_BTEMP_X: BTEMP_X

Horizontal position on screen

- Range: 0 29

## OSD4_BTEMP_Y: BTEMP_Y

Vertical position on screen

- Range: 0 15

## OSD4_ATEMP_EN: ATEMP_EN

Displays temperature reported by primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ATEMP_X: ATEMP_X

Horizontal position on screen

- Range: 0 29

## OSD4_ATEMP_Y: ATEMP_Y

Vertical position on screen

- Range: 0 15

## OSD4_BAT2_VLT_EN: BAT2VLT_EN

Displays battery2 voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_BAT2_VLT_X: BAT2VLT_X

Horizontal position on screen

- Range: 0 29

## OSD4_BAT2_VLT_Y: BAT2VLT_Y

Vertical position on screen

- Range: 0 15

## OSD4_BAT2USED_EN: BAT2USED_EN

Displays secondary battery mAh consumed

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_BAT2USED_X: BAT2USED_X

Horizontal position on screen

- Range: 0 29

## OSD4_BAT2USED_Y: BAT2USED_Y

Vertical position on screen

- Range: 0 15

## OSD4_ASPD2_EN: ASPD2_EN

Displays airspeed reported directly from secondary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ASPD2_X: ASPD2_X

Horizontal position on screen

- Range: 0 29

## OSD4_ASPD2_Y: ASPD2_Y

Vertical position on screen

- Range: 0 15

## OSD4_ASPD1_EN: ASPD1_EN

Displays airspeed reported directly from primary airspeed sensor

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ASPD1_X: ASPD1_X

Horizontal position on screen

- Range: 0 29

## OSD4_ASPD1_Y: ASPD1_Y

Vertical position on screen

- Range: 0 15

## OSD4_CLK_EN: CLK_EN

Displays a clock panel based on AP_RTC local time

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CLK_X: CLK_X

Horizontal position on screen

- Range: 0 29

## OSD4_CLK_Y: CLK_Y

Vertical position on screen

- Range: 0 15

## OSD4_SIDEBARS_EN: SIDEBARS_EN

Displays artificial horizon side bars

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_SIDEBARS_X: SIDEBARS_X

Horizontal position on screen

- Range: 0 29

## OSD4_SIDEBARS_Y: SIDEBARS_Y

Vertical position on screen

- Range: 0 15

## OSD4_CRSSHAIR_EN: CRSSHAIR_EN

Displays artificial horizon crosshair (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CRSSHAIR_X: CRSSHAIR_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD4_CRSSHAIR_Y: CRSSHAIR_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD4_HOMEDIST_EN: HOMEDIST_EN

Displays distance from HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_HOMEDIST_X: HOMEDIST_X

Horizontal position on screen (MSP OSD only)

- Range: 0 29

## OSD4_HOMEDIST_Y: HOMEDIST_Y

Vertical position on screen (MSP OSD only)

- Range: 0 15

## OSD4_HOMEDIR_EN: HOMEDIR_EN

Displays relative direction to HOME (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_HOMEDIR_X: HOMEDIR_X

Horizontal position on screen

- Range: 0 29

## OSD4_HOMEDIR_Y: HOMEDIR_Y

Vertical position on screen

- Range: 0 15

## OSD4_POWER_EN: POWER_EN

Displays power (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_POWER_X: POWER_X

Horizontal position on screen

- Range: 0 29

## OSD4_POWER_Y: POWER_Y

Vertical position on screen

- Range: 0 15

## OSD4_CELLVOLT_EN: CELL_VOLT_EN

Displays average cell voltage (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CELLVOLT_X: CELL_VOLT_X

Horizontal position on screen

- Range: 0 29

## OSD4_CELLVOLT_Y: CELL_VOLT_Y

Vertical position on screen

- Range: 0 15

## OSD4_BATTBAR_EN: BATT_BAR_EN

Displays battery usage bar (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_BATTBAR_X: BATT_BAR_X

Horizontal position on screen

- Range: 0 29

## OSD4_BATTBAR_Y: BATT_BAR_Y

Vertical position on screen

- Range: 0 15

## OSD4_ARMING_EN: ARMING_EN

Displays arming status (MSP OSD only)

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_ARMING_X: ARMING_X

Horizontal position on screen

- Range: 0 29

## OSD4_ARMING_Y: ARMING_Y

Vertical position on screen

- Range: 0 15

## OSD4_PLUSCODE_EN: PLUSCODE_EN

Displays pluscode (OLC) element

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_PLUSCODE_X: PLUSCODE_X

Horizontal position on screen

- Range: 0 29

## OSD4_PLUSCODE_Y: PLUSCODE_Y

Vertical position on screen

- Range: 0 15

## OSD4_CALLSIGN_EN: CALLSIGN_EN

Displays callsign from callsign.txt on microSD card

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CALLSIGN_X: CALLSIGN_X

Horizontal position on screen

- Range: 0 29

## OSD4_CALLSIGN_Y: CALLSIGN_Y

Vertical position on screen

- Range: 0 15

## OSD4_CURRENT2_EN: CURRENT2_EN

Displays 2nd battery current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_CURRENT2_X: CURRENT2_X

Horizontal position on screen

- Range: 0 29

## OSD4_CURRENT2_Y: CURRENT2_Y

Vertical position on screen

- Range: 0 15

## OSD4_VTX_PWR_EN: VTX_PWR_EN

Displays VTX Power

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_VTX_PWR_X: VTX_PWR_X

Horizontal position on screen

- Range: 0 29

## OSD4_VTX_PWR_Y: VTX_PWR_Y

Vertical position on screen

- Range: 0 15

## OSD4_TER_HGT_EN: TER_HGT_EN

Displays Height above terrain

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_TER_HGT_X: TER_HGT_X

Horizontal position on screen

- Range: 0 29

## OSD4_TER_HGT_Y: TER_HGT_Y

Vertical position on screen

- Range: 0 15

## OSD4_AVGCELLV_EN: AVGCELLV_EN

Displays average cell voltage. WARNING: this can be inaccurate if the cell count is not detected or set properly. If the  the battery is far from fully charged the detected cell count might not be accurate if auto cell count detection is used (OSD_CELL_COUNT=0).

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_AVGCELLV_X: AVGCELLV_X

Horizontal position on screen

- Range: 0 29

## OSD4_AVGCELLV_Y: AVGCELLV_Y

Vertical position on screen

- Range: 0 15

## OSD4_RESTVOLT_EN: RESTVOLT_EN

Displays main battery resting voltage

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_RESTVOLT_X: RESTVOLT_X

Horizontal position on screen

- Range: 0 29

## OSD4_RESTVOLT_Y: RESTVOLT_Y

Vertical position on screen

- Range: 0 15

## OSD4_FENCE_EN: FENCE_EN

Displays indication of fence enable and breach

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_FENCE_X: FENCE_X

Horizontal position on screen

- Range: 0 29

## OSD4_FENCE_Y: FENCE_Y

Vertical position on screen

- Range: 0 15

## OSD4_RNGF_EN: RNGF_EN

Displays a rangefinder's distance in cm

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_RNGF_X: RNGF_X

Horizontal position on screen

- Range: 0 29

## OSD4_RNGF_Y: RNGF_Y

Vertical position on screen

- Range: 0 15

## OSD4_LINK_Q_EN: LINK_Q_EN

Displays Receiver link quality

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD4_LINK_Q_X: LINK_Q_X

Horizontal position on screen

- Range: 0 29

## OSD4_LINK_Q_Y: LINK_Q_Y

Vertical position on screen

- Range: 0 15

## OSD4_TXT_RES: Sets the overlay text resolution (MSP DisplayPort only)

Sets the overlay text resolution for this screen to either LD 30x16 or HD 50x18 (MSP DisplayPort only)

|Value|Meaning|
|:---:|:---:|
|0|30x16|
|1|50x18|

## OSD4_FONT: Sets the font index for this screen (MSP DisplayPort only)

Sets the font index for this screen (MSP DisplayPort only)

- Range: 0 15

# OSD5 Parameters

## OSD5_ENABLE: Enable screen

Enable this screen

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_CHAN_MIN: Transmitter switch screen minimum pwm

This sets the PWM lower limit for this screen

- Range: 900 2100

## OSD5_CHAN_MAX: Transmitter switch screen maximum pwm

This sets the PWM upper limit for this screen

- Range: 900 2100

## OSD5_SAVE_X: SAVE_X

*Note: This parameter is for advanced users*

Horizontal position of Save button on screen

- Range: 0 25

## OSD5_SAVE_Y: SAVE_Y

*Note: This parameter is for advanced users*

Vertical position of Save button on screen

- Range: 0 15

# OSD5PARAM1 Parameters

## OSD5_PARAM1_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM1_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM1_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM1_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM1_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM1_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM1_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM1_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM1_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM1_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM2 Parameters

## OSD5_PARAM2_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM2_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM2_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM2_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM2_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM2_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM2_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM2_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM2_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM2_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM3 Parameters

## OSD5_PARAM3_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM3_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM3_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM3_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM3_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM3_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM3_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM3_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM3_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM3_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM4 Parameters

## OSD5_PARAM4_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM4_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM4_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM4_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM4_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM4_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM4_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM4_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM4_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM4_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM5 Parameters

## OSD5_PARAM5_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM5_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM5_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM5_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM5_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM5_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM5_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM5_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM5_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM5_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM6 Parameters

## OSD5_PARAM6_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM6_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM6_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM6_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM6_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM6_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM6_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM6_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM6_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM6_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM7 Parameters

## OSD5_PARAM7_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM7_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM7_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM7_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM7_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM7_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM7_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM7_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM7_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM7_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM8 Parameters

## OSD5_PARAM8_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM8_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM8_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM8_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM8_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM8_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM8_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM8_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM8_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM8_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD5PARAM9 Parameters

## OSD5_PARAM9_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD5_PARAM9_X: X position

Horizontal position on screen

- Range: 0 29

## OSD5_PARAM9_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD5_PARAM9_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD5_PARAM9_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD5_PARAM9_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD5_PARAM9_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD5_PARAM9_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD5_PARAM9_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD5_PARAM9_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6 Parameters

## OSD6_ENABLE: Enable screen

Enable this screen

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_CHAN_MIN: Transmitter switch screen minimum pwm

This sets the PWM lower limit for this screen

- Range: 900 2100

## OSD6_CHAN_MAX: Transmitter switch screen maximum pwm

This sets the PWM upper limit for this screen

- Range: 900 2100

## OSD6_SAVE_X: SAVE_X

*Note: This parameter is for advanced users*

Horizontal position of Save button on screen

- Range: 0 25

## OSD6_SAVE_Y: SAVE_Y

*Note: This parameter is for advanced users*

Vertical position of Save button on screen

- Range: 0 15

# OSD6PARAM1 Parameters

## OSD6_PARAM1_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM1_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM1_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM1_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM1_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM1_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM1_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM1_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM1_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM1_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM2 Parameters

## OSD6_PARAM2_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM2_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM2_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM2_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM2_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM2_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM2_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM2_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM2_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM2_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM3 Parameters

## OSD6_PARAM3_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM3_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM3_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM3_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM3_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM3_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM3_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM3_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM3_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM3_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM4 Parameters

## OSD6_PARAM4_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM4_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM4_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM4_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM4_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM4_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM4_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM4_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM4_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM4_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM5 Parameters

## OSD6_PARAM5_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM5_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM5_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM5_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM5_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM5_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM5_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM5_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM5_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM5_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM6 Parameters

## OSD6_PARAM6_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM6_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM6_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM6_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM6_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM6_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM6_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM6_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM6_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM6_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM7 Parameters

## OSD6_PARAM7_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM7_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM7_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM7_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM7_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM7_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM7_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM7_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM7_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM7_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM8 Parameters

## OSD6_PARAM8_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM8_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM8_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM8_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM8_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM8_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM8_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM8_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM8_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM8_TYPE: Parameter type

Type of the parameter to be displayed and modified

# OSD6PARAM9 Parameters

## OSD6_PARAM9_EN: Enable

Enable setting

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## OSD6_PARAM9_X: X position

Horizontal position on screen

- Range: 0 29

## OSD6_PARAM9_Y: Y position

Vertical position on screen

- Range: 0 15

## OSD6_PARAM9_KEY: Parameter key

Key of the parameter to be displayed and modified

## OSD6_PARAM9_IDX: Parameter index

Index of the parameter to be displayed and modified

## OSD6_PARAM9_GRP: Parameter group

Group of the parameter to be displayed and modified

## OSD6_PARAM9_MIN: Parameter minimum

Minimum value of the parameter to be displayed and modified

## OSD6_PARAM9_MAX: Parameter maximum

Maximum of the parameter to be displayed and modified

## OSD6_PARAM9_INCR: Parameter increment

Increment of the parameter to be displayed and modified

## OSD6_PARAM9_TYPE: Parameter type

Type of the parameter to be displayed and modified

# PLND Parameters

## PLND_ENABLED: Precision Land enabled/disabled

*Note: This parameter is for advanced users*

Precision Land enabled/disabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## PLND_TYPE: Precision Land Type

*Note: This parameter is for advanced users*

Precision Land Type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|CompanionComputer|
|2|IRLock|
|3|SITL_Gazebo|
|4|SITL|

## PLND_YAW_ALIGN: Sensor yaw alignment

*Note: This parameter is for advanced users*

Yaw angle from body x-axis to sensor x-axis.

- Range: 0 36000

- Increment: 10

- Units: cdeg

## PLND_LAND_OFS_X: Land offset forward

*Note: This parameter is for advanced users*

Desired landing position of the camera forward of the target in vehicle body frame

- Range: -20 20

- Increment: 1

- Units: cm

## PLND_LAND_OFS_Y: Land offset right

*Note: This parameter is for advanced users*

desired landing position of the camera right of the target in vehicle body frame

- Range: -20 20

- Increment: 1

- Units: cm

## PLND_EST_TYPE: Precision Land Estimator Type

*Note: This parameter is for advanced users*

Specifies the estimation method to be used

|Value|Meaning|
|:---:|:---:|
|0|RawSensor|
|1|KalmanFilter|

## PLND_ACC_P_NSE: Kalman Filter Accelerometer Noise

*Note: This parameter is for advanced users*

Kalman Filter Accelerometer Noise, higher values weight the input from the camera more, accels less

- Range: 0.5 5

## PLND_CAM_POS_X: Camera X position offset

*Note: This parameter is for advanced users*

X position of the camera in body frame. Positive X is forward of the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## PLND_CAM_POS_Y: Camera Y position offset

*Note: This parameter is for advanced users*

Y position of the camera in body frame. Positive Y is to the right of the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## PLND_CAM_POS_Z: Camera Z position offset

*Note: This parameter is for advanced users*

Z position of the camera in body frame. Positive Z is down from the origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## PLND_BUS: Sensor Bus

*Note: This parameter is for advanced users*

Precland sensor bus for I2C sensors.

|Value|Meaning|
|:---:|:---:|
|-1|DefaultBus|
|0|InternalI2C|
|1|ExternalI2C|

## PLND_LAG: Precision Landing sensor lag

*Note: This parameter is for advanced users*

Precision Landing sensor lag, to cope with variable landing_target latency

- Range: 0.02 0.250

- Increment: 1

- Units: s

- RebootRequired: True

## PLND_XY_DIST_MAX: Precision Landing maximum distance to target before descending

*Note: This parameter is for advanced users*

The vehicle will not start descending if the landing target is detected and it is further than this many meters away. Set 0 to always descend.

- Range: 0 10

- Units: m

## PLND_STRICT: PrecLand strictness

How strictly should the vehicle land on the target if target is lost

|Value|Meaning|
|:---:|:---:|
|0|Land Vertically (Not strict)|
|1|Retry Landing(Normal Strictness)|
|2|Do not land (just Hover) (Very Strict)|

## PLND_RET_MAX: PrecLand Maximum number of retires for a failed landing

PrecLand Maximum number of retires for a failed landing. Set to zero to disable landing retry.

- Range: 0 10

- Increment: 1

## PLND_TIMEOUT: PrecLand retry timeout

Time for which vehicle continues descend even if target is lost. After this time period, vehicle will attemp a landing retry depending on PLND_STRICT parameter.

- Range: 0 20

- Units: s

## PLND_RET_BEHAVE: PrecLand retry behaviour

Prec Land will do the action selected by this parameter if a retry to a landing is needed

|Value|Meaning|
|:---:|:---:|
|0|Go to the last location where landing target was detected|
|1|Go towards the approximate location of the detected landing target|

## PLND_ALT_MIN: PrecLand minimum alt for retry

Vehicle will continue landing vertically even if target is lost below this height. This needs a rangefinder to work. Set to zero to disable this.

- Range: 0 5

- Units: m

## PLND_ALT_MAX: PrecLand maximum alt for retry

Vehicle will continue landing vertically until this height if target is not found. Below this height if landing target is not found, landing retry/failsafe might be attempted. This needs a rangefinder to work. Set to zero to disable this.

- Range: 0 50

- Units: m

## PLND_OPTIONS: Precision Landing Extra Options

*Note: This parameter is for advanced users*

Precision Landing Extra Options

- Bitmask: 0: Moving Landing Target, 1: Allow Precision Landing after manual reposition 

## PLND_ORIENT: Camera Orientation

*Note: This parameter is for advanced users*

Orientation of camera/sensor on body

|Value|Meaning|
|:---:|:---:|
|0|Forward|
|4|Back|
|25|Down|

- RebootRequired: True

# PRX Parameters

## PRX_IGN_GND: Proximity sensor land detection

Ignore proximity data that is within 1 meter of the ground below the vehicle. This requires a downward facing rangefinder

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

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

# PSC Parameters

## PSC_ACC_XY_FILT: XY Acceleration filter cutoff frequency

*Note: This parameter is for advanced users*

Lower values will slow the response of the navigation controller and reduce twitchiness

- Units: Hz

- Range: 0.5 5

- Increment: 0.1

## PSC_POSZ_P: Position (vertical) controller P gain

Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller

- Range: 1.000 3.000

## PSC_VELZ_P: Velocity (vertical) controller P gain

Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller

- Range: 1.000 8.000

## PSC_VELZ_I: Velocity (vertical) controller I gain

*Note: This parameter is for advanced users*

Velocity (vertical) controller I gain.  Corrects long-term difference in desired velocity to a target acceleration

- Range: 0.02 1.00

- Increment: 0.01

## PSC_VELZ_IMAX: Velocity (vertical) controller I gain maximum

Velocity (vertical) controller I gain maximum.  Constrains the target acceleration that the I gain will output

- Range: 1.000 8.000

## PSC_VELZ_D: Velocity (vertical) controller D gain

*Note: This parameter is for advanced users*

Velocity (vertical) controller D gain.  Corrects short-term changes in velocity

- Range: 0.00 1.00

- Increment: 0.001

## PSC_VELZ_FF: Velocity (vertical) controller Feed Forward gain

*Note: This parameter is for advanced users*

Velocity (vertical) controller Feed Forward gain.  Produces an output that is proportional to the magnitude of the target

- Range: 0 1

- Increment: 0.01

## PSC_VELZ_FLTE: Velocity (vertical) error filter

*Note: This parameter is for advanced users*

Velocity (vertical) error filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## PSC_VELZ_FLTD: Velocity (vertical) input filter for D term

*Note: This parameter is for advanced users*

Velocity (vertical) input filter for D term.  This filter (in Hz) is applied to the input for D terms

- Range: 0 100

- Units: Hz

## PSC_ACCZ_P: Acceleration (vertical) controller P gain

Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output

- Range: 0.200 1.500

- Increment: 0.05

## PSC_ACCZ_I: Acceleration (vertical) controller I gain

Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration

- Range: 0.000 3.000

## PSC_ACCZ_IMAX: Acceleration (vertical) controller I gain maximum

Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate

- Range: 0 1000

- Units: d%

## PSC_ACCZ_D: Acceleration (vertical) controller D gain

Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration

- Range: 0.000 0.400

## PSC_ACCZ_FF: Acceleration (vertical) controller feed forward

Acceleration (vertical) controller feed forward

- Range: 0 0.5

- Increment: 0.001

## PSC_ACCZ_FLTT: Acceleration (vertical) controller target frequency in Hz

Acceleration (vertical) controller target frequency in Hz

- Range: 1 50

- Increment: 1

- Units: Hz

## PSC_ACCZ_FLTE: Acceleration (vertical) controller error frequency in Hz

Acceleration (vertical) controller error frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## PSC_ACCZ_FLTD: Acceleration (vertical) controller derivative frequency in Hz

Acceleration (vertical) controller derivative frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## PSC_ACCZ_SMAX: Accel (vertical) slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## PSC_POSXY_P: Position (horizontal) controller P gain

Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller

- Range: 0.500 2.000

## PSC_VELXY_P: Velocity (horizontal) P gain

*Note: This parameter is for advanced users*

Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration

- Range: 0.1 6.0

- Increment: 0.1

## PSC_VELXY_I: Velocity (horizontal) I gain

*Note: This parameter is for advanced users*

Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration

- Range: 0.02 1.00

- Increment: 0.01

## PSC_VELXY_D: Velocity (horizontal) D gain

*Note: This parameter is for advanced users*

Velocity (horizontal) D gain.  Corrects short-term changes in velocity

- Range: 0.00 1.00

- Increment: 0.001

## PSC_VELXY_IMAX: Velocity (horizontal) integrator maximum

*Note: This parameter is for advanced users*

Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output

- Range: 0 4500

- Increment: 10

- Units: cm/s/s

## PSC_VELXY_FLTE: Velocity (horizontal) input filter

*Note: This parameter is for advanced users*

Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## PSC_VELXY_FLTD: Velocity (horizontal) input filter

*Note: This parameter is for advanced users*

Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term

- Range: 0 100

- Units: Hz

## PSC_VELXY_FF: Velocity (horizontal) feed forward gain

*Note: This parameter is for advanced users*

Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration

- Range: 0 6

- Increment: 0.01

## PSC_ANGLE_MAX: Position Control Angle Max

*Note: This parameter is for advanced users*

Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value

- Units: deg

- Range: 0 45

- Increment: 1

## PSC_JERK_XY: Jerk limit for the horizontal kinematic input shaping

*Note: This parameter is for advanced users*

Jerk limit of the horizontal kinematic path generation used to determine how quickly the aircraft varies the acceleration target

- Units: m/s/s/s

- Range: 1 20

- Increment: 1

## PSC_JERK_Z: Jerk limit for the vertical kinematic input shaping

*Note: This parameter is for advanced users*

Jerk limit of the vertical kinematic path generation used to determine how quickly the aircraft varies the acceleration target

- Units: m/s/s/s

- Range: 5 50

- Increment: 1

# RALLY Parameters

## RALLY_TOTAL: Rally Total

*Note: This parameter is for advanced users*

Number of rally points currently loaded

## RALLY_LIMIT_KM: Rally Limit

*Note: This parameter is for advanced users*

Maximum distance to rally point. If the closest rally point is more than this number of kilometers from the current position and the home location is closer than any of the rally points from the current position then do RTL to home rather than to the closest rally point. This prevents a leftover rally point from a different airfield being used accidentally. If this is set to 0 then the closest rally point is always used.

- Units: km

- Increment: 0.1

## RALLY_INCL_HOME: Rally Include Home

Controls if Home is included as a Rally point (i.e. as a safe landing place) for RTL

|Value|Meaning|
|:---:|:---:|
|0|DoNotIncludeHome|
|1|IncludeHome|

# RC Parameters

## RC_OVERRIDE_TIME: RC override timeout

*Note: This parameter is for advanced users*

Timeout after which RC overrides will no longer be used, and RC input will resume, 0 will disable RC overrides, -1 will never timeout, and continue using overrides until they are disabled

- Range: 0.0 120.0

- Units: s

## RC_OPTIONS: RC options

*Note: This parameter is for advanced users*

RC input options

- Bitmask: 0:Ignore RC Receiver, 1:Ignore MAVLink Overrides, 2:Ignore Receiver Failsafe bit but allow other RC failsafes if setup, 3:FPort Pad, 4:Log RC input bytes, 5:Arming check throttle for 0 input, 6:Skip the arming check for neutral Roll/Pitch/Yaw sticks, 7:Allow Switch reverse, 8:Use passthrough for CRSF telemetry, 9:Suppress CRSF mode/rate message for ELRS systems,10:Enable multiple receiver support, 11:Use Link Quality for RSSI with CRSF, 13: Use 420kbaud for ELRS protocol

## RC_PROTOCOLS: RC protocols enabled

*Note: This parameter is for advanced users*

Bitmask of enabled RC protocols. Allows narrowing the protocol detection to only specific types of RC receivers which can avoid issues with incorrect detection. Set to 1 to enable all protocols.

- Bitmask: 0:All,1:PPM,2:IBUS,3:SBUS,4:SBUS_NI,5:DSM,6:SUMD,7:SRXL,8:SRXL2,9:CRSF,10:ST24,11:FPORT,12:FPORT2,13:FastSBUS

# RCn Parameters

## RCn_MIN: RC min PWM

*Note: This parameter is for advanced users*

RC minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## RCn_TRIM: RC trim PWM

*Note: This parameter is for advanced users*

RC trim (neutral) PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## RCn_MAX: RC max PWM

*Note: This parameter is for advanced users*

RC maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## RCn_REVERSED: RC reversed

*Note: This parameter is for advanced users*

Reverse channel input. Set to 0 for normal operation. Set to 1 to reverse this input channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## RCn_DZ: RC dead-zone

*Note: This parameter is for advanced users*

PWM dead zone in microseconds around trim or bottom

- Units: PWM

- Range: 0 200

## RCn_OPTION: RC input option

Function assigned to this RC channel

|Value|Meaning|
|:---:|:---:|
|0|Do Nothing|
|2|Flip|
|3|Simple Mode|
|4|RTL|
|5|Save Trim|
|7|Save WP|
|9|Camera Trigger|
|10|RangeFinder|
|11|Fence|
|13|Super Simple Mode|
|14|Acro Trainer|
|15|Sprayer|
|16|Auto|
|17|AutoTune|
|18|Land|
|19|Gripper|
|21|Parachute Enable|
|22|Parachute Release|
|23|Parachute 3pos|
|24|Auto Mission Reset|
|25|AttCon Feed Forward|
|26|AttCon Accel Limits|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Copter Sound|
|31|Motor Emergency Stop|
|32|Motor Interlock|
|33|Brake|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|37|Throw|
|38|ADSB Avoidance En|
|39|PrecLoiter|
|40|Proximity Avoidance|
|41|ArmDisarm (4.1 and lower)|
|42|SmartRTL|
|43|InvertedFlight|
|44|Winch Enable|
|46|RC Override Enable|
|47|User Function 1|
|48|User Function 2|
|49|User Function 3|
|52|Acro|
|55|Guided|
|56|Loiter|
|57|Follow|
|58|Clear Waypoints|
|60|ZigZag|
|61|ZigZag SaveWP|
|62|Compass Learn|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|68|Stabilize|
|69|PosHold|
|70|AltHold|
|71|FlowHold|
|72|Circle|
|73|Drift|
|75|SurfaceTrackingUpDown|
|76|Standby Mode|
|78|RunCam Control|
|79|RunCam OSD Control|
|80|VisOdom Align|
|81|Disarm|
|83|ZigZag Auto|
|84|Air Mode|
|85|Generator|
|90|EKF Pos Source|
|94|VTX Power|
|99|AUTO RTL|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|151|Turtle|
|152|simple heading reset|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with AirMode  (4.2 and higher)|
|158|Optflow Calibration|
|159|Force Flying|
|161|Turbine Start(heli)|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|212|Mount1 Roll|
|213|Mount1 Pitch|
|214|Mount1 Yaw|
|215|Mount2 Roll|
|216|Mount2 Pitch|
|217|Mount2 Yaw|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|

# RCMAP Parameters

## RCMAP_ROLL: Roll channel

*Note: This parameter is for advanced users*

Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.

- Range: 1 16

- Increment: 1

- RebootRequired: True

## RCMAP_PITCH: Pitch channel

*Note: This parameter is for advanced users*

Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.

- Range: 1 16

- Increment: 1

- RebootRequired: True

## RCMAP_THROTTLE: Throttle channel

*Note: This parameter is for advanced users*

Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Reboot is required for changes to take effect.

- Range: 1 16

- Increment: 1

- RebootRequired: True

## RCMAP_YAW: Yaw channel

*Note: This parameter is for advanced users*

Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.

- Range: 1 16

- Increment: 1

- RebootRequired: True

# RELAY Parameters

## RELAY_PIN: First Relay Pin

Digital pin number for first relay control. This is the pin used for camera shutter control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|
|27|BBBMini Pin P8.17|
|101|MainOut1|
|102|MainOut2|
|103|MainOut3|
|104|MainOut4|
|105|MainOut5|
|106|MainOut6|
|107|MainOut7|
|108|MainOut8|

## RELAY_PIN2: Second Relay Pin

Digital pin number for 2nd relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|
|65|BBBMini Pin P8.18|
|101|MainOut1|
|102|MainOut2|
|103|MainOut3|
|104|MainOut4|
|105|MainOut5|
|106|MainOut6|
|107|MainOut7|
|108|MainOut8|

## RELAY_PIN3: Third Relay Pin

Digital pin number for 3rd relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|
|22|BBBMini Pin P8.19|
|101|MainOut1|
|102|MainOut2|
|103|MainOut3|
|104|MainOut4|
|105|MainOut5|
|106|MainOut6|
|107|MainOut7|
|108|MainOut8|

## RELAY_PIN4: Fourth Relay Pin

Digital pin number for 4th relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|
|63|BBBMini Pin P8.34|
|101|MainOut1|
|102|MainOut2|
|103|MainOut3|
|104|MainOut4|
|105|MainOut5|
|106|MainOut6|
|107|MainOut7|
|108|MainOut8|

## RELAY_DEFAULT: Default relay state

The state of the relay on boot.

|Value|Meaning|
|:---:|:---:|
|0|Off|
|1|On|
|2|NoChange|

## RELAY_PIN5: Fifth Relay Pin

Digital pin number for 5th relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|
|62|BBBMini Pin P8.13|
|101|MainOut1|
|102|MainOut2|
|103|MainOut3|
|104|MainOut4|
|105|MainOut5|
|106|MainOut6|
|107|MainOut7|
|108|MainOut8|

## RELAY_PIN6: Sixth Relay Pin

Digital pin number for 6th relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|49|BB Blue GP0 pin 4|
|50|AUXOUT1|
|51|AUXOUT2|
|52|AUXOUT3|
|53|AUXOUT4|
|54|AUXOUT5|
|55|AUXOUT6|
|57|BB Blue GP0 pin 3|
|113|BB Blue GP0 pin 6|
|116|BB Blue GP0 pin 5|
|37|BBBMini Pin P8.14|
|101|MainOut1|
|102|MainOut2|
|103|MainOut3|
|104|MainOut4|
|105|MainOut5|
|106|MainOut6|
|107|MainOut7|
|108|MainOut8|

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

# RPM1 Parameters

## RPM1_TYPE: RPM type

What type of RPM sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Not Used|
|2|GPIO|
|3|EFI|
|4|Harmonic Notch|
|5|ESC Telemetry Motors Bitmask|
|6|Generator|

## RPM1_SCALING: RPM scaling

Scaling factor between sensor reading and RPM.

- Increment: 0.001

## RPM1_MAX: Maximum RPM

Maximum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM1_MIN: Minimum RPM

Minimum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM1_MIN_QUAL: Minimum Quality

*Note: This parameter is for advanced users*

Minimum data quality to be used

- Increment: 0.1

## RPM1_PIN: Input pin number

Which digital GPIO pin to use. Only used on type = GPIO. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|

## RPM1_ESC_MASK: Bitmask of ESC telemetry channels to average

*Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry. RPM telemetry of the selected channels will be averaged

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

# RPM2 Parameters

## RPM2_TYPE: RPM type

What type of RPM sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Not Used|
|2|GPIO|
|3|EFI|
|4|Harmonic Notch|
|5|ESC Telemetry Motors Bitmask|
|6|Generator|

## RPM2_SCALING: RPM scaling

Scaling factor between sensor reading and RPM.

- Increment: 0.001

## RPM2_MAX: Maximum RPM

Maximum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM2_MIN: Minimum RPM

Minimum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM2_MIN_QUAL: Minimum Quality

*Note: This parameter is for advanced users*

Minimum data quality to be used

- Increment: 0.1

## RPM2_PIN: Input pin number

Which digital GPIO pin to use. Only used on type = GPIO. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|AUX1|
|51|AUX2|
|52|AUX3|
|53|AUX4|
|54|AUX5|
|55|AUX6|

## RPM2_ESC_MASK: Bitmask of ESC telemetry channels to average

*Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry. RPM telemetry of the selected channels will be averaged

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

# RSSI Parameters

## RSSI_TYPE: RSSI Type

Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|AnalogPin|
|2|RCChannelPwmValue|
|3|ReceiverProtocol|
|4|PWMInputPin|
|5|TelemetryRadioRSSI|

## RSSI_ANA_PIN: Receiver RSSI sensing pin

Pin used to read the RSSI voltage or PWM value

|Value|Meaning|
|:---:|:---:|
|8|V5 Nano|
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

## RSSI_PIN_LOW: RSSI pin's lowest voltage

RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the weakest. Some radio receivers put out inverted values so this value may be higher than RSSI_PIN_HIGH. When using pin 103, the maximum value of the parameter is 3.3V.

- Units: V

- Increment: 0.01

- Range: 0 5.0

## RSSI_PIN_HIGH: RSSI pin's highest voltage

RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the strongest. Some radio receivers put out inverted values so this value may be lower than RSSI_PIN_LOW. When using pin 103, the maximum value of the parameter is 3.3V.

- Units: V

- Increment: 0.01

- Range: 0 5.0

## RSSI_CHANNEL: Receiver RSSI channel number

The channel number where RSSI will be output by the radio receiver (5 and above).

- Range: 0 16

## RSSI_CHAN_LOW: RSSI PWM low value

PWM value that the radio receiver will put on the RSSI_CHANNEL or RSSI_ANA_PIN when the signal strength is the weakest. Some radio receivers output inverted values so this value may be lower than RSSI_CHAN_HIGH

- Units: PWM

- Range: 0 2000

## RSSI_CHAN_HIGH: Receiver RSSI PWM high value

PWM value that the radio receiver will put on the RSSI_CHANNEL or RSSI_ANA_PIN when the signal strength is the strongest. Some radio receivers output inverted values so this value may be higher than RSSI_CHAN_LOW

- Units: PWM

- Range: 0 2000

# SCHED Parameters

## SCHED_DEBUG: Scheduler debug level

*Note: This parameter is for advanced users*

Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|2|ShowSlips|
|3|ShowOverruns|

## SCHED_LOOP_RATE: Scheduling main loop rate

*Note: This parameter is for advanced users*

This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart. Values over 400 are considered highly experimental.

|Value|Meaning|
|:---:|:---:|
|50|50Hz|
|100|100Hz|
|200|200Hz|
|250|250Hz|
|300|300Hz|
|400|400Hz|

- RebootRequired: True

## SCHED_OPTIONS: Scheduling options

*Note: This parameter is for advanced users*

This controls optional aspects of the scheduler.

- Bitmask: 0:Enable per-task perf info

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
|8|SToRM32 Gimbal Serial|
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
|8|SToRM32 Gimbal Serial|
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
|8|SToRM32 Gimbal Serial|
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
|8|SToRM32 Gimbal Serial|
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
|8|SToRM32 Gimbal Serial|
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
|8|SToRM32 Gimbal Serial|
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

## SERIAL1_OPTIONS: Telem1 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL2_OPTIONS: Telem2 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL3_OPTIONS: Serial3 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL4_OPTIONS: Serial4 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL5_OPTIONS: Serial5 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

## SERIAL6_OPTIONS: Serial6 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

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
|8|SToRM32 Gimbal Serial|
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

## SERIAL7_OPTIONS: Serial7 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

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
|8|SToRM32 Gimbal Serial|
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

## SERIAL8_OPTIONS: Serial8 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

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
|8|SToRM32 Gimbal Serial|
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

## SERIAL9_OPTIONS: Serial9 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate

- RebootRequired: True

# SERVO Parameters

## SERVO_RATE: Servo default output rate

*Note: This parameter is for advanced users*

This sets the default output rate in Hz for all outputs.

- Range: 25 400

- Units: Hz

## SERVO_DSHOT_RATE: Servo DShot output rate

*Note: This parameter is for advanced users*

This sets the DShot output rate for all outputs as a multiple of the loop rate. 0 sets the output rate to be fixed at 1Khz for low loop rates. This value should never be set below 500Hz.

|Value|Meaning|
|:---:|:---:|
|0|1Khz|
|1|loop-rate|
|2|double loop-rate|
|3|triple loop-rate|
|4|quadruple loop rate|

## SERVO_DSHOT_ESC: Servo DShot ESC type

*Note: This parameter is for advanced users*

This sets the DShot ESC type for all outputs. The ESC type affects the range of DShot commands available. None means that no dshot commands will be executed.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|BLHeli32/Kiss|
|2|BLHeli_S|

## SERVO_GPIO_MASK: Servo GPIO mask

*Note: This parameter is for advanced users*

This sets a bitmask of outputs which will be available as GPIOs. Any auxiliary output with either the function set to -1 or with the corresponding bit set in this mask will be available for use as a GPIO pin

- Bitmask: 0:Servo 1, 1:Servo 2, 2:Servo 3, 3:Servo 4, 4:Servo 5, 5:Servo 6, 6:Servo 7, 7:Servo 8, 8:Servo 9, 9:Servo 10, 10:Servo 11, 11:Servo 12, 12:Servo 13, 13:Servo 14, 14:Servo 15, 15:Servo 16, 16:Servo 17, 17:Servo 18, 18:Servo 19, 19:Servo 20, 20:Servo 21, 21:Servo 22, 22:Servo 23, 23:Servo 24, 24:Servo 25, 25:Servo 26, 26:Servo 27, 27:Servo 28, 28:Servo 29, 29:Servo 30, 30:Servo 31, 31:Servo 32

- RebootRequired: True

## SERVO_32_ENABLE: Enable outputs 17 to 31

*Note: This parameter is for advanced users*

This allows for up to 32 outputs, enabling parameters for outputs above 16

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

# SERVOn Parameters

## SERVOn_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## SERVOn_MAX: Maximum PWM

maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## SERVOn_TRIM: Trim PWM

Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 800 2200

- Increment: 1

## SERVOn_REVERSED: Servo reverse

Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Reversed|

## SERVOn_FUNCTION: Servo output function

Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function

|Value|Meaning|
|:---:|:---:|
|-1|GPIO|
|0|Disabled|
|1|RCPassThru|
|6|Mount1Yaw|
|7|Mount1Pitch|
|8|Mount1Roll|
|9|Mount1Retract|
|10|CameraTrigger|
|12|Mount2Yaw|
|13|Mount2Pitch|
|14|Mount2Roll|
|15|Mount2Retract|
|22|SprayerPump|
|23|SprayerSpinner|
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
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|81|BoostThrottle|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|88|Winch|
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
|129|ProfiLED1|
|130|ProfiLED2|
|131|ProfiLED3|
|132|ProfiLEDClock|
|133|Winch Clutch|
|134|SERVOn_MIN|
|135|SERVOn_TRIM|
|136|SERVOn_MAX|
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

# SERVOBLH Parameters

## SERVO_BLH_MASK: BLHeli Channel Bitmask

*Note: This parameter is for advanced users*

Enable of BLHeli pass-thru servo protocol support to specific channels. This mask is in addition to motors enabled using SERVO_BLH_AUTO (if any)

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## SERVO_BLH_AUTO: BLHeli pass-thru auto-enable for multicopter motors

If set to 1 this auto-enables BLHeli pass-thru support for all multicopter motors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## SERVO_BLH_TEST: BLHeli internal interface test

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

## SERVO_BLH_TMOUT: BLHeli protocol timeout

This sets the inactivity timeout for the BLHeli protocol in seconds. If no packets are received in this time normal MAVLink operations are resumed. A value of 0 means no timeout

- Units: s

- Range: 0 300

## SERVO_BLH_TRATE: BLHeli telemetry rate

This sets the rate in Hz for requesting telemetry from ESCs. It is the rate per ESC. Setting to zero disables telemetry requests

- Units: Hz

- Range: 0 500

## SERVO_BLH_DEBUG: BLHeli debug level

When set to 1 this enabled verbose debugging output over MAVLink when the blheli protocol is active. This can be used to diagnose failures.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## SERVO_BLH_OTYPE: BLHeli output type override

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

## SERVO_BLH_PORT: Control port

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

## SERVO_BLH_POLES: BLHeli Motor Poles

*Note: This parameter is for advanced users*

This allows calculation of true RPM from ESC's eRPM. The default is 14.

- Range: 1 127

- RebootRequired: True

## SERVO_BLH_3DMASK: BLHeli bitmask of 3D channels

*Note: This parameter is for advanced users*

Mask of channels which are dynamically reversible. This is used to configure ESCs in '3D' mode, allowing for the motor to spin in either direction

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## SERVO_BLH_BDMASK: BLHeli bitmask of bi-directional dshot channels

*Note: This parameter is for advanced users*

Mask of channels which support bi-directional dshot. This is used for ESCs which have firmware that supports bi-directional dshot allowing fast rpm telemetry values to be returned for the harmonic notch.

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## SERVO_BLH_RVMASK: BLHeli bitmask of reversed channels

*Note: This parameter is for advanced users*

Mask of channels which are reversed. This is used to configure ESCs in reversed mode

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

# SERVOFTW Parameters

## SERVO_FTW_MASK: Servo channel output bitmask

Servo channel mask specifying FETtec ESC output.

- Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12

- RebootRequired: True

## SERVO_FTW_RVMASK: Servo channel reverse rotation bitmask

Servo channel mask to reverse rotation of FETtec ESC outputs.

- Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12

## SERVO_FTW_POLES: Nr. electrical poles

Number of motor electrical poles

- Range: 2 50

# SERVOROB Parameters

## SERVO_ROB_POSMIN: Robotis servo position min

Position minimum at servo min value. This should be within the position control range of the servos, normally 0 to 4095

- Range: 0 4095

## SERVO_ROB_POSMAX: Robotis servo position max

Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095

- Range: 0 4095

# SERVOSBUS Parameters

## SERVO_SBUS_RATE: SBUS default output rate

*Note: This parameter is for advanced users*

This sets the SBUS output frame rate in Hz.

- Range: 25 250

- Units: Hz

# SERVOVOLZ Parameters

## SERVO_VOLZ_MASK: Channel Bitmask

Enable of volz servo protocol to specific channels

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16,16:Channel17,17:Channel18,18:Channel19,19:Channel20,20:Channel21,21:Channel22,22:Channel23,23:Channel24,24:Channel25,25:Channel26,26:Channel27,28:Channel29,29:Channel30,30:Channel31,31:Channel32

# SID Parameters

## SID_AXIS: System identification axis

Controls which axis are being excited.  Set to non-zero to see more parameters

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Input Roll Angle|
|2|Input Pitch Angle|
|3|Input Yaw Angle|
|4|Recovery Roll Angle|
|5|Recovery Pitch Angle|
|6|Recovery Yaw Angle|
|7|Rate Roll|
|8|Rate Pitch|
|9|Rate Yaw|
|10|Mixer Roll|
|11|Mixer Pitch|
|12|Mixer Yaw|
|13|Mixer Thrust|

## SID_MAGNITUDE: System identification Chirp Magnitude

Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.

## SID_F_START_HZ: System identification Start Frequency

Frequency at the start of the sweep

- Range: 0.01 100

- Units: Hz

## SID_F_STOP_HZ: System identification Stop Frequency

Frequency at the end of the sweep

- Range: 0.01 100

- Units: Hz

## SID_T_FADE_IN: System identification Fade in time

Time to reach maximum amplitude of sweep

- Range: 0 20

- Units: s

## SID_T_REC: System identification Total Sweep length

Time taken to complete the sweep

- Range: 0 255

- Units: s

## SID_T_FADE_OUT: System identification Fade out time

Time to reach zero amplitude at the end of the sweep

- Range: 0 5

- Units: s

# SPRAY Parameters

## SPRAY_ENABLE: Sprayer enable/disable

Allows you to enable (1) or disable (0) the sprayer

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## SPRAY_PUMP_RATE: Pump speed

Desired pump speed when traveling 1m/s expressed as a percentage

- Units: %

- Range: 0 100

## SPRAY_SPINNER: Spinner rotation speed

Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)

- Units: ms

- Range: 1000 2000

## SPRAY_SPEED_MIN: Speed minimum

Speed minimum at which we will begin spraying

- Units: cm/s

- Range: 0 1000

## SPRAY_PUMP_MIN: Pump speed minimum

Minimum pump speed expressed as a percentage

- Units: %

- Range: 0 100

# SRn Parameters

## SRn_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and SENSOR_OFFSETS to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXT_STAT: Extended status stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_RC_CHAN: RC Channel stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_RAW_CTRL: Unused

*Note: This parameter is for advanced users*

Unused

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_POSITION: Position stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXTRA1: Extra data type 1 stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2 and PID_TUNING to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXTRA2: Extra data type 2 stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of VFR_HUD to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXTRA3: Extra data type 3 stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of AHRS, HWSTATUS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_PARAMS: Parameter stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of PARAM_VALUE to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_ADSB: ADSB stream rate to ground station

*Note: This parameter is for advanced users*

ADSB stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

# SRTL Parameters

## SRTL_ACCURACY: SmartRTL accuracy

*Note: This parameter is for advanced users*

SmartRTL accuracy. The minimum distance between points.

- Units: m

- Range: 0 10

## SRTL_POINTS: SmartRTL maximum number of points on path

*Note: This parameter is for advanced users*

SmartRTL maximum number of points on path. Set to 0 to disable SmartRTL.  100 points consumes about 3k of memory.

- Range: 0 500

- RebootRequired: True

## SRTL_OPTIONS: SmartRTL options

Bitmask of SmartRTL options.

- Bitmask: 2:Ignore pilot yaw

# STAT Parameters

## STAT_BOOTCNT: Boot Count

Number of times board has been booted

- ReadOnly: True

## STAT_FLTTIME: Total FlightTime

Total FlightTime (seconds)

- Units: s

- ReadOnly: True

## STAT_RUNTIME: Total RunTime

Total time autopilot has run

- Units: s

- ReadOnly: True

## STAT_RESET: Statistics Reset Time

Seconds since January 1st 2016 (Unix epoch+1451606400) since statistics reset (set to 0 to reset statistics)

- Units: s

- ReadOnly: True

# TCAL Parameters

## TCAL_ENABLED: Temperature calibration enable

*Note: This parameter is for advanced users*

Enable temperature calibration. Set to 0 to disable. Set to 1 to use learned values. Set to 2 to learn new values and use the values

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|EnableAndLearn|

## TCAL_TEMP_MIN: Temperature calibration min learned temperature

*Note: This parameter is for advanced users*

Minimum learned temperature. This is automatically set by the learning process

- Units: degC

- ReadOnly: True

- Volatile: True

## TCAL_TEMP_MAX: Temperature calibration max learned temperature

*Note: This parameter is for advanced users*

Maximum learned temperature. This is automatically set by the learning process

- Units: degC

- ReadOnly: True

- Volatile: True

## TCAL_BARO_EXP: Temperature Calibration barometer exponent

*Note: This parameter is for advanced users*

Learned exponent for barometer temperature correction

- ReadOnly: True

- Volatile: True

# TERRAIN Parameters

## TERRAIN_ENABLE: Terrain data enable

*Note: This parameter is for advanced users*

enable terrain data. This enables the vehicle storing a database of terrain data on the SD card. The terrain data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support TERRAIN_REQUEST messages and have access to a terrain database, such as the SRTM database.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## TERRAIN_SPACING: Terrain grid spacing

*Note: This parameter is for advanced users*

Distance between terrain grid points in meters. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station. If your GCS is using the ArduPilot SRTM database like Mission Planner or MAVProxy, then a resolution of 100 meters is appropriate. Grid spacings lower than 100 meters waste SD card space if the GCS cannot provide that resolution. The grid spacing also controls how much data is kept in memory during flight. A larger grid spacing will allow for a larger amount of data in memory. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2.7 kilometers by 3.2 kilometers. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be loaded as needed.

- Units: m

- Increment: 1

## TERRAIN_OPTIONS: Terrain options

*Note: This parameter is for advanced users*

Options to change behaviour of terrain system

- Bitmask: 0:Disable Download

## TERRAIN_MARGIN: Acceptance margin

*Note: This parameter is for advanced users*

Margin in centi-meters to accept terrain data from the GCS. This can be used to allow older terrain data generated with less accurate latitude/longitude scaling to be used

- Units: m

- Range: 0.05 50000

## TERRAIN_OFS_MAX: Terrain reference offset maximum

*Note: This parameter is for advanced users*

The maximum adjustment of terrain altitude based on the assumption that the vehicle is on the ground when it is armed. When the vehicle is armed the location of the vehicle is recorded, and when terrain data is available for that location a height adjustment for terrain data is calculated that aligns the terrain height at that location with the altitude recorded at arming. This height adjustment is applied to all terrain data. This parameter clamps the amount of adjustment. A value of zero disables the use of terrain height adjustment.

- Units: m

- Range: 0 50

# TMODE Parameters

## TMODE_ENABLE: tmode enable 

*Note: This parameter is for advanced users*

tmode (or "toy" mode) gives a simplified user interface designed for mass market drones. Version1 is for the SkyViper V2450GPS. Version2 is for the F412 based boards

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|EnableVersion1|
|2|EnableVersion2|

## TMODE_MODE1: Tmode first mode

This is the initial mode when the vehicle is first turned on. This mode is assumed to not require GPS

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|FlowHold|

## TMODE_MODE2: Tmode second mode

This is the secondary mode. This mode is assumed to require GPS

|Value|Meaning|
|:---:|:---:|
|0|Stabilize|
|1|Acro|
|2|AltHold|
|3|Auto|
|4|Guided|
|5|Loiter|
|6|RTL|
|7|Circle|
|9|Land|
|11|Drift|
|13|Sport|
|14|Flip|
|15|AutoTune|
|16|PosHold|
|17|Brake|
|18|Throw|
|19|Avoid_ADSB|
|20|Guided_NoGPS|
|21|FlowHold|

## TMODE_ACTION1: Tmode action 1

This is the action taken when the left action button is pressed

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_ACTION2: Tmode action 2

This is the action taken when the right action button is pressed

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_ACTION3: Tmode action 3

This is the action taken when the power button is pressed

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_ACTION4: Tmode action 4

This is the action taken when the left action button is pressed while the left (Mode) button is held down

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_ACTION5: Tmode action 5

This is the action taken when the right action is pressed while the left (Mode) button is held down

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_ACTION6: Tmode action 6

This is the action taken when the power button is pressed while the left (Mode) button is held down

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_LEFT: Tmode left action

This is the action taken when the left (Mode) button is pressed

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_LEFT_LONG: Tmode left long action

This is the action taken when the left (Mode) button is long-pressed

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|
|24|ModeFlowHold|

## TMODE_TRIM_AUTO: Stick auto trim limit

This is the amount of automatic stick trim that can be applied when disarmed with sticks not moving. It is a PWM limit value away from 1500

- Range: 0 100

## TMODE_RIGHT: Tmode right action

This is the action taken when the right (Return) button is pressed

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|TakePhoto|
|2|ToggleVideo|
|3|ModeAcro|
|4|ModeAltHold|
|5|ModeAuto|
|6|ModeLoiter|
|7|ModeRTL|
|8|ModeCircle|
|9|ModeLand|
|10|ModeDrift|
|11|ModeSport|
|12|ModeAutoTune|
|13|ModePosHold|
|14|ModeBrake|
|15|ModeThrow|
|16|Flip|
|17|ModeStabilize|
|18|Disarm|
|19|ToggleMode|
|20|Arm-Land-RTL|
|21|ToggleSimpleMode|
|22|ToggleSuperSimpleMode|
|23|MotorLoadTest|

## TMODE_FLAGS: Tmode flags

Bitmask of flags to change the behaviour of tmode. DisarmOnLowThrottle means to disarm if throttle is held down for 1 second when landed. ArmOnHighThrottle means to arm if throttle is above 80% for 1 second. UpgradeToLoiter means to allow takeoff in LOITER mode by switching to ALT_HOLD, then auto-upgrading to LOITER once GPS is available. RTLStickCancel means that on large stick inputs in RTL mode that LOITER mode is engaged

- Bitmask: 0:DisarmOnLowThrottle,1:ArmOnHighThrottle,2:UpgradeToLoiter,3:RTLStickCancel

## TMODE_VMIN: Min voltage for output limiting

*Note: This parameter is for advanced users*

This is the battery voltage below which no output limiting is done

- Range: 0 5

- Increment: 0.01

## TMODE_VMAX: Max voltage for output limiting

*Note: This parameter is for advanced users*

This is the battery voltage above which thrust min is used

- Range: 0 5

- Increment: 0.01

## TMODE_TMIN: Min thrust multiplier

*Note: This parameter is for advanced users*

This sets the thrust multiplier when voltage is high

- Range: 0 1

- Increment: 0.01

## TMODE_TMAX: Max thrust multiplier

*Note: This parameter is for advanced users*

This sets the thrust multiplier when voltage is low

- Range: 0 1

- Increment: 0.01

## TMODE_LOAD_MUL: Load test multiplier

*Note: This parameter is for advanced users*

This scales the load test output, as a value between 0 and 1

- Range: 0 1

- Increment: 0.01

## TMODE_LOAD_FILT: Load test filter

*Note: This parameter is for advanced users*

This filters the load test output. A value of 1 means no filter. 2 means values are repeated once. 3 means values are repeated 3 times, etc

- Range: 0 100

## TMODE_LOAD_TYPE: Load test type

*Note: This parameter is for advanced users*

This sets the type of load test

|Value|Meaning|
|:---:|:---:|
|0|ConstantThrust|
|1|LogReplay1|
|2|LogReplay2|

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

# WINCH Parameters

## WINCH_TYPE: Winch Type

Winch Type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|PWM|
|2|Daiwa|

## WINCH_RATE_MAX: Winch deploy or retract rate maximum

Winch deploy or retract rate maximum.  Set to maximum rate with no load.

- Range: 0 10

- Units: m/s

## WINCH_POS_P: Winch control position error P gain

Winch control position error P gain

- Range: 0.01 10.0

# WPNAV Parameters

## WPNAV_SPEED: Waypoint Horizontal Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission

- Units: cm/s

- Range: 20 2000

- Increment: 50

## WPNAV_RADIUS: Waypoint Radius

Defines the distance from a waypoint, that when crossed indicates the wp has been hit.

- Units: cm

- Range: 5 1000

- Increment: 1

## WPNAV_SPEED_UP: Waypoint Climb Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission

- Units: cm/s

- Range: 10 1000

- Increment: 50

## WPNAV_SPEED_DN: Waypoint Descent Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission

- Units: cm/s

- Range: 10 500

- Increment: 10

## WPNAV_ACCEL: Waypoint Acceleration 

Defines the horizontal acceleration in cm/s/s used during missions

- Units: cm/s/s

- Range: 50 500

- Increment: 10

## WPNAV_ACCEL_Z: Waypoint Vertical Acceleration

Defines the vertical acceleration in cm/s/s used during missions

- Units: cm/s/s

- Range: 50 500

- Increment: 10

## WPNAV_RFND_USE: Waypoint missions use rangefinder for terrain following

*Note: This parameter is for advanced users*

This controls if waypoint missions use rangefinder for terrain following

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## WPNAV_JERK: Waypoint Jerk

Defines the horizontal jerk in m/s/s used during missions

- Units: m/s/s/s

- Range: 1 20

## WPNAV_TER_MARGIN: Waypoint Terrain following altitude margin

*Note: This parameter is for advanced users*

Waypoint Terrain following altitude margin.  Vehicle will stop if distance from target altitude is larger than this margin (in meters)

- Units: m

- Range: 0.1 100

# ZIGZ Parameters

## ZIGZ_AUTO_ENABLE: ZigZag auto enable/disable

*Note: This parameter is for advanced users*

Allows you to enable (1) or disable (0) ZigZag auto feature

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ZIGZ_SPRAYER: Auto sprayer in ZigZag

*Note: This parameter is for advanced users*

Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ZIGZ_WP_DELAY: The delay for zigzag waypoint

*Note: This parameter is for advanced users*

Waiting time after reached the destination

- Units: s

- Range: 0 127

## ZIGZ_SIDE_DIST: Sideways distance in ZigZag auto

*Note: This parameter is for advanced users*

The distance to move sideways in ZigZag mode

- Units: m

- Range: 0.1 100

## ZIGZ_DIRECTION: Sideways direction in ZigZag auto

*Note: This parameter is for advanced users*

The direction to move sideways in ZigZag mode

|Value|Meaning|
|:---:|:---:|
|0|forward|
|1|right|
|2|backward|
|3|left|

## ZIGZ_LINE_NUM: Total number of lines

*Note: This parameter is for advanced users*

Total number of lines for ZigZag auto if 1 or more. -1: Infinity, 0: Just moving to sideways

- Range: -1 32767