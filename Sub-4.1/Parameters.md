
This is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your autopilot to control vehicle behaviour. Some parameters may only be available for developers, and are enabled at compile-time.

# ArduSub Parameters

## SURFACE_DEPTH: Depth reading at surface

The depth the external pressure sensor will read when the vehicle is considered at the surface (in centimeters)

- Units: cm

- Range: -100 0

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

## PILOT_THR_FILT: Throttle filter cutoff

*Note: This parameter is for advanced users*

Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable

- Units: Hz

- Range: 0 10

- Increment: .5

## GCS_PID_MASK: GCS PID tuning mask

*Note: This parameter is for advanced users*

bitmask of PIDs to send MAVLink PID_TUNING messages for

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Roll|
|2|Pitch|
|4|Yaw|

- Bitmask: 0:Roll,1:Pitch,2:Yaw

## RNGFND_GAIN: Rangefinder gain

Used to adjust the speed with which the target altitude is changed when objects are sensed below the sub

- Range: 0.01 2.0

- Increment: 0.01

## FS_GCS_ENABLE: Ground Station Failsafe Enable

Controls what action to take when GCS heartbeat is lost.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Warn only|
|2|Disarm|
|3|Enter depth hold mode|
|4|Enter surface mode|

## FS_LEAK_ENABLE: Leak Failsafe Enable

Controls what action to take if a leak is detected.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Warn only|
|2|Enter surface mode|

## FS_PRESS_ENABLE: Internal Pressure Failsafe Enable

Controls what action to take if internal pressure exceeds FS_PRESS_MAX parameter.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Warn only|

## FS_TEMP_ENABLE: Internal Temperature Failsafe Enable

Controls what action to take if internal temperature exceeds FS_TEMP_MAX parameter.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Warn only|

## FS_PRESS_MAX: Internal Pressure Failsafe Threshold

The maximum internal pressure allowed before triggering failsafe. Failsafe action is determined by FS_PRESS_ENABLE parameter

- Units: Pa

## FS_TEMP_MAX: Internal Temperature Failsafe Threshold

The maximum internal temperature allowed before triggering failsafe. Failsafe action is determined by FS_TEMP_ENABLE parameter.

- Units: degC

## FS_TERRAIN_ENAB: Terrain Failsafe Enable

Controls what action to take if terrain information is lost during AUTO mode

|Value|Meaning|
|:---:|:---:|
|0|Disarm|
|1|Hold Position|
|2|Surface|

## FS_PILOT_INPUT: Pilot input failsafe action

Controls what action to take if no pilot input has been received after the timeout period specified by the FS_PILOT_TIMEOUT parameter

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Warn Only|
|2|Disarm|

## FS_PILOT_TIMEOUT: Timeout for activation of pilot input failsafe

Controls the maximum interval between received pilot inputs before the failsafe action is triggered

- Units: s

- Range: 0.1 3.0

## XTRACK_ANG_LIM: Crosstrack correction angle limit

Maximum allowed angle (in degrees) between current track and desired heading during waypoint navigation

- Range: 10 90

## WP_YAW_BEHAVIOR: Yaw behaviour during missions

Determines how the autopilot controls the yaw during missions and RTL

|Value|Meaning|
|:---:|:---:|
|0|Never change yaw|
|1|Face next waypoint|
|2|Face next waypoint except RTL|
|3|Face along GPS course|
|4|Correct crosstrack error|

## PILOT_SPEED_UP: Pilot maximum vertical ascending speed

The maximum vertical ascending velocity the pilot may request in cm/s

- Units: cm/s

- Range: 50 500

- Increment: 10

## PILOT_SPEED_DN: Pilot maximum vertical descending speed

The maximum vertical descending velocity the pilot may request in cm/s

- Units: cm/s

- Range: 50 500

- Increment: 10

## PILOT_ACCEL_Z: Pilot vertical acceleration

The vertical acceleration used when pilot is controlling the altitude

- Units: cm/s/s

- Range: 50 500

- Increment: 10

## THR_DZ: Throttle deadzone

The PWM deadzone in microseconds above and below mid throttle. Used in AltHold, Loiter, PosHold flight modes

- Range: 0 300

- Units: PWM

- Increment: 1

## LOG_BITMASK: Log bitmask

4 byte bitmap of log types to enable

|Value|Meaning|
|:---:|:---:|
|830|Default|
|894|Default+RCIN|
|958|Default+IMU|
|1854|Default+Motors|
|-6146|NearlyAll-AC315|
|45054|NearlyAll|
|131071|All+FastATT|
|262142|All+MotBatt|
|393214|All+FastIMU|
|397310|All+FastIMU+PID|
|655358|All+FullIMU|
|0|Disabled|

- Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:RCIN,7:IMU,8:CMD,9:CURRENT,10:RCOUT,11:OPTFLOW,12:PID,13:COMPASS,14:INAV,15:CAMERA,17:MOTBATT,18:IMU_FAST,19:IMU_RAW

## ANGLE_MAX: Angle Max

*Note: This parameter is for advanced users*

Maximum lean angle in all flight modes

- Units: cdeg

- Increment: 10

- Range: 1000 8000

## FS_EKF_ACTION: EKF Failsafe Action

*Note: This parameter is for advanced users*

Controls the action that will be taken when an EKF failsafe is invoked

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Warn only|
|2|Disarm|

## FS_EKF_THRESH: EKF failsafe variance threshold

*Note: This parameter is for advanced users*

Allows setting the maximum acceptable compass and velocity variance

- Values: 0.6:Strict,0.8:Default,1.0:Relaxed

## FS_CRASH_CHECK: Crash check enable

*Note: This parameter is for advanced users*

This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Warn only|
|2|Disarm|

## JS_GAIN_DEFAULT: Default gain at boot

Default gain at boot, must be in range [JS_GAIN_MIN , JS_GAIN_MAX]

- Range: 0.1 1.0

## JS_GAIN_MAX: Maximum joystick gain

Maximum joystick gain

- Range: 0.2 1.0

## JS_GAIN_MIN: Minimum joystick gain

Minimum joystick gain

- Range: 0.1 0.8

## JS_GAIN_STEPS: Gain steps

Controls the number of steps between minimum and maximum joystick gain when the gain is adjusted using buttons. Set to 1 to always use JS_GAIN_DEFAULT.

- Range: 1 10

## JS_LIGHTS_STEPS: Lights brightness steps

Number of steps in brightness between minimum and maximum brightness

- Range: 1 10

- Units: PWM

## JS_THR_GAIN: Throttle gain scalar

Scalar for gain on the throttle channel

- Range: 0.5 4.0

## FRAME_CONFIG: Frame configuration

Set this parameter according to your vehicle/motor configuration

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|BlueROV1|
|1|Vectored|
|2|Vectored_6DOF|
|3|Vectored_6DOF_90|
|4|SimpleROV-3|
|5|SimpleROV-4|
|6|SimpleROV-5|
|7|Custom|

## RC_SPEED: ESC Update Speed

*Note: This parameter is for advanced users*

This is the speed in Hertz that your ESCs will receive updates

- Units: Hz

- Range: 50 490

- Increment: 1

## ACRO_RP_P: Acro Roll and Pitch P gain

Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.

- Range: 1 10

## ACRO_YAW_P: Acro Yaw P gain

Converts pilot yaw input into a desired rate of rotation.  Higher values mean faster rate of rotation.

- Range: 1 10

## ACRO_BAL_ROLL: Acro Balance Roll

*Note: This parameter is for advanced users*

rate at which roll angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.

- Range: 0 3

- Increment: 0.1

## ACRO_BAL_PITCH: Acro Balance Pitch

*Note: This parameter is for advanced users*

rate at which pitch angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.

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

## ACRO_EXPO: Acro Expo

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

## CONTROL_FRAME: Control Frame

Control Frame used in stabilized modes

|Value|Meaning|
|:---:|:---:|
|0|Standard|
|1|Body frame Yaw/Pitch/Roll|

# AHRS Parameters

## AHRS_GPS_GAIN: AHRS GPS gain

*Note: This parameter is for advanced users*

This controls how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.

- Range: 0.0 1.0

- Increment: .01

## AHRS_GPS_USE: AHRS use GPS for navigation

*Note: This parameter is for advanced users*

This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight. Currently this affects only the DCM-based AHRS: the EKF uses GPS whenever it is available.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

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

Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.

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
|9|Roll180Yaw45|
|10|Roll180Yaw90|
|11|Roll180Yaw135|
|12|Pitch180|
|13|Roll180Yaw225|
|14|Roll180Yaw270|
|15|Roll180Yaw315|
|16|Roll90|
|17|Roll90Yaw45|
|18|Roll90Yaw90|
|19|Roll90Yaw135|
|20|Roll270|
|21|Roll270Yaw45|
|22|Roll270Yaw90|
|23|Roll270Yaw135|
|24|Pitch90|
|25|Pitch270|
|26|Pitch180Yaw90|
|27|Pitch180Yaw270|
|28|Roll90Pitch90|
|29|Roll180Pitch90|
|30|Roll270Pitch90|
|31|Roll90Pitch180|
|32|Roll270Pitch180|
|33|Roll90Pitch270|
|34|Roll180Pitch270|
|35|Roll270Pitch270|
|36|Roll90Pitch180Yaw90|
|37|Roll90Yaw270|
|38|Yaw293Pitch68Roll180|
|39|Pitch315|
|40|Roll90Pitch315|
|100|Custom|

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
|8|UAVCAN|
|9|I2C-DLVR-10in|
|10|I2C-DLVR-20in|
|11|I2C-DLVR-30in|
|12|I2C-DLVR-60in|
|13|NMEA water speed|
|14|MSP|
|15|ASP5033|

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

Enables automatic adjustment of ARSPD_RATIO during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD_TUBE_ORDER: Control pitot tube order

*Note: This parameter is for advanced users*

Changes the pitot tube order to specify the dynamic pressure side of the sensor. Accepts either if set to 2. Accepts only one side if set to 0 or 1 and can help detect excessive pressure on the static port without indicating positive airspeed.

## ARSPD_SKIP_CAL: Skip airspeed calibration on startup

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## ARSPD_PSI_RANGE: The PSI range of the device

*Note: This parameter is for advanced users*

This parameter allows you to to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD_BUS: Airspeed I2C bus

*Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|Bus0(internal)|
|1|Bus1(external)|
|2|Bus2(auxillary)|

## ARSPD_PRIMARY: Primary airspeed sensor

*Note: This parameter is for advanced users*

This selects which airspeed sensor will be the primary if multiple sensors are found

|Value|Meaning|
|:---:|:---:|
|0|FirstSensor|
|1|2ndSensor|

## ARSPD_OPTIONS: Airspeed options bitmask

*Note: This parameter is for advanced users*

Bitmask of options to use with airspeed. Disable and/or re-enable sensor based on the difference between airspeed and ground speed based on ARSPD_WIND_MAX threshold, if set

- Bitmask: 0:Disable sensor, 1:Re-enable sensor

## ARSPD_WIND_MAX: Maximum airspeed and ground speed difference

*Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy. Using ARSPD_OPTION this health value can be used to disable the sensor.

- Units: m/s

## ARSPD_WIND_WARN: Airspeed and ground speed difference that gives a warning

*Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will issue a warning. If 0 ARSPD_WIND_MAX is used.

- Units: m/s

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
|8|UAVCAN|
|9|I2C-DLVR-10in|
|10|I2C-DLVR-20in|
|11|I2C-DLVR-30in|
|12|I2C-DLVR-60in|
|13|NMEA water speed|
|14|MSP|
|15|ASP5033|

## ARSPD2_USE: Enable use of 2nd airspeed sensor

use airspeed for flight control. When set to 0 airspeed sensor can be logged and displayed on a GCS but won't be used for flight. When set to 1 it will be logged and used. When set to 2 it will be only used when the throttle is zero, which can be useful in gliders with airspeed sensors behind a propeller

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

If this is enabled then the autopilot will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. This option should be enabled for a calibration flight then disabled again when calibration is complete. Leaving it enabled all the time is not recommended.

## ARSPD2_TUBE_ORDR: Control pitot tube order of 2nd airspeed sensor

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure. If set to 1 then the bottom connector needs to be the dynamic pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft it receiving excessive pressure on the static port, which would otherwise be seen as a positive airspeed.

## ARSPD2_SKIP_CAL: Skip airspeed calibration on startup for 2nd sensor

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## ARSPD2_PSI_RANGE: The PSI range of the device for 2nd sensor

*Note: This parameter is for advanced users*

This parameter allows you to to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD2_BUS: Airspeed I2C bus for 2nd sensor

*Note: This parameter is for advanced users*

The bus number of the I2C bus to look for the sensor on

|Value|Meaning|
|:---:|:---:|
|0|Bus0(internal)|
|1|Bus1(external)|
|2|Bus2(auxillary)|

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

- Range: 0.0 12.000

## ATC_ANG_PIT_P: Pitch axis angle controller P gain

Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate

- Range: 0.0 12.000

## ATC_ANG_YAW_P: Yaw axis angle controller P gain

Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate

- Range: 0.0 6.000

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
|360|Slow|
|720|Medium|
|1080|Fast|

## ATC_RATE_P_MAX: Angular Velocity Max for Pitch

*Note: This parameter is for advanced users*

Maximum angular velocity in pitch axis

- Units: deg/s

- Range: 0 1080

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|360|Slow|
|720|Medium|
|1080|Fast|

## ATC_RATE_Y_MAX: Angular Velocity Max for Yaw

*Note: This parameter is for advanced users*

Maximum angular velocity in yaw axis

- Units: deg/s

- Range: 0 1080

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|360|Slow|
|720|Medium|
|1080|Fast|

## ATC_INPUT_TC: Attitude control input time constant

Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response

- Units: s

- Range: 0 1

- Increment: 0.01

- Values: 0.5:Very Soft,0.2:Soft,0.15:Medium,0.1:Crisp,0.05:Very Crisp

## ATC_RAT_RLL_P: Roll axis rate controller P gain

Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output

- Range: 0.0 0.30

- Increment: 0.005

## ATC_RAT_RLL_I: Roll axis rate controller I gain

Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate

- Range: 0.0 0.5

- Increment: 0.01

## ATC_RAT_RLL_IMAX: Roll axis rate controller I gain maximum

Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_RLL_D: Roll axis rate controller D gain

Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate

- Range: 0.0 0.02

- Increment: 0.001

## ATC_RAT_RLL_FF: Roll axis rate controller feed forward

Roll axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## ATC_RAT_RLL_FLTT: Roll axis rate controller input frequency in Hz

Roll axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_FLTE: Roll axis rate controller input frequency in Hz

Roll axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_FLTD: Roll axis rate controller input frequency in Hz

Roll axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_RLL_SMAX: Roll slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_RAT_PIT_P: Pitch axis rate controller P gain

Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output

- Range: 0.0 0.30

- Increment: 0.005

## ATC_RAT_PIT_I: Pitch axis rate controller I gain

Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate

- Range: 0.0 0.5

- Increment: 0.01

## ATC_RAT_PIT_IMAX: Pitch axis rate controller I gain maximum

Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## ATC_RAT_PIT_D: Pitch axis rate controller D gain

Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate

- Range: 0.0 0.02

- Increment: 0.001

## ATC_RAT_PIT_FF: Pitch axis rate controller feed forward

Pitch axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## ATC_RAT_PIT_FLTT: Pitch axis rate controller input frequency in Hz

Pitch axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_FLTE: Pitch axis rate controller input frequency in Hz

Pitch axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_FLTD: Pitch axis rate controller input frequency in Hz

Pitch axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_SMAX: Pitch slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## ATC_RAT_YAW_P: Yaw axis rate controller P gain

Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output

- Range: 0.0 0.50

- Increment: 0.005

## ATC_RAT_YAW_I: Yaw axis rate controller I gain

Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate

- Range: 0.0 0.05

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

## ATC_RAT_YAW_FLTT: Yaw axis rate controller input frequency in Hz

Yaw axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_FLTE: Yaw axis rate controller input frequency in Hz

Yaw axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_FLTD: Yaw axis rate controller input frequency in Hz

Yaw axis rate controller input frequency in Hz

- Range: 1 100

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

- Range: 0.5 0.9

## ATC_RAT_RLL_FILT: Roll axis rate controller input frequency in Hz

Roll axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_PIT_FILT: Pitch axis rate controller input frequency in Hz

Pitch axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## ATC_RAT_YAW_FILT: Yaw axis rate controller input frequency in Hz

Yaw axis rate controller input frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

# AVOID Parameters

## AVOID_ENABLE: Avoidance control enable/disable

Enabled/disable avoidance input sources

- Bitmask: 0:UseFence,1:UseProximitySensor,2:UseBeaconFence

## AVOID_MARGIN: Avoidance distance margin in GPS modes

Vehicle will attempt to stay at least this distance (in meters) from objects while in GPS modes

- Units: m

- Range: 1 10

## AVOID_BACKUP_SPD: Avoidance maximum backup speed

Maximum speed that will be used to back away from obstacles in GPS modes (m/s). Set zero to disable

- Units: m/s

- Range: 0 2

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

This selects the bus number for looking for an I2C barometer. When set to -1 it will probe all external i2c buses based on the BARO_PROBE_EXT parameter.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Bus0|
|1|Bus1|
|6|Bus6|

## BARO_SPEC_GRAV: Specific Gravity (For water depth measurement)

This sets the specific gravity of the fluid when flying an underwater ROV.

- Values: 1.0:Freshwater,1.024:Saltwater

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

# BAROnWCF Parameters

## BAROn_WCF_ENABLE: Wind coefficient enable

*Note: This parameter is for advanced users*

This enables the use of wind coefficients for barometer compensation

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BAROn_WCF_FWD: Pressure error coefficient in positive X direction (forward)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis. If the baro height estimate rises during forwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_BCOEF_X and EK3_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BAROn_WCF_BCK: Pressure error coefficient in negative X direction (backwards)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis. If the baro height estimate rises during backwards flight, then this will be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_BCOEF_X and EK3_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BAROn_WCF_RGT: Pressure error coefficient in positive Y direction (right)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the right, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_BCOEF_X and EK3_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

## BAROn_WCF_LFT: Pressure error coefficient in negative Y direction (left)

*Note: This parameter is for advanced users*

This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the Y body axis. If the baro height estimate rises during sideways flight to the left, then this should be a negative number. Multirotors can use this feature only if using EKF3 and if the EK3_BCOEF_X and EK3_BCOEF_Y parameters have been tuned.

- Range: -1.0 1.0

- Increment: 0.05

# BATTn Parameters

## BATTn_MONITOR: Battery monitoring

Controls enabling monitoring of the battery's voltage and current

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Analog Voltage Only|
|4|Analog Voltage and Current|
|5|Solo|
|6|Bebop|
|7|SMBus-Generic|
|8|UAVCAN-BatteryInfo|
|9|ESC|
|10|SumOfFollowing|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|

- RebootRequired: True

## BATTn_VOLT_PIN: Battery Voltage sensing pin

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

## BATTn_CURR_PIN: Battery Current sensing pin

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

## BATTn_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT2_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTn_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.

- Units: A/V

## BATTn_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTn_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTn_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With UAVCAN it is the battery_id.

## BATTn_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTn_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTn_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT2_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT2_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTn_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT2_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTn_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATT2_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATT2_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTn_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT2__FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTn_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|2|Disarm|
|3|Enter surface mode|

## BATTn_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|2|Disarm|
|3|Enter surface mode|

## BATTn_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTn_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT2__ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTn_BUS: Battery monitor I2C bus number

Battery monitor I2C bus number

- Range: 0 3

## BATTn_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore UAVCAN SoC

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
|8|UAVCAN-BatteryInfo|
|9|ESC|
|10|SumOfFollowing|
|11|FuelFlow|
|12|FuelLevelPWM|
|13|SMBUS-SUI3|
|14|SMBUS-SUI6|
|15|NeoDesign|
|16|SMBus-Maxell|
|17|Generator-Elec|
|18|Generator-Fuel|
|19|Rotoye|

- RebootRequired: True

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

## BATT_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATT_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With UAVCAN it is the battery_id.

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
|2|Disarm|
|3|Enter surface mode|

## BATT_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|2|Disarm|
|3|Enter surface mode|

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

## BATT_BUS: Battery monitor I2C bus number

Battery monitor I2C bus number

- Range: 0 3

## BATT_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore UAVCAN SoC

# BRD Parameters

## BRD_PWM_COUNT: Auxiliary pin config

*Note: This parameter is for advanced users*

Controls number of FMU outputs which are setup for PWM. All unassigned pins can be used for GPIO

|Value|Meaning|
|:---:|:---:|
|0|No PWMs|
|1|One PWMs|
|2|Two PWMs|
|3|Three PWMs|
|4|Four PWMs|
|5|Five PWMs|
|6|Six PWMs|
|7|Seven PWMs|
|8|Eight PWMs|

- RebootRequired: True

## BRD_SER1_RTSCTS: Serial 1 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 1 (telemetry 1) on Pixhawk. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|

- RebootRequired: True

## BRD_SER2_RTSCTS: Serial 2 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 2 (telemetry 2) on Pixhawk and STATE. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

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

- Range: -32768 32767

## BRD_SAFETY_MASK: Outputs which ignore the safety switch state

*Note: This parameter is for advanced users*

A bitmask which controls what outputs can move while the safety switch has not been pressed

- Bitmask: 0:Output1,1:Output2,2:Output3,3:Output4,4:Output5,5:Output6,6:Output7,7:Output8,8:Output9,9:Output10,10:Output11,11:Output12,12:Output13,13:Output14

- RebootRequired: True

## BRD_IMU_TARGTEMP: Target IMU temperature

*Note: This parameter is for advanced users*

This sets the target IMU temperature for boards with controllable IMU heating units. DO NOT SET to -1 on the Cube. Set to -1 to disable the heater, please reboot after setting to -1.

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

This allows for the IO co-processor on FMUv1 and FMUv2 to be disabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## BRD_SAFETYOPTION: Options for safety button behavior

This controls the activation of the safety button. It allows you to control if the safety button can be used for safety enable and/or disable, and whether the button is only active when disarmed

- Bitmask: 0:ActiveForSafetyEnable,1:ActiveForSafetyDisable,2:ActiveWhenArmed,3:Force safety on when the aircraft disarms

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

- Bitmask: 0:Enable hardware watchdog, 1:Disable MAVftp, 2:Enable set of internal parameters, 3:Enable Debug Pins

## BRD_BOOT_DELAY: Boot delay

*Note: This parameter is for advanced users*

This adds a delay in milliseconds to boot to ensure peripherals initialise fully

- Range: 0 10000

- Units: ms

## BRD_IMUHEAT_P: IMU Heater P gain

*Note: This parameter is for advanced users*

IMU Heater P gain

- Range: 1 500

- Increment: 1

## BRD_IMUHEAT_I: IMU Heater I gain

*Note: This parameter is for advanced users*

IMU Heater integrator gain

- Range: 0 1

- Increment: 0.1

## BRD_IMUHEAT_IMAX: IMU Heater IMAX

*Note: This parameter is for advanced users*

IMU Heater integrator maximum

- Range: 0 100

- Increment: 1

## BRD_ALT_CONFIG: Alternative HW config

*Note: This parameter is for advanced users*

Select an alternative hardware configuration. A value of zero selects the default configuration for this board. Other values are board specific. Please see the documentation for your board for details on any alternative configuration values that may be available.

- Range: 0 10

- Increment: 1

- RebootRequired: True

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

# BTNn Parameters

## BTNn_FUNCTION: Function for button

Set to 0 to disable or choose a function

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|shift|
|2|arm_toggle|
|3|arm|
|4|disarm|
|5|mode_manual|
|6|mode_stabilize|
|7|mode_depth_hold|
|8|mode_poshold|
|9|mode_auto|
|10|mode_circle|
|11|mode_guided|
|12|mode_acro|
|21|mount_center|
|22|mount_tilt_up|
|23|mount_tilt_down|
|24|camera_trigger|
|25|camera_source_toggle|
|26|mount_pan_right|
|27|mount_pan_left|
|31|lights1_cycle|
|32|lights1_brighter|
|33|lights1_dimmer|
|34|lights2_cycle|
|35|lights2_brighter|
|36|lights2_dimmer|
|41|gain_toggle|
|42|gain_inc|
|43|gain_dec|
|44|trim_roll_inc|
|45|trim_roll_dec|
|46|trim_pitch_inc|
|47|trim_pitch_dec|
|48|input_hold_set|
|49|roll_pitch_toggle|
|51|relay_1_on|
|52|relay_1_off|
|53|relay_1_toggle|
|54|relay_2_on|
|55|relay_2_off|
|56|relay_2_toggle|
|57|relay_3_on|
|58|relay_3_off|
|59|relay_3_toggle|
|61|servo_1_inc|
|62|servo_1_dec|
|63|servo_1_min|
|64|servo_1_max|
|65|servo_1_center|
|66|servo_2_inc|
|67|servo_2_dec|
|68|servo_2_min|
|69|servo_2_max|
|70|servo_2_center|
|71|servo_3_inc|
|72|servo_3_dec|
|73|servo_3_min|
|74|servo_3_max|
|75|servo_3_center|
|76|servo_1_min_momentary|
|77|servo_1_max_momentary|
|78|servo_1_min_toggle|
|79|servo_1_max_toggle|
|80|servo_2_min_momentary|
|81|servo_2_max_momentary|
|82|servo_2_min_toggle|
|83|servo_2_max_toggle|
|84|servo_3_min_momentary|
|85|servo_3_max_momentary|
|86|servo_3_min_toggle|
|87|servo_3_max_toggle|
|91|custom_1|
|92|custom_2|
|93|custom_3|
|94|custom_4|
|95|custom_5|
|96|custom_6|
|101|relay_4_on|
|102|relay_4_off|
|103|relay_4_toggle|
|104|relay_1_momentary|
|105|relay_2_momentary|
|106|relay_3_momentary|
|107|relay_4_momentary|

## BTNn_SFUNCTION: Function for button when the shift mode is toggled on

Set to 0 to disable or choose a function

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|shift|
|2|arm_toggle|
|3|arm|
|4|disarm|
|5|mode_manual|
|6|mode_stabilize|
|7|mode_depth_hold|
|8|mode_poshold|
|9|mode_auto|
|10|mode_circle|
|11|mode_guided|
|12|mode_acro|
|21|mount_center|
|22|mount_tilt_up|
|23|mount_tilt_down|
|24|camera_trigger|
|25|camera_source_toggle|
|26|mount_pan_right|
|27|mount_pan_left|
|31|lights1_cycle|
|32|lights1_brighter|
|33|lights1_dimmer|
|34|lights2_cycle|
|35|lights2_brighter|
|36|lights2_dimmer|
|41|gain_toggle|
|42|gain_inc|
|43|gain_dec|
|44|trim_roll_inc|
|45|trim_roll_dec|
|46|trim_pitch_inc|
|47|trim_pitch_dec|
|48|input_hold_set|
|49|roll_pitch_toggle|
|51|relay_1_on|
|52|relay_1_off|
|53|relay_1_toggle|
|54|relay_2_on|
|55|relay_2_off|
|56|relay_2_toggle|
|57|relay_3_on|
|58|relay_3_off|
|59|relay_3_toggle|
|61|servo_1_inc|
|62|servo_1_dec|
|63|servo_1_min|
|64|servo_1_max|
|65|servo_1_center|
|66|servo_2_inc|
|67|servo_2_dec|
|68|servo_2_min|
|69|servo_2_max|
|70|servo_2_center|
|71|servo_3_inc|
|72|servo_3_dec|
|73|servo_3_min|
|74|servo_3_max|
|75|servo_3_center|
|76|servo_1_min_momentary|
|77|servo_1_max_momentary|
|78|servo_1_min_toggle|
|79|servo_1_max_toggle|
|80|servo_2_min_momentary|
|81|servo_2_max_momentary|
|82|servo_2_min_toggle|
|83|servo_2_max_toggle|
|84|servo_3_min_momentary|
|85|servo_3_max_momentary|
|86|servo_3_min_toggle|
|87|servo_3_max_toggle|
|91|custom_1|
|92|custom_2|
|93|custom_3|
|94|custom_4|
|95|custom_5|
|96|custom_6|
|101|relay_4_on|
|102|relay_4_off|
|103|relay_4_toggle|
|104|relay_1_momentary|
|105|relay_2_momentary|
|106|relay_3_momentary|
|107|relay_4_momentary|

# CAM Parameters

## CAM_TRIGG_TYPE: Camera shutter (trigger) type

how to trigger the camera to take a picture

|Value|Meaning|
|:---:|:---:|
|0|Servo|
|1|Relay|
|2|GoPro in Solo Gimbal|

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

pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection. See also the CAM_FEEDBACK_POL option.

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
|4|RunCam Hybrid|

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

# CANDn Parameters

## CAN_Dn_PROTOCOL: Enable use of specific protocol over virtual driver

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UAVCAN|
|3|ToshibaCAN|
|4|PiccoloCAN|
|5|CANTester|
|8|KDECAN|
|9|PacketDigitalCAN|

- RebootRequired: True

# CANDnKDE Parameters

## CAN_Dn_KDE_NPOLE: Number of motor poles

Sets the number of motor poles to calculate the correct RPM value

# CANDnPC Parameters

## CAN_Dn_PC_ESC_BM: ESC channels

*Note: This parameter is for advanced users*

Bitmask defining which ESC (motor) channels are to be transmitted over Piccolo CAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16

## CAN_Dn_PC_ESC_RT: ESC output rate

*Note: This parameter is for advanced users*

Output rate of ESC command messages

- Units: Hz

- Range: 1 500

## CAN_Dn_PC_SRV_BM: Servo channels

*Note: This parameter is for advanced users*

Bitmask defining which servo channels are to be transmitted over Piccolo CAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16

## CAN_Dn_PC_SRV_RT: Servo command output rate

*Note: This parameter is for advanced users*

Output rate of servo command messages

- Units: Hz

- Range: 1 500

# CANDnTST Parameters

## CAN_Dn_TST_ID: CAN Test Index

*Note: This parameter is for advanced users*

Selects the Index of Test that needs to be run recursively, this value gets reset to 0 at boot.

- Range: 0 4

|Value|Meaning|
|:---:|:---:|
|0|TEST_NONE|
|1|TEST_LOOPBACK|
|2|TEST_BUSOFF_RECOVERY|
|3|TEST_UAVCAN_DNA|
|4|TEST_TOSHIBA_CAN|
|5|TEST_KDE_CAN|
|6|TEST_UAVCAN_ESC|

## CAN_Dn_TST_LPR8: CANTester LoopRate

*Note: This parameter is for advanced users*

Selects the Looprate of Test methods

- Units: us

# CANDnUC Parameters

## CAN_Dn_UC_NODE: UAVCAN node that is used for this network

*Note: This parameter is for advanced users*

UAVCAN node should be set implicitly

- Range: 1 250

## CAN_Dn_UC_SRV_BM: RC Out channels to be transmitted as servo over UAVCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a servo command over UAVCAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15

## CAN_Dn_UC_ESC_BM: RC Out channels to be transmitted as ESC over UAVCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16

## CAN_Dn_UC_SRV_RT: Servo output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for servo outputs

- Range: 1 200

- Units: Hz

## CAN_Dn_UC_OPTION: UAVCAN options

*Note: This parameter is for advanced users*

Option flags

- Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts

# CANPn Parameters

## CAN_Pn_DRIVER: Index of virtual driver to be used with physical CAN interface

Enabling this option enables use of CAN buses.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|First driver|
|2|Second driver|

- RebootRequired: True

## CAN_Pn_BITRATE: Bitrate of CAN interface

*Note: This parameter is for advanced users*

Bit rate can be set up to from 10000 to 1000000

- Range: 10000 1000000

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

The orientation of the first external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.

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
|9|Roll180Yaw45|
|10|Roll180Yaw90|
|11|Roll180Yaw135|
|12|Pitch180|
|13|Roll180Yaw225|
|14|Roll180Yaw270|
|15|Roll180Yaw315|
|16|Roll90|
|17|Roll90Yaw45|
|18|Roll90Yaw90|
|19|Roll90Yaw135|
|20|Roll270|
|21|Roll270Yaw45|
|22|Roll270Yaw90|
|23|Roll270Yaw135|
|24|Pitch90|
|25|Pitch270|
|26|Pitch180Yaw90|
|27|Pitch180Yaw270|
|28|Roll90Pitch90|
|29|Roll180Pitch90|
|30|Roll270Pitch90|
|31|Roll90Pitch180|
|32|Roll270Pitch180|
|33|Roll90Pitch270|
|34|Roll180Pitch270|
|35|Roll270Pitch270|
|36|Roll90Pitch180Yaw90|
|37|Roll90Yaw270|
|38|Yaw293Pitch68Roll180|
|39|Pitch315|
|40|Roll90Pitch315|
|100|Custom|

## COMPASS_EXTERNAL: Compass is attached via an external cable

*Note: This parameter is for advanced users*

Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.

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

The orientation of a second external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.

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
|9|Roll180Yaw45|
|10|Roll180Yaw90|
|11|Roll180Yaw135|
|12|Pitch180|
|13|Roll180Yaw225|
|14|Roll180Yaw270|
|15|Roll180Yaw315|
|16|Roll90|
|17|Roll90Yaw45|
|18|Roll90Yaw90|
|19|Roll90Yaw135|
|20|Roll270|
|21|Roll270Yaw45|
|22|Roll270Yaw90|
|23|Roll270Yaw135|
|24|Pitch90|
|25|Pitch270|
|26|Pitch180Yaw90|
|27|Pitch180Yaw270|
|28|Roll90Pitch90|
|29|Roll180Pitch90|
|30|Roll270Pitch90|
|31|Roll90Pitch180|
|32|Roll270Pitch180|
|33|Roll90Pitch270|
|34|Roll180Pitch270|
|35|Roll270Pitch270|
|36|Roll90Pitch180Yaw90|
|37|Roll90Yaw270|
|38|Yaw293Pitch68Roll180|
|39|Pitch315|
|40|Roll90Pitch315|
|100|Custom|

## COMPASS_EXTERN2: Compass2 is attached via an external cable

*Note: This parameter is for advanced users*

Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.

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

The orientation of a third external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.

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
|9|Roll180Yaw45|
|10|Roll180Yaw90|
|11|Roll180Yaw135|
|12|Pitch180|
|13|Roll180Yaw225|
|14|Roll180Yaw270|
|15|Roll180Yaw315|
|16|Roll90|
|17|Roll90Yaw45|
|18|Roll90Yaw90|
|19|Roll90Yaw135|
|20|Roll270|
|21|Roll270Yaw45|
|22|Roll270Yaw90|
|23|Roll270Yaw135|
|24|Pitch90|
|25|Pitch270|
|26|Pitch180Yaw90|
|27|Pitch180Yaw270|
|28|Roll90Pitch90|
|29|Roll180Pitch90|
|30|Roll270Pitch90|
|31|Roll90Pitch180|
|32|Roll270Pitch180|
|33|Roll90Pitch270|
|34|Roll180Pitch270|
|35|Roll270Pitch270|
|36|Roll90Pitch180Yaw90|
|37|Roll90Yaw270|
|38|Yaw293Pitch68Roll180|
|39|Pitch315|
|40|Roll90Pitch315|
|100|Custom|

## COMPASS_EXTERN3: Compass3 is attached via an external cable

*Note: This parameter is for advanced users*

Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.

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

- Bitmask: 0:HMC5883,1:LSM303D,2:AK8963,3:BMM150,4:LSM9DS1,5:LIS3MDL,6:AK09916,7:IST8310,8:ICM20948,9:MMC3416,11:UAVCAN,12:QMC5883,14:MAG3110,15:IST8308,16:RM3100,17:MSP,18:ExternalAHRS

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

# EAHRS Parameters

## EAHRS_TYPE: AHRS type

Type of AHRS device

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|VectorNav|

## EAHRS_RATE: AHRS data rate

Requested rate for AHRS device

- Units: Hz

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

This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.

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

- RebootRequired: False

## EK2_GSF_RUN_MASK: Bitmask of which EKF-GSF yaw estimators run

*Note: This parameter is for advanced users*

1 byte bitmap of which EKF2 instances run an independant EKF-GSF yaw estimator to provide a backup yaw estimate that doesn't rely on magnetometer data. This estimator uses IMU, GPS and, if available, airspeed data. EKF-GSF yaw estimator data for the primary EKF2 instance will be logged as GSF0 and GSF1 messages. Use of the yaw estimate generated by this algorithm is controlled by the EK2_GSF_USE, EK2_GSF_DELAY and EK2_GSF_MAXCOUNT parameters. To run the EKF-GSF yaw estimator in ride-along and logging only, set EK2_GSF_USE to 0. 

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK2_GSF_USE_MASK: Bitmask of which EKF-GSF yaw estimators are used

*Note: This parameter is for advanced users*

1 byte bitmap of which EKF2 instances will use the output from the EKF-GSF yaw estimator that has been turned on by the EK2_GSF_RUN parameter. If the inertial navigation calculation stops following the GPS, then the vehicle code can request EKF2 to attempt to resolve the issue, either by performing a yaw reset if enabled by this parameter by switching to another EKF2 instance. Additionally the EKF2 will  initiate a reset internally if navigation is lost for more than EK2_GSF_DELAY milli seconds.

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK2_GSF_RST_MAX: Maximum number of resets to the EKF-GSF yaw estimate allowed

*Note: This parameter is for advanced users*

Sets the maximum number of times the EKF2 will be allowed to reset it's yaw to the estimate from the EKF-GSF yaw estimator. No resets will be allowed unless the use of the EKF-GSF yaw estimate is enabled via the EK2_GSF_USE parameter.

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

This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.

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

- RebootRequired: False

## EK3_MAG_EF_LIM: EarthField error limit

*Note: This parameter is for advanced users*

This limits the difference between the learned earth magnetic field and the earth field from the world magnetic model tables. A value of zero means to disable the use of the WMM tables.

- Range: 0 500

- Units: mGauss

## EK3_GSF_RUN_MASK: Bitmask of which EKF-GSF yaw estimators run

*Note: This parameter is for advanced users*

1 byte bitmap of which EKF3 instances run an independant EKF-GSF yaw estimator to provide a backup yaw estimate that doesn't rely on magnetometer data. This estimator uses IMU, GPS and, if available, airspeed data. EKF-GSF yaw estimator data for the primary EKF3 instance will be logged as GSF0 and GSF1 messages. Use of the yaw estimate generated by this algorithm is controlled by the EK3_GSF_USE, EK3_GSF_DELAY and EK3_GSF_MAXCOUNT parameters. To run the EKF-GSF yaw estimator in ride-along and logging only, set EK3_GSF_USE to 0. 

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK3_GSF_USE_MASK: Bitmask of which EKF-GSF yaw estimators are used

*Note: This parameter is for advanced users*

1 byte bitmap of which EKF3 instances will use the output from the EKF-GSF yaw estimator that has been turned on by the EK3_GSF_RUN parameter. If the inertial navigation calculation stops following the GPS, then the vehicle code can request EKF3 to attempt to resolve the issue, either by performing a yaw reset if enabled by this parameter by switching to another EKF3 instance. Additionally the EKF3 will  initiate a reset internally if navigation is lost for more than EK3_GSF_DELAY milli seconds.

- Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF

- RebootRequired: True

## EK3_GSF_RST_MAX: Maximum number of resets to the EKF-GSF yaw estimate allowed

*Note: This parameter is for advanced users*

Sets the maximum number of times the EKF3 will be allowed to reset it's yaw to the estimate from the EKF-GSF yaw estimator. No resets will be allowed unless the use of the EKF-GSF yaw estimate is enabled via the EK3_GSF_USE parameter.

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

Ratio of mass to drag coefficient measured along the X body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared.  Set to a postive value > 1.0 to enable. A starting value is the mass in Kg divided by the frontal area. The predicted drag from the rotors is specified separately by the EK3_MCOEF parameter.

- Range: 0.0 1000.0

- Units: kg/m/m

## EK3_DRAG_BCOEF_Y: Ballistic coefficient for Y axis drag

*Note: This parameter is for advanced users*

Ratio of mass to drag coefficient measured along the Y body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared.  Set to a postive value > 1.0 to enable. A starting value is the mass in Kg divided by the side area. The predicted drag from the rotors is specified separately by the EK3_MCOEF parameter.

- Range: 50.0 1000.0

- Units: kg/m/m

## EK3_DRAG_M_NSE: Observation noise for drag acceleration

*Note: This parameter is for advanced users*

This sets the amount of noise used when fusing X and Y acceleration as an observation that enables esitmation of wind velocity for multi-rotor vehicles. This feature is enabled by the EK3_BCOEF_X and EK3_BCOEF_Y parameters

- Range: 0.1 2.0

- Increment: 0.1

- Units: m/s/s

## EK3_DRAG_MCOEF: Momentum coefficient for propeller drag

*Note: This parameter is for advanced users*

This parameter is used to predict the drag produced by the rotors when flying a multi-copter, enabling estimation of wind drift. The drag produced by this effect scales with speed not speed squared and is produced because some of the air velocity normal to the rotors axis of rotation is lost when passing through the rotor disc which changes the momentum of the airflow causing drag. For unducted rotors the effect is roughly proportional to the area of the propeller blades when viewed side on and changes with different propellers. It is higher for ducted rotors. For example if flying at 15 m/s at sea level conditions produces a rotor induced drag acceleration of 1.5 m/s/s, then EK3_MCOEF would be set to 0.1 = (1.5/15.0). Set EK3_MCOEF to a postive value to enable wind estimation using this drag effect. To account for the drag produced by the body which scales with speed squared, see documentation for the EK3_BCOEF_X and EK3_BCOEF_Y parameters.

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

This parameter sets the size of the dead zone that is applied to negative baro height spikes that can occur when takeing off or landing when a vehicle with lift rotors is operating in ground effect ground effect. Set to about 0.5m less than the amount of negative offset in baro height that occurs just prior to takeoff when lift motors are spooling up. Set to 0 if no ground effect is present. 

- Range: 0.0 10.0

- Increment: 0.5

## EK3_PRIMARY: Primary core number

*Note: This parameter is for advanced users*

The core number (index in IMU mask) that will be used as the primary EKF core on startup. While disarmed the EKF will force the use of this core. A value of 0 corresponds to the first IMU in EK3_IMU_MASK.

- Range: 0 2

- Increment: 1

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
|6|UAVCAN|
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

# GPS Parameters

## GPS_TYPE: 1st GPS type

*Note: This parameter is for advanced users*

GPS type of 1st GPS

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|AUTO|
|2|uBlox|
|3|MTK|
|4|MTK19|
|5|NMEA|
|6|SiRF|
|7|HIL|
|8|SwiftNav|
|9|UAVCAN|
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

- RebootRequired: True

## GPS_TYPE2: 2nd GPS type

*Note: This parameter is for advanced users*

GPS type of 2nd GPS

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|AUTO|
|2|uBlox|
|3|MTK|
|4|MTK19|
|5|NMEA|
|6|SiRF|
|7|HIL|
|8|SwiftNav|
|9|UAVCAN|
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
|1|Enable automatic configuration|

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

- Bitmask: 0:Use UART2 for moving baseline on ublox,1:Use base station for GPS yaw on SBF,2:Use baudrate 115200

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

GPS Node id for discovered first.

- ReadOnly: True

## GPS_CAN_NODEID2: GPS Node ID 2

*Note: This parameter is for advanced users*

GPS Node id for discovered second.

- ReadOnly: True

## GPS1_CAN_OVRIDE: First UAVCAN GPS NODE ID

*Note: This parameter is for advanced users*

GPS Node id for first GPS. If 0 the gps will be automatically selected on first come basis.

## GPS2_CAN_OVRIDE: Second UAVCAN GPS NODE ID

*Note: This parameter is for advanced users*

GPS Node id for second GPS. If 0 the gps will be automatically selected on first come basis.

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

## GRIP_REGRAB: Gripper Regrab interval

*Note: This parameter is for advanced users*

Time in seconds that gripper will regrab the cargo to ensure grip has not weakened; 0 to disable

- Range: 0 255

- Units: s

## GRIP_UAVCAN_ID: EPM UAVCAN Hardpoint ID

Refer to https://docs.zubax.com/opengrab_epm_v3#UAVCAN_interface

- Range: 0 255

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

- Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU

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

Bitmask of harmonic frequencies to apply Harmonic Notch Filter to. This option takes effect on the next reboot. A maximum of 3 harmonics can be used at any one time.

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

## INS_HNTCH_OPTS: Harmonic Notch Filter options

*Note: This parameter is for advanced users*

Harmonic Notch Filter options. Double-notches can provide deeper attenuation across a wider bandwidth than single notches and are suitable for larger aircraft. Dynamic harmonics attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz.

- Bitmask: 0:Double notch,1:Dynamic harmonic,2:Update at loop rate

- RebootRequired: True

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

Options for the BatchSampler. Post-filter and sensor-rate logging cannot be used at the same time.

- Bitmask: 0:Sensor-Rate Logging (sample at full sensor rate seen by AP), 1: Sample post-filtering

## INS_LOG_BAT_LGIN: logging interval

Interval between pushing samples to the AP_Logger log

- Units: ms

- Increment: 10

## INS_LOG_BAT_LGCT: logging count

Number of samples to push to count every INS_LOG_BAT_LGIN

- Increment: 1

# INSNOTCH Parameters

## INS_NOTCH_ENABLE: Enable

*Note: This parameter is for advanced users*

Enable notch filter

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_NOTCH_ATT: Attenuation

*Note: This parameter is for advanced users*

Notch attenuation in dB

- Range: 5 30

- Units: dB

## INS_NOTCH_FREQ: Frequency

*Note: This parameter is for advanced users*

Notch center frequency in Hz

- Range: 10 400

- Units: Hz

## INS_NOTCH_BW: Bandwidth

*Note: This parameter is for advanced users*

Notch bandwidth in Hz

- Range: 5 100

- Units: Hz

# INSTCALn Parameters

## INS_TCALn_ENABLE: Enable temperature calibration

*Note: This parameter is for advanced users*

Enable the use of temperature calibration parameters for this IMU. For automatic learning set to 2 and also set the INS_TCALn_TMAX to the target temperature, then reboot

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|LearnCalibration|

- RebootRequired: True

## INS_TCALn_TMIN: Temperature calibration min

*Note: This parameter is for advanced users*

The minimum temperature that the calibration is valid for

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCALn_TMAX: Temperature calibration max

*Note: This parameter is for advanced users*

The maximum temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration

- Range: -70 80

- Units: degC

- Calibration: 1

## INS_TCALn_ACC1_X: Accelerometer 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC1_Y: Accelerometer 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC1_Z: Accelerometer 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC2_X: Accelerometer 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC2_Y: Accelerometer 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC2_Z: Accelerometer 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC3_X: Accelerometer 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC3_Y: Accelerometer 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_ACC3_Z: Accelerometer 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR1_X: Gyroscope 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR1_Y: Gyroscope 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR1_Z: Gyroscope 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR2_X: Gyroscope 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR2_Y: Gyroscope 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR2_Z: Gyroscope 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR3_X: Gyroscope 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR3_Y: Gyroscope 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS_TCALn_GYR3_Z: Gyroscope 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

# LEAK Parameters

## LEAK1_PIN: Pin that leak detector is connected to

Pin that the leak detector is connected to

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|Pixhawk Aux1|
|51|Pixhawk Aux2|
|52|Pixhawk Aux3|
|53|Pixhawk Aux4|
|54|Pixhawk Aux5|
|55|Pixhawk Aux6|
|13|Pixhawk 3.3ADC1|
|14|Pixhawk 3.3ADC2|
|15|Pixhawk 6.6ADC|
|27|Navigator Built-In|

- RebootRequired: True

## LEAK1_LOGIC: Default reading of leak detector when dry

Default reading of leak detector when dry

|Value|Meaning|
|:---:|:---:|
|0|Low|
|1|High|

## LEAK2_PIN: Pin that leak detector is connected to

Pin that the leak detector is connected to

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|Pixhawk Aux1|
|51|Pixhawk Aux2|
|52|Pixhawk Aux3|
|53|Pixhawk Aux4|
|54|Pixhawk Aux5|
|55|Pixhawk Aux6|
|13|Pixhawk 3.3ADC1|
|14|Pixhawk 3.3ADC2|
|15|Pixhawk 6.6ADC|
|27|Navigator Leak1|

- RebootRequired: True

## LEAK2_LOGIC: Default reading of leak detector when dry

Default reading of leak detector when dry

|Value|Meaning|
|:---:|:---:|
|0|Low|
|1|High|

## LEAK3_PIN: Pin that leak detector is connected to

Pin that the leak detector is connected to

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|Pixhawk Aux1|
|51|Pixhawk Aux2|
|52|Pixhawk Aux3|
|53|Pixhawk Aux4|
|54|Pixhawk Aux5|
|55|Pixhawk Aux6|
|13|Pixhawk 3.3ADC1|
|14|Pixhawk 3.3ADC2|
|15|Pixhawk 6.6ADC|
|27|Navigator Leak1|

- RebootRequired: True

## LEAK3_LOGIC: Default reading of leak detector when dry

Default reading of leak detector when dry

|Value|Meaning|
|:---:|:---:|
|0|Low|
|1|High|

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

# LOIT Parameters

## LOIT_ANG_MAX: Loiter Angle Max

*Note: This parameter is for advanced users*

Loiter maximum lean angle. Set to zero for 2/3 of PSC_ANGLE_MAX or ANGLE_MAX

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

- Bitmask: 0:Clear Mission on reboot

# MNT Parameters

## MNT_TYPE: Mount Type

Mount Type (None, Servo or MAVLink)

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Servo|
|2|3DR Solo|
|3|Alexmos Serial|
|4|SToRM32 MAVLink|
|5|SToRM32 Serial|

- RebootRequired: True

## MNT_DEFLT_MODE: Mount default operating mode

Mount default operating mode on startup and after control is returned from autopilot

|Value|Meaning|
|:---:|:---:|
|0|Retracted|
|1|Neutral|
|2|MavLink Targeting|
|3|RC Targeting|
|4|GPS Point|

## MNT_RETRACT_X: Mount roll angle when in retracted position

Mount roll angle when in retracted position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT_RETRACT_Y: Mount tilt/pitch angle when in retracted position

Mount tilt/pitch angle when in retracted position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT_RETRACT_Z: Mount yaw/pan angle when in retracted position

Mount yaw/pan angle when in retracted position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT_NEUTRAL_X: Mount roll angle when in neutral position

Mount roll angle when in neutral position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT_NEUTRAL_Y: Mount tilt/pitch angle when in neutral position

Mount tilt/pitch angle when in neutral position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT_NEUTRAL_Z: Mount pan/yaw angle when in neutral position

Mount pan/yaw angle when in neutral position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT_STAB_ROLL: Stabilize mount's roll angle

enable roll stabilisation relative to Earth

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MNT_STAB_TILT: Stabilize mount's pitch/tilt angle

enable tilt/pitch stabilisation relative to Earth

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MNT_STAB_PAN: Stabilize mount pan/yaw angle

enable pan/yaw stabilisation relative to Earth

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MNT_RC_IN_ROLL: roll RC input channel

0 for none, any other for the RC channel to be used to control roll movements

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|5|RC5|
|6|RC6|
|7|RC7|
|8|RC8|
|9|RC9|
|10|RC10|
|11|RC11|
|12|RC12|

## MNT_ANGMIN_ROL: Minimum roll angle

Minimum physical roll angular position of mount.

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT_ANGMAX_ROL: Maximum roll angle

Maximum physical roll angular position of the mount

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT_RC_IN_TILT: tilt (pitch) RC input channel

0 for none, any other for the RC channel to be used to control tilt (pitch) movements

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|5|RC5|
|6|RC6|
|7|RC7|
|8|RC8|
|9|RC9|
|10|RC10|
|11|RC11|
|12|RC12|

## MNT_ANGMIN_TIL: Minimum tilt angle

Minimum physical tilt (pitch) angular position of mount.

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT_ANGMAX_TIL: Maximum tilt angle

Maximum physical tilt (pitch) angular position of the mount

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT_RC_IN_PAN: pan (yaw) RC input channel

0 for none, any other for the RC channel to be used to control pan (yaw) movements

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|5|RC5|
|6|RC6|
|7|RC7|
|8|RC8|
|9|RC9|
|10|RC10|
|11|RC11|
|12|RC12|

## MNT_ANGMIN_PAN: Minimum pan angle

Minimum physical pan (yaw) angular position of mount.

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT_ANGMAX_PAN: Maximum pan angle

Maximum physical pan (yaw) angular position of the mount

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT_JSTICK_SPD: mount joystick speed

0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.

- Range: 0 100

- Increment: 1

## MNT_LEAD_RLL: Roll stabilization lead time

Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.

- Units: s

- Range: 0.0 0.2

- Increment: .005

## MNT_LEAD_PTCH: Pitch stabilization lead time

Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.

- Units: s

- Range: 0.0 0.2

- Increment: .005

## MNT2_DEFLT_MODE: Mount default operating mode

Mount default operating mode on startup and after control is returned from autopilot

|Value|Meaning|
|:---:|:---:|
|0|Retracted|
|1|Neutral|
|2|MavLink Targeting|
|3|RC Targeting|
|4|GPS Point|

## MNT2_RETRACT_X: Mount2 roll angle when in retracted position

Mount2 roll angle when in retracted position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT2_RETRACT_Y: Mount2 tilt/pitch angle when in retracted position

Mount2 tilt/pitch angle when in retracted position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT2_RETRACT_Z: Mount2 yaw/pan angle when in retracted position

Mount2 yaw/pan angle when in retracted position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT2_NEUTRAL_X: Mount2 roll angle when in neutral position

Mount2 roll angle when in neutral position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT2_NEUTRAL_Y: Mount2 tilt/pitch angle when in neutral position

Mount2 tilt/pitch angle when in neutral position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT2_NEUTRAL_Z: Mount2 pan/yaw angle when in neutral position

Mount2 pan/yaw angle when in neutral position

- Units: deg

- Range: -180.00 179.99

- Increment: 1

## MNT2_STAB_ROLL: Stabilize Mount2's roll angle

enable roll stabilisation relative to Earth

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MNT2_STAB_TILT: Stabilize Mount2's pitch/tilt angle

enable tilt/pitch stabilisation relative to Earth

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MNT2_STAB_PAN: Stabilize mount2 pan/yaw angle

enable pan/yaw stabilisation relative to Earth

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MNT2_RC_IN_ROLL: Mount2's roll RC input channel

0 for none, any other for the RC channel to be used to control roll movements

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|5|RC5|
|6|RC6|
|7|RC7|
|8|RC8|
|9|RC9|
|10|RC10|
|11|RC11|
|12|RC12|

## MNT2_ANGMIN_ROL: Mount2's minimum roll angle

Mount2's minimum physical roll angular position

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT2_ANGMAX_ROL: Mount2's maximum roll angle

Mount2's maximum physical roll angular position

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT2_RC_IN_TILT: Mount2's tilt (pitch) RC input channel

0 for none, any other for the RC channel to be used to control tilt (pitch) movements

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|5|RC5|
|6|RC6|
|7|RC7|
|8|RC8|
|9|RC9|
|10|RC10|
|11|RC11|
|12|RC12|

## MNT2_ANGMIN_TIL: Mount2's minimum tilt angle

Mount2's minimum physical tilt (pitch) angular position

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT2_ANGMAX_TIL: Mount2's maximum tilt angle

Mount2's maximum physical tilt (pitch) angular position

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT2_RC_IN_PAN: Mount2's pan (yaw) RC input channel

0 for none, any other for the RC channel to be used to control pan (yaw) movements

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|5|RC5|
|6|RC6|
|7|RC7|
|8|RC8|
|9|RC9|
|10|RC10|
|11|RC11|
|12|RC12|

## MNT2_ANGMIN_PAN: Mount2's minimum pan angle

Mount2's minimum physical pan (yaw) angular position

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT2_ANGMAX_PAN: Mount2's maximum pan angle

MOunt2's maximum physical pan (yaw) angular position

- Units: cdeg

- Range: -18000 17999

- Increment: 10

## MNT2_LEAD_RLL: Mount2's Roll stabilization lead time

Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.

- Units: s

- Range: 0.0 0.2

- Increment: .005

## MNT2_LEAD_PTCH: Mount2's Pitch stabilization lead time

Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.

- Units: s

- Range: 0.0 0.2

- Increment: .005

## MNT2_TYPE: Mount2 Type

Mount Type (None, Servo or MAVLink)

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Servo|
|2|3DR Solo|
|3|Alexmos Serial|
|4|SToRM32 MAVLink|
|5|SToRM32 Serial|

# MOT Parameters

## MOT_1_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_2_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_3_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_4_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_5_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_6_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_7_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_8_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_FV_CPLNG_K: Forward/vertical to pitch decoupling factor

Used to decouple pitch from forward/vertical motion. 0 to disable, 1.2 normal

- Range: 0.0 1.5

- Increment: 0.1

## MOT_9_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_10_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_11_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

## MOT_12_DIRECTION: Motor normal or reverse

Used to change motor rotation directions without changing wires

|Value|Meaning|
|:---:|:---:|
|1|normal|
|-1|reverse|

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

- Values: 0.9:Low,0.95:Default,1.0:High

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

- Values: 0.0:Low,0.15:Default,0.3:High

## MOT_SPIN_ARM: Motor Spin armed

*Note: This parameter is for advanced users*

Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range.  Should be lower than MOT_SPIN_MIN.

- Values: 0.0:Low,0.1:Default,0.2:High

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

## MOT_SAFE_DISARM: Motor PWM output disabled when disarmed

*Note: This parameter is for advanced users*

Disables motor PWM output when disarmed

|Value|Meaning|
|:---:|:---:|
|0|PWM enabled while disarmed|
|1|PWM disabled while disarmed|

## MOT_YAW_SV_ANGLE: Yaw Servo Max Lean Angle

Yaw servo's maximum lean angle

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

- Bitmask: 0:EnableTelemetryMode

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

- Bitmask: 0:Built-in buzzer, 1:DShot, 2:UAVCAN

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

Enables to connect active buzzer to arbitrary pin. Requires 3-pin buzzer or additional MOSFET!

|Value|Meaning|
|:---:|:---:|
|0|Disabled|

## NTF_LED_TYPES: LED Driver Types

*Note: This parameter is for advanced users*

Controls what types of LEDs will be enabled

- Bitmask: 0:Built-in LED, 1:Internal ToshibaLED, 2:External ToshibaLED, 3:External PCA9685, 4:Oreo LED, 5:UAVCAN, 6:NCP5623 External, 7:NCP5623 Internal, 8:NeoPixel, 9:ProfiLED, 10:Scripting, 11:DShot

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

# PRX Parameters

## PRX_TYPE: Proximity type

What type of proximity sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|7|LightwareSF40c|
|1|LightWareSF40C-legacy|
|2|MAVLink|
|3|TeraRangerTower|
|4|RangeFinder|
|5|RPLidarA2|
|6|TeraRangerTowerEvo|
|8|LightwareSF45B|
|10|SITL|
|12|AirSimSITL|

- RebootRequired: True

## PRX_ORIENT: Proximity sensor orientation

Proximity sensor orientation

|Value|Meaning|
|:---:|:---:|
|0|Default|
|1|Upside Down|

## PRX_YAW_CORR: Proximity sensor yaw correction

Proximity sensor yaw correction

- Units: deg

- Range: -180 180

## PRX_IGN_ANG1: Proximity sensor ignore angle 1

Proximity sensor ignore angle 1

- Units: deg

- Range: 0 360

## PRX_IGN_WID1: Proximity sensor ignore width 1

Proximity sensor ignore width 1

- Units: deg

- Range: 0 127

## PRX_IGN_ANG2: Proximity sensor ignore angle 2

Proximity sensor ignore angle 2

- Units: deg

- Range: 0 360

## PRX_IGN_WID2: Proximity sensor ignore width 2

Proximity sensor ignore width 2

- Units: deg

- Range: 0 127

## PRX_IGN_ANG3: Proximity sensor ignore angle 3

Proximity sensor ignore angle 3

- Units: deg

- Range: 0 360

## PRX_IGN_WID3: Proximity sensor ignore width 3

Proximity sensor ignore width 3

- Units: deg

- Range: 0 127

## PRX_IGN_ANG4: Proximity sensor ignore angle 4

Proximity sensor ignore angle 4

- Units: deg

- Range: 0 360

## PRX_IGN_WID4: Proximity sensor ignore width 4

Proximity sensor ignore width 4

- Units: deg

- Range: 0 127

## PRX_IGN_ANG5: Proximity sensor ignore angle 5

Proximity sensor ignore angle 5

- Units: deg

- Range: 0 360

## PRX_IGN_WID5: Proximity sensor ignore width 5

Proximity sensor ignore width 5

- Units: deg

- Range: 0 127

## PRX_IGN_ANG6: Proximity sensor ignore angle 6

Proximity sensor ignore angle 6

- Units: deg

- Range: 0 360

## PRX_IGN_WID6: Proximity sensor ignore width 6

Proximity sensor ignore width 6

- Units: deg

- Range: 0 127

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

- Bitmask: 0:Ignore RC Receiver, 1:Ignore MAVLink Overrides, 2:Ignore Receiver Failsafe bit but allow other RC failsafes if setup, 3:FPort Pad, 4:Log RC input bytes, 5:Arming check throttle for 0 input, 6:Skip the arming check for neutral Roll/Pitch/Yay sticks, 7:Allow Switch reverse, 8:Use passthrough for CRSF telemetry, 9:Suppress CRSF mode/rate message for ELRS systems, 10:Enable RC Protocol re-detection

## RC_PROTOCOLS: RC protocols enabled

*Note: This parameter is for advanced users*

Bitmask of enabled RC protocols. Allows narrowing the protocol detection to only specific types of RC receivers which can avoid issues with incorrect detection. Set to 1 to enable all protocols.

- Bitmask: 0:All,1:PPM,2:IBUS,3:SBUS,4:SBUS_NI,5:DSM,6:SUMD,7:SRXL,8:SRXL2,9:CRSF,10:ST24,11:FPORT,12:FPORT2

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

# RELAY Parameters

## RELAY_PIN: First Relay Pin

Digital pin number for first relay control. This is the pin used for camera control.

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

## RELAY_PIN2: Second Relay Pin

Digital pin number for 2nd relay control.

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

## RELAY_PIN3: Third Relay Pin

Digital pin number for 3rd relay control.

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

## RELAY_PIN4: Fourth Relay Pin

Digital pin number for 4th relay control.

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

## RELAY_DEFAULT: Default relay state

The state of the relay on boot.

|Value|Meaning|
|:---:|:---:|
|0|Off|
|1|On|
|2|NoChange|

## RELAY_PIN5: Fifth Relay Pin

Digital pin number for 5th relay control.

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

## RELAY_PIN6: Sixth Relay Pin

Digital pin number for 6th relay control.

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

# RNGFNDn Parameters

## RNGFNDn_TYPE: Rangefinder type

What type of rangefinder device that is connected

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
|11|uLanding|
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
|24|UAVCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|100|SITL|

## RNGFNDn_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to.  Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|PX4-airspeed port|
|15|Pixhawk-airspeed port|
|50|Pixhawk AUXOUT1|
|51|Pixhawk AUXOUT2|
|52|Pixhawk AUXOUT3|
|53|Pixhawk AUXOUT4|
|54|Pixhawk AUXOUT5|
|55|Pixhawk AUXOUT6|

## RNGFNDn_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.

- Units: m/V

- Increment: 0.001

## RNGFNDn_OFFSET: rangefinder offset

Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars

- Units: V

- Increment: 0.001

## RNGFNDn_FUNCTION: Rangefinder function

Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.

|Value|Meaning|
|:---:|:---:|
|0|Linear|
|1|Inverted|
|2|Hyperbolic|

## RNGFNDn_MIN_CM: Rangefinder minimum distance

Minimum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFNDn_MAX_CM: Rangefinder maximum distance

Maximum distance in centimeters that rangefinder can reliably read

- Units: cm

- Increment: 1

## RNGFNDn_STOP_PIN: Rangefinder stop pin

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|Pixhawk AUXOUT1|
|51|Pixhawk AUXOUT2|
|52|Pixhawk AUXOUT3|
|53|Pixhawk AUXOUT4|
|54|Pixhawk AUXOUT5|
|55|Pixhawk AUXOUT6|
|111|PX4 FMU Relay1|
|112|PX4 FMU Relay2|
|113|PX4IO Relay1|
|114|PX4IO Relay2|
|115|PX4IO ACC1|
|116|PX4IO ACC2|

## RNGFNDn_RMETRIC: Ratiometric

This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.

|Value|Meaning|
|:---:|:---:|
|0|No|
|1|Yes|

## RNGFNDn_PWRRNG: Powersave range

This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled

- Units: m

- Range: 0 32767

## RNGFNDn_GNDCLEAR: Distance (in cm) from the range finder to the ground

This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.

- Units: cm

- Range: 5 127

- Increment: 1

## RNGFNDn_ADDR: Bus address of sensor

This sets the bus address of the sensor, where applicable. Used for the I2C and UAVCAN sensors to allow for multiple sensors on different addresses.

- Range: 0 127

- Increment: 1

## RNGFNDn_POS_X:  X position offset

*Note: This parameter is for advanced users*

X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFNDn_POS_Y: Y position offset

*Note: This parameter is for advanced users*

Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFNDn_POS_Z: Z position offset

*Note: This parameter is for advanced users*

Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.

- Units: m

- Range: -5 5

- Increment: 0.01

## RNGFNDn_ORIENT: Rangefinder orientation

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

## RNGFNDn_WSP_MAVG: Moving Average Range

*Note: This parameter is for advanced users*

Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results

- Range: 0 255

## RNGFNDn_WSP_MEDF: Moving Median Filter

*Note: This parameter is for advanced users*

Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active

- Range: 0 255

## RNGFNDn_WSP_FRQ: Frequency

*Note: This parameter is for advanced users*

Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.

- Range: 0 10000

## RNGFNDn_WSP_AVG: Multi-pulse averages

*Note: This parameter is for advanced users*

Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement

- Range: 0 255

## RNGFNDn_WSP_THR: Sensitivity threshold

*Note: This parameter is for advanced users*

Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments

- Range: -1 255

## RNGFNDn_WSP_BAUD: Baud rate

*Note: This parameter is for advanced users*

Desired baud rate

|Value|Meaning|
|:---:|:---:|
|0|Low Speed|
|1|High Speed|

# RNGFNDA Parameters

## RNGFNDA_TYPE: Rangefinder type

What type of rangefinder device that is connected

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
|11|uLanding|
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
|24|UAVCAN|
|25|BenewakeTFminiPlus-I2C|
|26|LanbaoPSK-CM8JL65-CC5|
|27|BenewakeTF03|
|28|VL53L1X-ShortRange|
|29|LeddarVu8-Serial|
|30|HC-SR04|
|31|GYUS42v2|
|32|MSP|
|33|USD1_CAN|
|100|SITL|

## RNGFNDA_PIN: Rangefinder pin

Analog or PWM input pin that rangefinder is connected to.  Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|11|PX4-airspeed port|
|15|Pixhawk-airspeed port|
|50|Pixhawk AUXOUT1|
|51|Pixhawk AUXOUT2|
|52|Pixhawk AUXOUT3|
|53|Pixhawk AUXOUT4|
|54|Pixhawk AUXOUT5|
|55|Pixhawk AUXOUT6|

## RNGFNDA_SCALING: Rangefinder scaling

Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.

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

Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range.

|Value|Meaning|
|:---:|:---:|
|-1|Not Used|
|50|Pixhawk AUXOUT1|
|51|Pixhawk AUXOUT2|
|52|Pixhawk AUXOUT3|
|53|Pixhawk AUXOUT4|
|54|Pixhawk AUXOUT5|
|55|Pixhawk AUXOUT6|
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

This sets the bus address of the sensor, where applicable. Used for the I2C and UAVCAN sensors to allow for multiple sensors on different addresses.

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

# RPM Parameters

## RPM_TYPE: RPM type

What type of RPM sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|PWM|
|2|AUXPIN|
|3|EFI|
|4|Harmonic Notch|

## RPM_SCALING: RPM scaling

Scaling factor between sensor reading and RPM.

- Increment: 0.001

## RPM_MAX: Maximum RPM

Maximum RPM to report

- Increment: 1

## RPM_MIN: Minimum RPM

Minimum RPM to report

- Increment: 1

## RPM_MIN_QUAL: Minimum Quality

*Note: This parameter is for advanced users*

Minimum data quality to be used

- Increment: 0.1

## RPM_PIN: Input pin number

Which pin to use

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|PixhawkAUX1|
|51|PixhawkAUX2|
|52|PixhawkAUX3|
|53|PixhawkAUX4|
|54|PixhawkAUX5|
|55|PixhawkAUX6|

## RPM2_TYPE: Second RPM type

*Note: This parameter is for advanced users*

What type of RPM sensor is connected

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|PWM|
|2|AUXPIN|
|3|EFI|
|4|Harmonic Notch|

## RPM2_SCALING: RPM scaling

*Note: This parameter is for advanced users*

Scaling factor between sensor reading and RPM.

- Increment: 0.001

## RPM2_PIN: RPM2 input pin number

Which pin to use

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|50|PixhawkAUX1|
|51|PixhawkAUX2|
|52|PixhawkAUX3|
|53|PixhawkAUX4|
|54|PixhawkAUX5|
|55|PixhawkAUX6|

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

## SCR_DEBUG_LVL: Scripting Debug Level

*Note: This parameter is for advanced users*

The higher the number the more verbose builtin scripting debug will be.

## SCR_USER1: Scripting User Parameter1

General purpose user variable input for scripts

## SCR_USER2: Scripting User Parameter2

General purpose user variable input for scripts

## SCR_USER3: Scripting User Parameter3

General purpose user variable input for scripts

## SCR_USER4: Scripting User Parameter4

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

- RebootRequired: True

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

- RebootRequired: True

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

- RebootRequired: True

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

- RebootRequired: True

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

- RebootRequired: True

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

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

- RebootRequired: True

## SERIAL2_OPTIONS: Telem2 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

- RebootRequired: True

## SERIAL3_OPTIONS: Serial3 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

- RebootRequired: True

## SERIAL4_OPTIONS: Serial4 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

- RebootRequired: True

## SERIAL5_OPTIONS: Serial5 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

- RebootRequired: True

## SERIAL6_OPTIONS: Serial6 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

- RebootRequired: True

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

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

- RebootRequired: True

## SERIAL8_PROTOCOL: Serial8 protocol selection

Control what protocol Serial8 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.

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
|24|MegaSquirt EFI|
|25|LTM|
|26|RunCam|
|27|HottTelem|
|28|Scripting|
|29|Crossfire|
|30|Generator|
|31|Winch|
|32|MSP|
|33|DJI FPV|
|34|AirSpeed|
|35|ADSB|
|36|AHRS|
|37|SmartAudio|
|38|FETtecOneWire|

- RebootRequired: True

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

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO

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
|1|BLHeli32/BLHeli_S/Kiss|

# SERVOn Parameters

## SERVOn_MIN: Minimum PWM

minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.

- Units: PWM

- Range: 500 2200

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
|0|Disabled|
|1|RCPassThru|
|2|Flap|
|3|FlapAuto|
|4|Aileron|
|6|MountPan|
|7|MountTilt|
|8|MountRoll|
|9|MountOpen|
|10|CameraTrigger|
|12|Mount2Pan|
|13|Mount2Tilt|
|14|Mount2Roll|
|15|Mount2Open|
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

# SERVOBLH Parameters

## SERVO_BLH_MASK: BLHeli Channel Bitmask

*Note: This parameter is for advanced users*

Enable of BLHeli pass-thru servo protocol support to specific channels. This mask is in addition to motors enabled using SERVO_BLH_AUTO (if any)

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

## SERVO_BLH_AUTO: BLHeli pass-thru auto-enable for multicopter motors

If set to 1 this auto-enables BLHeli pass-thru support for all multicopter motors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

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

## SERVO_BLH_PORT: Control port

*Note: This parameter is for advanced users*

This sets the serial port to use for blheli pass-thru

|Value|Meaning|
|:---:|:---:|
|0|Console|
|1|Serial1|
|2|Serial2|
|3|Serial3|
|4|Serial4|
|5|Serial5|

## SERVO_BLH_POLES: BLHeli Motor Poles

*Note: This parameter is for advanced users*

This allows calculation of true RPM from ESC's eRPM. The default is 14.

- Range: 1 127

## SERVO_BLH_3DMASK: BLHeli bitmask of 3D channels

*Note: This parameter is for advanced users*

Mask of channels which are dynamically reversible. This is used to configure ESCs in '3D' mode, allowing for the motor to spin in either direction

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

## SERVO_BLH_BDMASK: BLHeli bitmask of bi-directional dshot channels

*Note: This parameter is for advanced users*

Mask of channels which support bi-directional dshot. This is used for ESCs which have firmware that supports bi-directional dshot allowing fast rpm telemetry values to be returned for the harmonic notch.

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

## SERVO_BLH_RVMASK: BLHeli bitmask of reversed channels

*Note: This parameter is for advanced users*

Mask of channels which are reversed. This is used to configure ESCs in reversed mode

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

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

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

# SRn Parameters

## SRn_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

## SRn_EXT_STAT: Extended status stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

## SRn_RC_CHAN: RC Channel stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

## SRn_POSITION: Position stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of GLOBAL_POSITION_INT to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

## SRn_EXTRA1: Extra data type 1 stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

## SRn_EXTRA2: Extra data type 2 stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of VFR_HUD to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

## SRn_EXTRA3: Extra data type 3 stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

## SRn_PARAMS: Parameter stream rate to ground station

*Note: This parameter is for advanced users*

Stream rate of PARAM_VALUE to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

# VISO Parameters

## VISO_TYPE: Visual odometry camera connection type

*Note: This parameter is for advanced users*

Visual odometry camera connection type

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|MAVLink|
|2|IntelT265|

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

Video Transmitter Options. Pitmode puts the VTX in a low power state. Unlocked enables certain restricted frequencies and power levels. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels.

- Bitmask: 0:Pitmode,1:Pitmode until armed,2:Pitmode when disarmed,3:Unlocked,4:Add leading zero byte to requests

## VTX_MAX_POWER: Video Transmitter Max Power Level

Video Transmitter Maximum Power Level. Different VTXs support different power levels, this prevents the power aux switch from requesting too high a power level. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting.

- Range: 25 1000

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