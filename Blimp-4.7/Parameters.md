
This is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your autopilot to control vehicle behaviour. This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code. Some parameters may only be available for developers, and are enabled at compile-time.

# Blimp Parameters

## FORMAT_VERSION: Eeprom format version number

*Note: This parameter is for advanced users*

This value is incremented when changes are made to the eeprom format

## PILOT_THR_FILT: Throttle filter cutoff

*Note: This parameter is for advanced users*

Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable

- Units: Hz

- Range: 0 10

- Increment: 0.5

## PILOT_THR_BHV: Throttle stick behavior

Bitmask containing various throttle stick options. TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.

- Bitmask: 0:Feedback from mid stick,1:High throttle cancels landing,2:Disarm on land detection

## GCS_PID_MASK: GCS PID tuning mask

*Note: This parameter is for advanced users*

bitmask of PIDs to send MAVLink PID_TUNING messages for

- Bitmask: 0:VELX,1:VELY,2:VELZ,3:VELYAW,4:POSX,5:POSY,6:POZ,7:POSYAW

## FS_GCS_ENABLE: Ground Station Failsafe Enable

Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. See FS_OPTIONS param for additional actions, or for cases allowing Mission continuation, when GCS failsafe is enabled.

|Value|Meaning|
|:---:|:---:|
|0|Disabled/NoAction|
|5|Land|

## GPS_HDOP_GOOD: GPS Hdop Good

*Note: This parameter is for advanced users*

GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks

- Range: 100 900

## FS_THR_ENABLE: Throttle Failsafe Enable

The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|3|Enabled always Land|

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

Flight mode when Channel 5 pwm is <= 1230

|Value|Meaning|
|:---:|:---:|
|0|LAND|
|1|MANUAL|
|2|VELOCITY|
|3|LOITER|

## FLTMODE2: Flight Mode 2

Flight mode when Channel 5 pwm is >1230, <= 1360

|Value|Meaning|
|:---:|:---:|
|0|LAND|
|1|MANUAL|
|2|VELOCITY|
|3|LOITER|

## FLTMODE3: Flight Mode 3

Flight mode when Channel 5 pwm is >1360, <= 1490

|Value|Meaning|
|:---:|:---:|
|0|LAND|
|1|MANUAL|
|2|VELOCITY|
|3|LOITER|

## FLTMODE4: Flight Mode 4

Flight mode when Channel 5 pwm is >1490, <= 1620

|Value|Meaning|
|:---:|:---:|
|0|LAND|
|1|MANUAL|
|2|VELOCITY|
|3|LOITER|

## FLTMODE5: Flight Mode 5

Flight mode when Channel 5 pwm is >1620, <= 1749

|Value|Meaning|
|:---:|:---:|
|0|LAND|
|1|MANUAL|
|2|VELOCITY|
|3|LOITER|

## FLTMODE6: Flight Mode 6

Flight mode when Channel 5 pwm is >=1750

|Value|Meaning|
|:---:|:---:|
|0|LAND|
|1|MANUAL|
|2|VELOCITY|
|3|LOITER|

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

## INITIAL_MODE: Initial flight mode

*Note: This parameter is for advanced users*

This selects the mode to start in on boot.

|Value|Meaning|
|:---:|:---:|
|0|LAND|
|1|MANUAL|
|2|VELOCITY|
|3|LOITER|

## LOG_BITMASK: Log bitmask

Bitmap of what log types to enable in on-board logger. This value is made up of the sum of each of the log types you want to be saved. On boards supporting microSD cards or other large block-storage devices it is usually best just to enable all basic log types by setting this to 65535.

- Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System Performance,4:Control Tuning,6:RC Input,7:IMU,9:Battery Monitor,10:RC Output,12:PID,13:Compass

## DISARM_DELAY: Disarm delay

*Note: This parameter is for advanced users*

Delay before automatic disarm in seconds. A value of zero disables auto disarm.

- Units: s

- Range: 0 127

## FS_EKF_ACTION: EKF Failsafe Action

*Note: This parameter is for advanced users*

Controls the action that will be taken when an EKF failsafe is invoked

|Value|Meaning|
|:---:|:---:|
|1|Land|
|3|Land even in MANUAL|

## FS_EKF_THRESH: EKF failsafe variance threshold

*Note: This parameter is for advanced users*

Allows setting the maximum acceptable compass and velocity variance

|Value|Meaning|
|:---:|:---:|
|0.6|Strict|
|0.8|Default|
|1.0|Relaxed|

- Range: 0.6 1.0

## FS_CRASH_CHECK: Crash check enable

*Note: This parameter is for advanced users*

This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MAX_VEL_XY: Max XY Velocity

Sets the maximum XY velocity, in m/s

- Range: 0.2 5

## MAX_VEL_Z: Max Z Velocity

Sets the maximum Z velocity, in m/s

- Range: 0.2 5

## MAX_VEL_YAW: Max yaw Velocity

Sets the maximum yaw velocity, in rad/s

- Range: 0.2 5

## MAX_POS_XY: Max XY Position change

Sets the maximum XY position change, in m/s

- Range: 0.1 5

## MAX_POS_Z: Max Z Position change

Sets the maximum Z position change, in m/s

- Range: 0.1 5

## MAX_POS_YAW: Max Yaw Position change

Sets the maximum Yaw position change, in rad/s

- Range: 0.1 5

## SIMPLE_MODE: Simple mode

Simple mode for Position control - "forward" moves blimp in +ve X direction world-frame

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## DIS_MASK: Disable output mask

Mask for disabling (setting to zero) one or more of the 4 output axis in mode Velocity or Loiter

- Bitmask: 0:Right,1:Front,2:Down,3:Yaw

## PID_DZ: Deadzone for the position PIDs

Output 0 thrust signal when blimp is within this distance (in meters) of the target position. Warning: If this param is greater than MAX_POS_XY param then the blimp won't move at all in the XY plane in Loiter mode as it does not allow more than a second's lag. Same for the other axes.

- Units: m

- Range: 0.1 1

## RC_SPEED: ESC Update Speed

*Note: This parameter is for advanced users*

This is the speed in Hertz that your ESCs will receive updates

- Units: Hz

- Range: 50 490

- Increment: 1

## VELXY_P: Velocity (horizontal) P gain

*Note: This parameter is for advanced users*

Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration

- Range: 0.1 6.0

- Increment: 0.1

## VELXY_I: Velocity (horizontal) I gain

*Note: This parameter is for advanced users*

Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration

- Range: 0.02 1.00

- Increment: 0.01

## VELXY_D: Velocity (horizontal) D gain

*Note: This parameter is for advanced users*

Velocity (horizontal) D gain.  Corrects short-term changes in velocity

- Range: 0.00 1.00

- Increment: 0.001

## VELXY_IMAX: Velocity (horizontal) integrator maximum

*Note: This parameter is for advanced users*

Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output

- Range: 0 4500

- Increment: 10

- Units: cm/s/s

## VELXY_FLTE: Velocity (horizontal) input filter

*Note: This parameter is for advanced users*

Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## VELXY_FLTD: Velocity (horizontal) input filter

*Note: This parameter is for advanced users*

Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term

- Range: 0 100

- Units: Hz

## VELXY_FF: Velocity (horizontal) feed forward gain

*Note: This parameter is for advanced users*

Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration

- Range: 0 6

- Increment: 0.01

## VELZ_P: Velocity (vertical) P gain

*Note: This parameter is for advanced users*

Velocity (vertical) P gain.  Converts the difference between desired and actual velocity to a target acceleration

- Range: 0.1 6.0

- Increment: 0.1

## VELZ_I: Velocity (vertical) I gain

*Note: This parameter is for advanced users*

Velocity (vertical) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration

- Range: 0.02 1.00

- Increment: 0.01

## VELZ_D: Velocity (vertical) D gain

*Note: This parameter is for advanced users*

Velocity (vertical) D gain.  Corrects short-term changes in velocity

- Range: 0.00 1.00

- Increment: 0.001

## VELZ_IMAX: Velocity (vertical) integrator maximum

*Note: This parameter is for advanced users*

Velocity (vertical) integrator maximum.  Constrains the target acceleration that the I gain will output

- Range: 0 4500

- Increment: 10

- Units: cm/s/s

## VELZ_FLTE: Velocity (vertical) input filter

*Note: This parameter is for advanced users*

Velocity (vertical) input filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## VELZ_FLTD: Velocity (vertical) input filter

*Note: This parameter is for advanced users*

Velocity (vertical) input filter.  This filter (in Hz) is applied to the input for D term

- Range: 0 100

- Units: Hz

## VELZ_FF: Velocity (vertical) feed forward gain

*Note: This parameter is for advanced users*

Velocity (vertical) feed forward gain.  Converts the difference between desired velocity to a target acceleration

- Range: 0 6

- Increment: 0.01

## VELYAW_P: Velocity (yaw) P gain

*Note: This parameter is for advanced users*

Velocity (yaw) P gain.  Converts the difference between desired and actual velocity to a target acceleration

- Range: 0.1 6.0

- Increment: 0.1

## VELYAW_I: Velocity (yaw) I gain

*Note: This parameter is for advanced users*

Velocity (yaw) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration

- Range: 0.02 1.00

- Increment: 0.01

## VELYAW_D: Velocity (yaw) D gain

*Note: This parameter is for advanced users*

Velocity (yaw) D gain.  Corrects short-term changes in velocity

- Range: 0.00 1.00

- Increment: 0.001

## VELYAW_IMAX: Velocity (yaw) integrator maximum

*Note: This parameter is for advanced users*

Velocity (yaw) integrator maximum.  Constrains the target acceleration that the I gain will output

- Range: 0 4500

- Increment: 10

- Units: cm/s/s

## VELYAW_FLTE: Velocity (yaw) input filter

*Note: This parameter is for advanced users*

Velocity (yaw) input filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## VELYAW_FF: Velocity (yaw) feed forward gain

*Note: This parameter is for advanced users*

Velocity (yaw) feed forward gain.  Converts the difference between desired velocity to a target acceleration

- Range: 0 6

- Increment: 0.01

## POSXY_P: Position (horizontal) P gain

*Note: This parameter is for advanced users*

Position (horizontal) P gain.  Converts the difference between desired and actual position to a target velocity

- Range: 0.1 6.0

- Increment: 0.1

## POSXY_I: Position (horizontal) I gain

*Note: This parameter is for advanced users*

Position (horizontal) I gain.  Corrects long-term difference between desired and actual position to a target velocity

- Range: 0.02 1.00

- Increment: 0.01

## POSXY_D: Position (horizontal) D gain

*Note: This parameter is for advanced users*

Position (horizontal) D gain.  Corrects short-term changes in position

- Range: 0.00 1.00

- Increment: 0.001

## POSXY_IMAX: Position (horizontal) integrator maximum

*Note: This parameter is for advanced users*

Position (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output

- Range: 0 4500

- Increment: 10

- Units: cm/s/s

## POSXY_FLTE: Position (horizontal) input filter

*Note: This parameter is for advanced users*

Position (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## POSXY_FLTD: Position (horizontal) input filter

*Note: This parameter is for advanced users*

Position (horizontal) input filter.  This filter (in Hz) is applied to the input for D term

- Range: 0 100

- Units: Hz

## POSXY_FF: Position (horizontal) feed forward gain

*Note: This parameter is for advanced users*

Position (horizontal) feed forward gain.  Converts the difference between desired position to a target velocity

- Range: 0 6

- Increment: 0.01

## POSZ_P: Position (vertical) P gain

*Note: This parameter is for advanced users*

Position (vertical) P gain.  Converts the difference between desired and actual position to a target velocity

- Range: 0.1 6.0

- Increment: 0.1

## POSZ_I: Position (vertical) I gain

*Note: This parameter is for advanced users*

Position (vertical) I gain.  Corrects long-term difference between desired and actual position to a target velocity

- Range: 0.02 1.00

- Increment: 0.01

## POSZ_D: Position (vertical) D gain

*Note: This parameter is for advanced users*

Position (vertical) D gain.  Corrects short-term changes in position

- Range: 0.00 1.00

- Increment: 0.001

## POSZ_IMAX: Position (vertical) integrator maximum

*Note: This parameter is for advanced users*

Position (vertical) integrator maximum.  Constrains the target acceleration that the I gain will output

- Range: 0 4500

- Increment: 10

- Units: cm/s/s

## POSZ_FLTE: Position (vertical) input filter

*Note: This parameter is for advanced users*

Position (vertical) input filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## POSZ_FLTD: Position (vertical) input filter

*Note: This parameter is for advanced users*

Position (vertical) input filter.  This filter (in Hz) is applied to the input for D term

- Range: 0 100

- Units: Hz

## POSZ_FF: Position (vertical) feed forward gain

*Note: This parameter is for advanced users*

Position (vertical) feed forward gain.  Converts the difference between desired position to a target velocity

- Range: 0 6

- Increment: 0.01

## POSYAW_P: Position (yaw) axis controller P gain

Position (yaw) axis controller P gain.

- Range: 0.0 3.0

- Increment: 0.01

## POSYAW_I: Position (yaw) axis controller I gain

Position (yaw) axis controller I gain.

- Range: 0.0 3.0

- Increment: 0.01

## POSYAW_IMAX: Position (yaw) axis controller I gain maximum

Position (yaw) axis controller I gain maximum.

- Range: 0 4000

- Increment: 10

- Units: d%

## POSYAW_D: Position (yaw) axis controller D gain

Position (yaw) axis controller D gain.

- Range: 0.001 0.1

- Increment: 0.001

## POSYAW_FF: Position (yaw) axis controller feed forward

Position (yaw) axis controller feed forward

- Range: 0 0.5

- Increment: 0.001

## POSYAW_FLTT: Position (yaw) target frequency filter in Hz

Position (yaw) target frequency filter in Hz

- Range: 1 50

- Increment: 1

- Units: Hz

## POSYAW_FLTE: Position (yaw) error frequency filter in Hz

Position (yaw) error frequency filter in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## POSYAW_FLTD: Position (yaw) derivative input filter in Hz

Position (yaw) derivative input filter in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## POSYAW_SMAX: Yaw slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains.

- Range: 0 200

- Increment: 0.5

## POSYAW_PDMX: Position (yaw) axis controller PD sum maximum

*Note: This parameter is for advanced users*

Position (yaw) axis controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output

- Range: 0 4000

- Increment: 10

- Units: d%

## POSYAW_D_FF: Position (yaw) Derivative FeedForward Gain

*Note: This parameter is for advanced users*

FF D Gain which produces an output that is proportional to the rate of change of the target

- Range: 0 0.1

- Increment: 0.001

## POSYAW_NTF: Position (yaw) Target notch filter index

*Note: This parameter is for advanced users*

Position (yaw) Target notch filter index

- Range: 1 8

## POSYAW_NEF: Position (yaw) Error notch filter index

*Note: This parameter is for advanced users*

Position (yaw) Error notch filter index

- Range: 1 8

## DEV_OPTIONS: Development options

*Note: This parameter is for advanced users*

Bitmask of developer options. The meanings of the bit fields in this parameter may vary at any time. Developers should check the source code for current meaning

- Bitmask: 0:Unknown

## FRAME_CLASS: Frame Class

Controls major frame class for blimp.

|Value|Meaning|
|:---:|:---:|
|0|Finnedblimp|

- RebootRequired: True

## PILOT_SPEED_DN: Pilot maximum vertical speed descending

The maximum vertical descending velocity the pilot may request in cm/s

- Units: cm/s

- Range: 50 500

- Increment: 10

## FS_VIBE_ENABLE: Vibration Failsafe enable

This enables the vibration failsafe which will use modified altitude estimation and control during high vibrations

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## FS_OPTIONS: Failsafe options bitmask

*Note: This parameter is for advanced users*

Bitmask of additional options for battery, radio, & GCS failsafes. 0 (default) disables all options.

- Bitmask: 4:Continue if in pilot controlled modes on GCS failsafe

## FS_GCS_TIMEOUT: GCS failsafe timeout

Timeout before triggering the GCS failsafe

- Units: s

- Range: 2 120

- Increment: 1

# Lua Script Parameters

## TERR_BRK_ENABLE: terrain brake enable

terrain brake enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## TERR_BRK_ALT: terrain brake altitude

terrain brake altitude. The altitude above the ground below which BRAKE mode will be engaged if in LOITER mode.

- Range: 1 100

- Units: m

## TERR_BRK_HDIST: terrain brake home distance

terrain brake home distance. The distance from home where the auto BRAKE will be enabled. When within this distance of home the script will not activate

- Range: 0 1000

- Units: m

## TERR_BRK_SPD: terrain brake speed threshold

terrain brake speed threshold. Don't trigger BRAKE if both horizontal speed and descent rate are below this threshold. By setting this to a small value this can be used to allow the user to climb up to a safe altitude in LOITER mode. A value of 0.5 is recommended if you want to use LOITER to recover from an emergency terrain BRAKE mode change.

- Range: 0 5

- Units: m/s

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

## POI_DIST_MAX: Mount POI distance max

POI's max distance (in meters) from the vehicle

- Range: 0 10000

## BATT_SOC_COUNT: Count of SOC estimators

Number of battery SOC estimators

- Range: 0 4

## BATT_SOC1_IDX: Battery estimator index

Battery estimator index

- Range: 0 4

## BATT_SOC1_NCELL: Battery estimator cell count

Battery estimator cell count

- Range: 0 48

## BATT_SOC1_C1: Battery estimator coefficient1

Battery estimator coefficient1

- Range: 100 200

## BATT_SOC1_C2: Battery estimator coefficient2

Battery estimator coefficient2

- Range: 2 5

## BATT_SOC1_C3: Battery estimator coefficient3

Battery estimator coefficient3

- Range: 0.01 0.5

## BATT_SOC1_C4: Battery estimator coefficient4

Battery estimator coefficient4

- Range: 5 100

## BATT_SOC2_IDX: Battery estimator index

Battery estimator index

- Range: 0 4

## BATT_SOC2_NCELL: Battery estimator cell count

Battery estimator cell count

- Range: 0 48

## BATT_SOC2_C1: Battery estimator coefficient1

Battery estimator coefficient1

- Range: 100 200

## BATT_SOC2_C2: Battery estimator coefficient2

Battery estimator coefficient2

- Range: 2 5

## BATT_SOC2_C3: Battery estimator coefficient3

Battery estimator coefficient3

- Range: 0.01 0.5

## BATT_SOC2_C4: Battery estimator coefficient4

Battery estimator coefficient4

- Range: 5 100

## BATT_SOC3_IDX: Battery estimator index

Battery estimator index

- Range: 0 4

## BATT_SOC3_NCELL: Battery estimator cell count

Battery estimator cell count

- Range: 0 48

## BATT_SOC3_C1: Battery estimator coefficient1

Battery estimator coefficient1

- Range: 100 200

## BATT_SOC3_C2: Battery estimator coefficient2

Battery estimator coefficient2

- Range: 2 5

## BATT_SOC3_C3: Battery estimator coefficient3

Battery estimator coefficient3

- Range: 0.01 0.5

## BATT_SOC3_C4: Battery estimator coefficient4

Battery estimator coefficient4

- Range: 5 100

## BATT_SOC4_IDX: Battery estimator index

Battery estimator index

- Range: 0 4

## BATT_SOC4_NCELL: Battery estimator cell count

Battery estimator cell count

- Range: 0 48

## BATT_SOC4_C1: Battery estimator coefficient1

Battery estimator coefficient1

- Range: 100 200

## BATT_SOC4_C2: Battery estimator coefficient2

Battery estimator coefficient2

- Range: 2 5

## BATT_SOC4_C3: Battery estimator coefficient3

Battery estimator coefficient3

- Range: 0.01 0.5

## BATT_SOC4_C4: Battery estimator coefficient4

Battery estimator coefficient4

- Range: 5 100

## ARM_SYSID: MAV_SYSID must be set

Check that MAV_SYSID (or SYDID_THISMAV) has been set. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_FOLL_SYSID: FOLL_SYSID must be set

If FOLL_ENABLE = 1, check that FOLL_SYSID has been set. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_FOLL_SYSID_X: Vehicle should not follow itself

If FOLL_ENABLE = 1, check that FOLL_SYSID is different to MAV_SYSID. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_FOLL_OFS_DEF: Follow Offsets defaulted

Follow offsets should not be left as default (zero) if FOLL_ENABLE = 1. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_MNTX_SYSID: Follow and Mount should follow the same vehicle

If FOLL_ENABLE = 1 and MNTx_SYSID_DEFLT is set, check that FOLL_SYSID is equal MNTx. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_RTL_CLIMB: RTL_CLIMB_MIN should be a valid value

RTL_CLIMB_MIN should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_ESTOP: Motors EStopped

Emergency Stop disables arming. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_FENCE: Fence not enabled

Fences loaded but no fence enabled. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_RALLY: Rally too far

Rally Point more than RALLY_LIMIT_KM kilometers away. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_C_RTL_ALT: RTL_ALT should be a valid value

RTL_ALT should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_P_Q_FS_LAND: Warn if Q failsafe will land

Notify the user that on failsafe a QuadPlan will land. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_P_Q_FS_RTL: Warn if Q failsafe will QRTL

Notify the user that on failsafe a QuadPlan will QRTL. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_P_AIRSPEED: Check AIRSPEED_ parameters

Validate that AIRSPEED_STALL(if set) < MIN < CRUISE < MAX d. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_P_STALL: AIRSPEED_MIN should be 25% above STALL

Validate that AIRSPEED_MIN is at least 25% above AIRSPEED_STALL(if set). 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_P_SCALING: SCALING_SPEED valid

Validate that SCALING_SPEED is within 20% of AIRSPEED_CRUISE. If SCALING_SPEED changes the vehicle may need to be retuned. 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_P_RTL_ALT: RTL_ALTITUDE should be a valid value

RTL_ALTITITUDE should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_P_QRTL_ALT: Q_RTL_ALT should be a valid value

Q_RTL_ALT should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|0|Emergency(PreArm)|
|1|Alert(PreArm)|
|2|Critical(PreArm)|
|3|Error(PreArm)|
|4|Warning|
|5|Notice|
|6|Info|
|7|Debug|

## ARM_V_ALT_LEGAL: Legal max altitude

Legal max altitude for UAV/RPAS/drones in your jurisdiction

- Units: m

## ALAND_ENABLE: Auto land enable

enable Auto land script action

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ALAND_WP_ALT: Final approach waypoint alt

Altitude of final approach waypoint created by script

- Range: 1 100

- Units: m

## ALAND_WP_DIST: Final approach waypoint distance

Distance from landing point (HOME) to final approach waypoint created by script in the opposite direction of initial takeoff

- Range: 0 1000

- Units: m

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

## QUIK_OPTIONS: Quicktune options

Additional options. When the Two Position Switch option is enabled then a high switch position will start the tune, low will disable the tune. you should also set a QUIK_AUTO_SAVE time so that you will be able to save the tune.

- Bitmask: 0:UseTwoPositionSwitch

## QUIK_ANGLE_MAX: maximum angle error for tune abort

If while tuning the angle error goes over this limit then the tune will aborts to prevent a bad oscillation in the case of the tuning algorithm failing. If you get an error "Tuning: attitude error ABORTING" and you think it is a false positive then you can either raise this parameter or you can try increasing the QUIK_DOUBLE_TIME to do the tune more slowly. A value of zero disables this check.

- Units: deg

## CAM1_THERM_PAL: Camera1 Thermal Palette

thermal image colour palette

|Value|Meaning|
|:---:|:---:|
|-1|Leave Unchanged|
|0|WhiteHot|
|2|Sepia|
|3|IronBow|
|4|Rainbow|
|5|Night|
|6|Aurora|
|7|RedHot|
|8|Jungle|
|9|Medical|
|10|BlackHot|
|11|GloryHot|

## CAM1_THERM_GAIN: Camera1 Thermal Gain

thermal image temperature range

|Value|Meaning|
|:---:|:---:|
|-1|Leave Unchanged|
|0|LowGain (50C to 550C)|
|1|HighGain (-20C to 150C)|

## CAM1_THERM_RAW: Camera1 Thermal Raw Data

save images with raw temperatures

|Value|Meaning|
|:---:|:---:|
|-1|Leave Unchanged|
|0|Disabled (30fps)|
|1|Enabled (25 fps)|

- Units: m

## FOLLP_FAIL_MODE: Plane Follow lost target mode

Mode to switch to if the target is lost (no signal or > FOLL_DIST_MAX).

## FOLLP_EXIT_MODE: Plane Follow exit mode

Mode to switch to when follow mode is exited normally

## FOLLP_ACT_FN: Plane Follow Scripting ActivationFunction

Setting an RC channel's _OPTION to this value will use it for Plane Follow enable/disable

- Range: 300 307

## FOLLP_TIMEOUT: Plane Follow Telemetry Timeout

How long to try reaquire a target if telemetry from the lead vehicle is lost.

- Range: 0 30

- Units: s

## FOLLP_OVRSHT_DEG: Plane Follow Scripting Overshoot Angle

If the target is greater than this many degrees left or right, assume an overshoot

- Range: 0 180

- Units: deg

## FOLLP_TURN_DEG: Plane Follow Scripting Turn Angle

If the target is greater than this many degrees left or right, assume it's turning

- Range: 0 180

- Units: deg

## FOLLP_DIST_CLOSE: Plane Follow Scripting Close Distance

When closer than this distance assume we track by heading

- Range: 0 100

- Units: m

## FOLLP_WIDE_TURNS: Plane Follow Scripting Wide Turns

Use wide turns when following a turning target. Alternative is "cutting the corner"

- Range: 0 1

## FOLLP_ALT: Plane Follow Scripting Altitude Override

When non zero, this altitude value (in FOLL_ALT_TYPE frame) overrides the value sent by the target vehicle

- Range: 0 1000

- Units: m

## FOLLP_D_P: Plane Follow Scripting distance P gain

P gain for the speed PID controller distance component

- Range: 0 1

## FOLLP_D_I: Plane Follow Scripting distance I gain

I gain for the speed PID  distance component

- Range: 0 1

## FOLLP_D_D: Plane Follow Scripting distance D gain

D gain for the speed PID controller distance component

- Range: 0 1

## FOLLP_V_P: Plane Follow Scripting speed P gain

P gain for the speed PID controller velocity component

- Range: 0 1

## FOLLP_V_I: Plane Follow Scripting speed I gain

I gain for the speed PID controller velocity component

- Range: 0 1

## FOLLP_V_D: Plane Follow Scripting speed D gain

D gain for the speed PID controller velocity component

- Range: 0 1

## FOLLP_LkAHD: Plane Follow Lookahead seconds

Time to "lookahead" when calculating distance errors

- Units: s

## FOLLP_SIM_TLF_FN: Plane Follow Simulate Telemetry fail function

Set this switch to simulate losing telemetry from the other vehicle

- Range: 300 307

## FOLLP_XT_P: Plane Follow crosstrack PID controller P term

P term for the crosstrack/heading PID controller

- Range: 0 1

## FOLLP_XT_I: Plane Follow crosstrack PID controller I term

I term for the crosstrack/heading PID controller

- Range: 0 1

## FOLLP_XT_D: Plane Follow crosstrack PID controller D term

D term for the crosstrack/heading PID controller

- Range: 0 1

## FOLLP_XT_MAX: Plane Follow crosstrack PID controller maximum correction

maximum correction retured by the crosstrack/heading PID controller

- Range: 0 1

- Units: deg

## FOLLP_XT_I_MAX: Plane Follow crosstrack PID controller maximum integral

maximum I applied the crosstrack/heading PID controller

- Range: 0 100

- Units: ms

## FOLLP_REFRESH: Plane Follow refresh rate

refresh rate for Plane Follow updates

- Range: 0 0.2

- Units: s

## ESRC_EXTN_THRESH: EKF Source ExternalNav Innovation Threshold

ExternalNav may be used if innovations are below this threshold

- Range: 0 1

## ESRC_EXTN_QUAL: EKF Source ExternalNav Quality Threshold

ExternalNav may be used if quality is above this threshold

- Range: 0 100

- Units: %

## ESRC_FLOW_THRESH: EKF Source OpticalFlow Innovation Threshold

OpticalFlow may be used if innovations are below this threshold

- Range: 0 1

## ESRC_FLOW_QUAL: EKF Source OpticalFlow Quality Threshold

OpticalFlow may be used if quality is above this threshold

- Range: 0 100

- Units: %

## ESRC_RNGFND_MAX: EKF Source Rangefinder Max

OpticalFlow may be used if rangefinder distance is below this threshold

- Range: 0 50

- Units: m

## SLUP_ENABLE: Slung Payload enable

Slung Payload enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## SLUP_VEL_P: Slung Payload Velocity P gain

Slung Payload Velocity P gain, higher values will result in faster movements in sync with payload

- Range: 0 0.8

## SLUP_DIST_MAX: Slung Payload horizontal distance max

Oscillation is suppressed when vehicle and payload are no more than this distance horizontally.  Set to 0 to always suppress

- Range: 0 30

## SLUP_SYSID: Slung Payload mavlink system id

Slung Payload mavlink system id.  0 to use any/all system ids

- Range: 0 255

## SLUP_WP_POS_P: Slung Payload return to WP position P gain

WP position P gain. higher values will result in vehicle moving more quickly back to the original waypoint

- Range: 0 1

## SLUP_RESTOFS_TC: Slung Payload resting offset estimate filter time constant

payload's position estimator's time constant used to compensate for GPS errors and wind.  Higher values result in smoother estimate but slower response

- Range: 1 20

## SLUP_DEBUG: Slung Payload debug output

Slung payload debug output, set to 1 to enable debug

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

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

## RK9_FORCEHL: Force enable High Latency mode

Automatically enables High Latency mode if not already enabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Enabled on telemetry loss|

## RK9_PERIOD: Update rate

When in High Latency mode, send Rockblock updates every N seconds

- Range: 0 600

- Units: s

## RK9_DEBUG: Display Rockblock debugging text

Sends Rockblock debug text to GCS via statustexts

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## RK9_ENABLE: Enable Message transmission

Enables the Rockblock sending and recieving

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## RK9_TIMEOUT: GCS timeout to start sendin Rockblock messages

If RK9_FORCEHL=2, this is the number of seconds of GCS timeout until High Latency mode is auto-enabled

- Range: 0 600

- Units: s

## RK9_SERPORT: Rockblock Serial Port

Serial port number to which the Rockblock is connected.This is the index of the SERIALn_ ports that are set to 28 for "scripting"

- Range: 0 8

## RK9_SCRPORT: Rockblock Scripting Serial Port

Scripting Serial port number to which the Rockblock is connected for HL2 messages This is the index of the SCR_SDEV ports that are set to 2 for "MavlinkHL"

- Range: 0 8

## RK9_RELAY: Rockblock Power Relay

RELAYn output to control Rockblock power. This connects to I_EN on the Rockblock header.

- Range: 1 8

## RK9_BOOTED: Rockblock booted feedback

SERVOn GPIO channel that reads the Rockblock booted state. This connects to I_BTD on the Rockblock header. Requires SERVON_FUNCTION=-1

- Range: 50 110

## PARAM_SET_ENABLE: Param Set enable

Param Set enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## AHRS_ORIG_LAT: AHRS/EKF Origin Latitude

AHRS/EKF origin will be set to this latitude if not already set

- Range: -180 180

## AHRS_ORIG_LON: AHRS/EKF Origin Longitude

AHRS/EKF origin will be set to this longitude if not already set

- Range: -180 180

## AHRS_ORIG_ALT: AHRS/EKF Origin Altitude

AHRS/EKF origin will be set to this altitude (in meters above sea level) if not already set

- Range: 0 10000

## WEB_ENABLE: enable web server

enable web server

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## WEB_BIND_PORT: web server TCP port

web server TCP port

- Range: 1 65535

## WEB_DEBUG: web server debugging

*Note: This parameter is for advanced users*

web server debugging

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## WEB_BLOCK_SIZE: web server block size

*Note: This parameter is for advanced users*

web server block size for download

- Range: 1 65535

## WEB_TIMEOUT: web server timeout

*Note: This parameter is for advanced users*

timeout for inactive connections

- Units: s

- Range: 0.1 60

## WEB_SENDFILE_MIN: web server minimum file size for sendfile

*Note: This parameter is for advanced users*

sendfile is an offloading mechanism for faster file download. If this is non-zero and the file is larger than this size then sendfile will be used for file download

- Range: 0 10000000

## CGA_RATIO: CoG adjustment ratio

*Note: This parameter is for advanced users*

The ratio between the front and back motor outputs during steady-state hover. Positive when the CoG is in front of the motors midpoint (front motors work harder).

- Range: 0.5 2

## PREV_ENABLE: parameter reversion enable

Enable parameter reversion system

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## PREV_RC_FUNC: param reversion RC function

RCn_OPTION number to used to trigger parameter reversion

## TA_ACT_FN: Activation Function for Terrain Avoidance

Setting an RC channel's _OPTION to this value will use it for Terrain Avoidance enable/disable

- Range: 300 307

## TA_PTCH_DWN_MIN: down distance minimum for Pitching

If the downward distance is less than this value then start Pitching up to gain altitude.

- Units: m

## TA_PTCH_FWD_MIN: forward distance minimum for Pitching

If the farwardward distance is less than this value then start Pitching up to gain altitude.

- Units: m

## TA_QUAD_DWN_MIN: Downward distance minimum Quading

If the downward distance is less than this value then start Quading up to gain altitude.

- Units: m

## TA_QUAD_FWD_MIN: minimum forward distance for Quading

If the farwardward distance is less than this value then start Quading up to gain altitude.

- Units: m

## TA_PTCH_GSP_MIN: minimum ground speed for Pitching

Minimum Groundspeed (not airspeed) to be flying for Pitching to be used.

- Units: m/s

## TA_PTCH_TIMEOUT: timeout Pitching

Minimum down or forward distance must be triggered for more than this many seconds to start Pitching

- Units: s

## TA_HOME_DIST: safe distance around home

Terrain avoidance will not be applied if the vehicle is less than this distance from home

- Units: m

## TA_ALT_MAX: ceiling for pitching/quading

This is a limit on how high the terrain avoidane will take the vehicle. It acts a failsafe to prevent vertical flyaways.

- Range: 20 1000

- Units: m

## TA_GSP_MAX: Maximum Groundspeed

This is a limit on how fast in groundspeeed terrain avoidance will take the vehicle. This is to allow for reliable sensor readings. -1 for disabled.

- Range: 10 40

- Units: m/s

## TA_GSP_AIRBRAKE: Groudspeed Airbrake limt

This is the limit for triggering airbrake to slow groundspeed as a difference between the airspeed and groundspeed. -1 for disabled.

- Range: -1 -10

- Units: m/s

## TA_CMTC_HGT: CMTC Height

The minimum Height above terrain to maintain when following an AUTO mission or RTL. If zero(0) use TA_PTCH_DOW_MIN.

- Units: m

## TA_CMTC_ENABLE: CMTC Enable

Whether to enable Can't Make That Climb while running Terrain Avoidance

- Range: 0 1

## TA_UPDATE_RATE: Frequency to process avoidance

Avoidance processing rate

- Units: Hz

## TA_CMTC_RAD: CMTC loiter radius

Use this radius for the loiter when trying to gain altitude. If not set or <=0 use WP_LOITER_RAD

- Units: m

## VID1_CAMMODEL: Camera1 Video Stream Camera Model

Video stream camera model

|Value|Meaning|
|:---:|:---:|
|0|Unknown|
|1|Siyi A8|
|2|Siyi ZR10|
|3|Siyi ZR30|
|4|Siyi ZT30 Zoom|
|5|Siyi ZT30 Wide|
|6|Siyi ZT30 IR|
|7|Siyi ZT6 RGB|
|8|Siyi ZT6 IR|
|9|Herelink WifiAP|
|10|Herelink USB-tethering|
|11|Topotek 1080p|
|12|Topotek 480p|
|13|Viewpro|

## VID1_ID: Camera1 Video Stream Id

Video stream id

- Range: 0 50

## VID1_TYPE: Camera1 Video Stream Type

Video stream type

|Value|Meaning|
|:---:|:---:|
|0|RTSP|
|1|RTPUDP|
|2|TCP_MPEG|
|3|MPEG_TS|

## VID1_FLAG: Camera1 Video Stream Flags

Video stream flags

- Bitmask: 0:Running,1:Thermal,2:Thermal Range Enabled

## VID1_FRAME_RATE: Camera1 Video Stream Frame Rate

Video stream frame rate

- Range: 0 50

## VID1_HRES: Camera1 Video Stream Horizontal Resolution

Video stream horizontal resolution

- Range: 0 4096

## VID1_VRES: Camera1 Video Stream Vertical Resolution

Video stream vertical resolution

- Range: 0 4096

## VID1_BITRATE: Camera1 Video Stream Bitrate

Video stream bitrate

- Range: 0 10000

## VID1_HFOV: Camera1 Video Stream Horizontal FOV

Video stream horizontal FOV

- Range: 0 360

## VID1_ENCODING: Camera1 Video Stream Encoding

Video stream encoding

|Value|Meaning|
|:---:|:---:|
|0|Unknown|
|1|H264|
|2|H265|

## VID1_IPADDR0: Camera1 Video Stream IP Address 0

Video stream IP Address first octet

- Range: 0 255

## VID1_IPADDR1: Camera1 Video Stream IP Address 1

Video stream IP Address second octet

- Range: 0 255

## VID1_IPADDR2: Camera1 Video Stream IP Address 2

Video stream IP Address third octet

- Range: 0 255

## VID1_IPADDR3: Camera1 Video Stream IP Address 3

Video stream IP Address fourth octet

- Range: 0 255

## VID1_IPPORT: Camera1 Video Stream IP Address Port

Video stream IP Address Port

- Range: 0 65535

## BTAG_ENABLE: enable battery info support

enable battery info support

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BTAG_MAX_CYCLES: max battery cycles

max battery cycles for arming

- Range: 0 10000

## BTAG_CUR_CYCLES: current battery cycles

*Note: This parameter is for advanced users*

this is the highest value for battery cycles for all connected batteries

- Range: 0 10000

## THR_KILL_FUNC: AUX function to kill engine

AUX function to kill engine. This can be activated either with a RCn_OPTION and a R/C switch or with a ground station auxilliary function

- Range: 300 307

## THR_KILL_PWM: PWM on kill active

PWM on kill active

- Range: 800 2200

## THR_KILL_CHAN: output channel to change on throttle kill

output channel to change on throttle kill, a value of zero disables the feature

- Range: 0 32

## THR_KILL_VAL: auxilliary value to kill throttle

auxilliary value to kill throttle. Set to 2 to kill the throttle when the auxilliary is high. Set to 0 to kill when auxilliary is low

- Range: 0 2

## THR_KILL_DEF: throttle kill default value

throttle kill default value. The default auxilliary function position on boot

- Range: 0 2

## RCK_FORCEHL: Force enable High Latency mode

Automatically enables High Latency mode if not already enabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Enabled on telemetry loss|

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

## RCK_TIMEOUT: GCS timeout to start sendin Rockblock messages

If RCK_FORCEHL=2, this is the number of seconds of GCS timeout until High Latency mode is auto-enabled

- Range: 0 600

- Units: s

## FOLT_ENABLE: Follow Target Send Enable

Follow Target Send Enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## FOLT_MAV_CHAN: Follow Target Send MAVLink Channel

MAVLink channel to which FOLLOW_TARGET should be sent

- Range: 0 10

## PLND_ALT_CUTOFF: Precland altitude cutoff

The altitude (rangefinder distance) below which we stop using the precision landing sensor and continue landing

- Range: 0 20

- Units: m

## DIST_CUTOFF: Precland distance cutoff

The distance from target beyond which the target is ignored

- Range: 0 100

- Units: m

## RTUN_ENABLE: Rover Quicktune enable

Enable quicktune system

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## RTUN_AXES: Rover Quicktune axes

axes to tune

- Bitmask: 0:Steering,1:Speed

## RTUN_STR_FFRATIO: Rover Quicktune Steering Rate FeedForward ratio

Ratio between measured response and FF gain. Raise this to get a higher FF gain

- Range: 0 1.0

## RTUN_STR_P_RATIO: Rover Quicktune Steering FF to P ratio

Ratio between steering FF and P gains. Raise this to get a higher P gain, 0 to leave P unchanged

- Range: 0 2.0

## RTUN_STR_I_RATIO: Rover Quicktune Steering FF to I ratio

Ratio between steering FF and I gains. Raise this to get a higher I gain, 0 to leave I unchanged

- Range: 0 2.0

## RTUN_SPD_FFRATIO: Rover Quicktune Speed FeedForward (equivalent) ratio

Ratio between measured response and CRUISE_THROTTLE value. Raise this to get a higher CRUISE_THROTTLE value

- Range: 0 1.0

## RTUN_SPD_P_RATIO: Rover Quicktune Speed FF to P ratio

Ratio between speed FF and P gain. Raise this to get a higher P gain, 0 to leave P unchanged

- Range: 0 2.0

## RTUN_SPD_I_RATIO: Rover Quicktune Speed FF to I ratio

Ratio between speed FF and I gain. Raise this to get a higher I gain, 0 to leave I unchanged

- Range: 0 2.0

## RTUN_AUTO_FILTER: Rover Quicktune auto filter enable

When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## RTUN_AUTO_SAVE: Rover Quicktune auto save

Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune

- Units: s

## RTUN_RC_FUNC: Rover Quicktune RC function

RCn_OPTION number to use to control tuning stop/start/save

|Value|Meaning|
|:---:|:---:|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|

## RTUN_SPEED_MIN: Rover Quicktune minimum speed for tuning

The mimimum speed in m/s required for tuning to start

- Units: m/s

- Range: 0.1 0.5

## WINCH_RATE_UP: WinchControl Rate Up

Maximum rate when retracting line

- Range: 0.1 5.0

## WINCH_RATE_DN: WinchControl Rate Down

Maximum rate when releasing line

- Range: 0.1 5.0

## WINCH_RC_FUNC: Winch Rate Control RC function

RCn_OPTION number to use to control winch rate

|Value|Meaning|
|:---:|:---:|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|

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

## EFI_DLA_ENABLE: EFI DLA enable

Enable EFI DLA driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_DLA_LPS: EFI DLA fuel scale

EFI DLA litres of fuel per second of injection time

- Range: 0.00001 1

- Units: litres

## LTE_ENABLE: LTE Enable

Enable or disable the LTE modem driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## LTE_SERPORT: Serial Port

Serial port to use for the LTE modem. This is the index of the SERIALn_ ports that are set to 28 for "scripting"

- Range: 0 8

## LTE_SCRPORT: Scripting Serial Port

Scripting Serial port to use for the LTE modem. This is the index of the SCR_SDEV ports that are set to 2 for "MAVLink2"

- Range: 0 8

## LTE_SERVER_IP0: Server IP 0

First octet of the server IP address to connect to

- Range: 0 255

## LTE_SERVER_IP1: Server IP 1

Second octet of the server IP address to connect to

- Range: 0 255

## LTE_SERVER_IP2: Server IP 2

Third octet of the server IP address to connect to

- Range: 0 255

## LTE_SERVER_IP3: Server IP 3

Fourth octet of the server IP address to connect to

- Range: 0 255

## LTE_SERVER_PORT: Server Port

IPv4 Port of the server to connect to

- Range: 1 65525

## LTE_BAUD: Serial Baud Rate

Baud rate for the serial port to the LTE modem when connected. Initial power on baudrate is in LTE_IBAUD

|Value|Meaning|
|:---:|:---:|
|19200|19200|
|38400|38400|
|57600|57600|
|115200|115200|
|230400|230400|
|460800|460800|
|921600|921600|
|3686400|3686400|

## LTE_TIMEOUT: Timeout

Timeout in seconds for the LTE connection. If no data is received for this time, the connection will be reset. A value of zero disables the timeout

- Range: 0 60

- Units: s

## LTE_PROTOCOL: LTE protocol

The protocol that we will use in communication with the LTE modem. If this is PPP then the LTE_SERVER parameters are not used and instead a PPP connection will be established and you should use the NET_ parameters to enable network ports. If this is MAVLink2 then the LTE_SERVER parameters are used to create a TCP or UDP connection to a single server.

|Value|Meaning|
|:---:|:---:|
|2|MavLink2|
|48|PPP|

## LTE_OPTIONS: LTE options

Options to control the LTE modem driver. If VerboseSignalInfoGCS is set then additional NAMED_VALUE_FLOAT values are sent with verbose signal information

- Bitmask: 0:LogAllData,1:VerboseSignalInfoGCS,2:DisableMultiplexing,3:DisableSignalQueries,4:UseTCP

## LTE_IBAUD: LTE initial baudrate

This is the initial baud rate on power on for the modem. This is set in the modem with the AT+IREX=baud command

|Value|Meaning|
|:---:|:---:|
|19200|19200|
|38400|38400|
|57600|57600|
|115200|115200|
|230400|230400|
|460800|460800|
|921600|921600|
|3686400|3686400|

## LTE_MCCMNC: LTE operator selection

This allows selection of network operator

|Value|Meaning|
|:---:|:---:|
|-1|NoChange|
|0|Default|
|50501|AU-Telstra|
|50502|AU-Optus|
|50503|AU-Vodafone|

## LTE_TX_RATE: Max transmit rate

Maximum data transmit rate to the modem in bytes/second. Use zero for unlimited

## LTE_BAND: LTE band selection

This allows selection of LTE band. A value of -1 means no band setting change is made. A value of 0 sets all bands. Otherwise the specified band is set.

- Range: -1 50

## ESC_HW_ENABLE: Hobbywing ESC Enable

Enable Hobbywing ESC telemetry

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ESC_HW_POLES: Hobbywing ESC motor poles

Number of motor poles for eRPM scaling

- Range: 1 50

## ESC_HW_OFS: Hobbywing ESC motor offset

Motor number offset of first ESC

- Range: 0 31

## EFI_HFE_ENABLE: Enable HFE EFI driver

Enable HFE EFI driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_HFE_RATE_HZ: HFI EFI Update rate

HFI EFI Update rate

- Range: 0 400

## EFI_HFE_ECU_IDX: HFI EFI ECU index

HFI EFI ECU index, 0 for automatic

- Range: 0 10

## EFI_HFE_FUEL_DTY: HFI EFI fuel density

HFI EFI fuel density in gram per litre

- Range: 0 2000

## EFI_HFE_REL_IDX: HFI EFI relay index

HFI EFI relay index

- Range: 0 10

## EFI_HFE_CANDRV: HFI EFI CAN driver

HFI EFI CAN driver

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|1stCANDriver|
|2|2ndCanDriver|

## EFI_HFE_OPTIONS: HFI EFI options

HFI EFI options

- Bitmask: 1:EnableCANLogging

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

## EFI_H6K_OPTIONS: Halo6000 options

Halo6000 options

- Bitmask: 0:LogAllCanPackets

## UM_SERVO_MASK: Mask of UltraMotion servos

Mask of UltraMotion servos

- Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12

## UM_CANDRV: Set CAN driver

Set CAN driver

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|1stCANDriver|
|2|2ndCanDriver|

## UM_RATE_HZ: Update rate for UltraMotion servos

Update rate for UltraMotion servos

- Units: Hz

- Range: 1 400

## UM_OPTIONS: Optional settings

Optional settings

- Bitmask: 0:LogAllFrames,1:ParseTelemetry,2:SendPosAsNamedValueFloat

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

## EFI_SP_MODEL: SkyPower EFI ECU model

SkyPower EFI ECU model

|Value|Meaning|
|:---:|:---:|
|0|SRE_180|
|1|SP_275|

## EFI_SP_GEN_CTRL: SkyPower EFI enable generator control

SkyPower EFI enable generator control

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_SP_RST_TIME: SkyPower EFI restart time

SkyPower EFI restart time. If engine should be running and it has stopped for this amount of time then auto-restart. To disable this feature set this value to zero.

- Range: 0 10

- Units: s

## EFI_2K_ENABLE: Enable NMEA 2000 EFI driver

Enable NMEA 2000 EFI driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_2K_CANDRV: NMEA 2000 CAN driver

NMEA 2000 CAN driver. Use 1 for first CAN scripting driver, 2 for 2nd driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|FirstCAN|
|2|SecondCAN|

## EFI_2K_OPTIONS: NMEA 2000 options

NMEA 2000 driver options

- Bitmask: 0:EnableLogging

## EFI_DLA64_ENABLE: EFI DLA64 enable

Enable EFI DLA64 driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## TRQL_ENABLE: Torqeedo TorqLink Enable

Torqeedo TorqLink Enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## TRQL_DEBUG: Torqeedo TorqLink Debug Level

Torqeedo TorqLink Debug Level

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Low|
|2|Medium|
|3|High|

## EFI_SVF_ENABLE: Generator SVFFI enable

Enable SVFFI generator support

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_SVF_ARMCHECK: Generator SVFFI arming check

Check for Generator ARM state before arming

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BATT_ANX_ENABLE: Enable ANX battery support

Enable ANX battery support

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## BATT_ANX_CANDRV: Set ANX CAN driver

Set ANX CAN driver

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|1stCANDriver|
|2|2ndCanDriver|

## BATT_ANX_INDEX: ANX CAN battery index

ANX CAN battery index

- Range: 1 10

## BATT_ANX_OPTIONS: ANX CAN battery options

*Note: This parameter is for advanced users*

ANX CAN battery options

- Bitmask: 0:LogAllFrames

## ENABLE: Enable this script

When set to 0 this script will not run. When set to 1 this script will run.

- Range: 0 1

## BATT_IDX: Index of assigned battery.

Ensure this battery is configured with `BATT*_MONITOR=29`.

## CFACT: Measurement correction factor

This is multiplicative factor to correct the measured flow. Set to <1 if your sensor measures too high.

## MODE: Sensor operating mode

0: The sensor will save the fuel consumption across power resets. 1: The sensor will reset the power consumption.

## PORT: Scripting serial port number

Which Scripting serial port the sensor is connected at.

- Range: 1 10

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

## EFI_INF_ENABLE: EFI INF-Inject enable

Enable EFI INF-Inject driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_INF_OPTIONS: EFI INF-Inject options

EFI INF driver options

- Bitmask: 0:EnableLogging

## EFI_INF_THR_HZ: EFI INF-Inject throttle rate

EFI INF throttle output rate

- Range: 0 50

- Units: Hz

## EFI_INF_IGN_AUX: EFI INF-Inject ignition aux function

EFI INF throttle ignition aux function

## TOFSENSE_S1_PRX: TOFSENSE-M to be used as Proximity sensor

Set 0 if sensor is to be used as a 1-D rangefinder (minimum of all distances will be sent, typically used for height detection). Set 1 if it should be used as a 3-D proximity device (Eg. Obstacle Avoidance)

|Value|Meaning|
|:---:|:---:|
|0|Set as Rangefinder|
|1|Set as Proximity sensor|

## TOFSENSE_S1_SP: TOFSENSE-M serial port config

UART instance sensor is connected to. Set 1 if sensor is connected to the port with fist SERIALx_PROTOCOL = 28.

- Range: 1 4

## TOFSENSE_S1_BR: TOFSENSE-M serial port baudrate

Serial Port baud rate. Sensor baud rate can be changed from Nassistant software

## TOFSENSE_PRX: TOFSENSE-M to be used as Proximity sensor

Set 0 if sensor is to be used as a 1-D rangefinder (minimum of all distances will be sent, typically used for height detection). Set 1 if it should be used as a 3-D proximity device (Eg. Obstacle Avoidance)

|Value|Meaning|
|:---:|:---:|
|0|Set as Rangefinder|
|1|Set as Proximity sensor|

## TOFSENSE_NO: TOFSENSE-M Connected

Number of TOFSENSE-M CAN sensors connected

- Range: 1 3

## TOFSENSE_MODE: TOFSENSE-M mode to be used

TOFSENSE-M mode to be used. 0 for 8x8 mode. 1 for 4x4 mode

|Value|Meaning|
|:---:|:---:|
|0|8x8 mode|
|1|4x4 mode|

## TOFSENSE_INST1: TOFSENSE-M First Instance

First TOFSENSE-M sensors backend Instance. Setting this to 1 will pick the first backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)

- Range: 1 3

## TOFSENSE_ID1: TOFSENSE-M First ID

First TOFSENSE-M sensor ID. Leave this at 0 to accept all IDs and if only one sensor is present. You can change ID of sensor from NAssistant Software

- Range: 1 255

## TOFSENSE_INST2: TOFSENSE-M Second Instance

Second TOFSENSE-M sensors backend Instance. Setting this to 2 will pick the second backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)

- Range: 1 3

## TOFSENSE_ID2: TOFSENSE-M Second ID

Second TOFSENSE-M sensor ID. This cannot be 0. You can change ID of sensor from NAssistant Software

- Range: 1 255

## TOFSENSE_INST3: TOFSENSE-M Third Instance

Third TOFSENSE-M sensors backend Instance. Setting this to 3 will pick the second backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)

- Range: 1 3

## TOFSENSE_ID3: TOFSENSE-M Thir ID

Third TOFSENSE-M sensor ID. This cannot be 0. You can change ID of sensor from NAssistant Software

- Range: 1 255

# AHRS Parameters

## AHRS_GPS_GAIN: AHRS GPS gain

*Note: This parameter is for advanced users*

This controls how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.

- Range: 0.0 1.0

- Increment: 0.01

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

- Increment: 0.01

## AHRS_RP_P: AHRS RP_P

*Note: This parameter is for advanced users*

This controls how fast the accelerometers correct the attitude

- Range: 0.1 0.4

- Increment: 0.01

## AHRS_WIND_MAX: Maximum wind

*Note: This parameter is for advanced users*

This sets the maximum allowable difference between ground speed and airspeed. A value of zero means to use the airspeed as is. This allows the plane to cope with a failing airspeed sensor by clipping it to groundspeed plus/minus this limit. See ARSPD_OPTIONS and ARSPD_WIND_MAX to disable airspeed sensors.

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

- Increment: 0.01

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
|10|Sim|
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

## AHRS_OPTIONS: Optional AHRS behaviour

*Note: This parameter is for advanced users*

This controls optional AHRS behaviour. Setting DisableDCMFallbackFW will change the AHRS behaviour for fixed wing aircraft in fly-forward flight to not fall back to DCM when the EKF stops navigating. Setting DisableDCMFallbackVTOL will change the AHRS behaviour for fixed wing aircraft in non fly-forward (VTOL) flight to not fall back to DCM when the EKF stops navigating. Setting DontDisableAirspeedUsingEKF disables the EKF based innovation check for airspeed consistency

- Bitmask: 0:DisableDCMFallbackFW, 1:DisableDCMFallbackVTOL, 2:DontDisableAirspeedUsingEKF

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

Allow arm/disarm by rudder input. When enabled arming can be done with right rudder, disarming with left rudder. Rudder arming only works with throttle at zero +- deadzone (RCx_DZ). Depending on vehicle type, arming in certain modes is prevented. See the wiki for each vehicle. Caution is recommended when arming if it is allowed in an auto-throttle mode!

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|ArmingOnly|
|2|ArmOrDisarm|

## ARMING_MIS_ITEMS: Required mission items

*Note: This parameter is for advanced users*

Bitmask of mission items that are required to be planned in order to arm the aircraft

- Bitmask: 0:Land,1:VTOL Land,2:DO_LAND_START,3:Takeoff,4:VTOL Takeoff,5:Rallypoint,6:RTL

## ARMING_OPTIONS: Arming options

*Note: This parameter is for advanced users*

Options that can be applied to change arming behaviour

- Bitmask: 0:Disable prearm display,1:Do not send status text on state change,2:Skip IMU consistency checks when ICE motor running

## ARMING_MAGTHRESH: Compass magnetic field strength error threshold vs earth magnetic model

*Note: This parameter is for advanced users*

Compass magnetic field strength error threshold vs earth magnetic model.  X and y axis are compared using this threhold, Z axis uses 2x this threshold.  0 to disable check

- Units: mGauss

- Range: 0 500

## ARMING_CRSDP_IGN: Disable CrashDump Arming check

*Note: This parameter is for advanced users*

Must have value "1" if crashdump data is present on the system, or a prearm failure will be raised.  Do not set this parameter unless the risks of doing so are fully understood.  The presence of a crash dump means that the firmware currently installed has suffered a critical software failure which resulted in the autopilot immediately rebooting.  The crashdump file gives diagnostic information which can help in finding the issue, please contact the ArduPIlot support team.  If this crashdump data is present, the vehicle is likely unsafe to fly.  Check the ArduPilot documentation for more details.

|Value|Meaning|
|:---:|:---:|
|0|Crash Dump arming check active|
|1|Crash Dump arming check deactivated|

## ARMING_SKIPCHK: Arm Checks to Skip (bitmask)

Checks to skip prior to arming motor. This is a bitmask of checks before allowing arming that will be skipped. For most users it is recommended to leave this at the default of 0 (no checks skipped). In extreme circumstances, a value of -1 can be used to skip all non-mandatory current and future checks.

- Bitmask: 1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC Channels,7:Board voltage,8:Battery Level,10:Logging Available,11:Hardware safety switch,12:GPS Configuration,13:System,14:Mission,15:Rangefinder,16:Camera,17:AuxAuth,18:VisualOdometry,19:FFT

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
|1|2nd Sensor|
|2|3rd Sensor|
|3|4th Sensor|
|4|5th Sensor|
|5|6th Sensor|

## ARSPD_OPTIONS: Airspeed options bitmask

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle. Always set to 0.

- Bitmask: 0:SpeedMismatchDisable, 1:AllowSpeedMismatchRecovery, 2:DisableVoltageCorrection, 3:UseEkf3Consistency, 4:ReportOffset

## ARSPD_WIND_MAX: Maximum airspeed and ground speed difference

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle. Always set to 0.

- Units: m/s

## ARSPD_WIND_WARN: Airspeed and GPS speed difference that gives a warning

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle. Always set to 0.

- Units: m/s

## ARSPD_WIND_GATE: Re-enable Consistency Check Gate Size

*Note: This parameter is for advanced users*

This parameter and function is not used by this vehicle.

- Range: 0.0 10.0

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
|16|ExternalAHRS|
|17|AUAV-10in|
|18|AUAV-5in|
|19|AUAV-30in|
|100|SITL|

## ARSPD2_USE: Airspeed use

This parameter is not used by this vehicle. Always set to 0.

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

The pin number that the airspeed sensor is connected to for analog sensors. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## ARSPD2_AUTOCAL: This parameter and function is not used by this vehicle. Always set to 0.

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

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration or requiring a manual calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Do not require offset calibration before flight. Manual calibration should be performed during initial setup.|
|2|Do not calibrate on start up. Manual calibration must be performed once per boot.|

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
|3|Bus3|

- RebootRequired: True

## ARSPD2_DEVID: Airspeed ID

*Note: This parameter is for advanced users*

Airspeed sensor ID, taking into account its type, bus and instance

- ReadOnly: True

# ARSPD3 Parameters

## ARSPD3_TYPE: Airspeed type

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
|16|ExternalAHRS|
|17|AUAV-10in|
|18|AUAV-5in|
|19|AUAV-30in|
|100|SITL|

## ARSPD3_USE: Airspeed use

This parameter is not used by this vehicle. Always set to 0.

|Value|Meaning|
|:---:|:---:|
|0|DoNotUse|
|1|Use|
|2|UseWhenZeroThrottle|

## ARSPD3_OFFSET: Airspeed offset

*Note: This parameter is for advanced users*

Airspeed calibration offset

- Increment: 0.1

## ARSPD3_RATIO: Airspeed ratio

*Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.

- Increment: 0.1

## ARSPD3_PIN: Airspeed pin

*Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## ARSPD3_AUTOCAL: This parameter and function is not used by this vehicle. Always set to 0.

*Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD3_TUBE_ORDR: Control pitot tube order

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD3_SKIP_CAL: Skip airspeed offset calibration on startup

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration or requiring a manual calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Do not require offset calibration before flight. Manual calibration should be performed during initial setup.|
|2|Do not calibrate on start up. Manual calibration must be performed once per boot.|

## ARSPD3_PSI_RANGE: The PSI range of the device

*Note: This parameter is for advanced users*

This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD3_BUS: Airspeed I2C bus

*Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected. May not correspond to board's I2C bus number labels. Retry another bus and reboot if airspeed sensor fails to initialize.

|Value|Meaning|
|:---:|:---:|
|0|Bus0|
|1|Bus1|
|2|Bus2|
|3|Bus3|

- RebootRequired: True

## ARSPD3_DEVID: Airspeed ID

*Note: This parameter is for advanced users*

Airspeed sensor ID, taking into account its type, bus and instance

- ReadOnly: True

# ARSPD4 Parameters

## ARSPD4_TYPE: Airspeed type

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
|16|ExternalAHRS|
|17|AUAV-10in|
|18|AUAV-5in|
|19|AUAV-30in|
|100|SITL|

## ARSPD4_USE: Airspeed use

This parameter is not used by this vehicle. Always set to 0.

|Value|Meaning|
|:---:|:---:|
|0|DoNotUse|
|1|Use|
|2|UseWhenZeroThrottle|

## ARSPD4_OFFSET: Airspeed offset

*Note: This parameter is for advanced users*

Airspeed calibration offset

- Increment: 0.1

## ARSPD4_RATIO: Airspeed ratio

*Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.

- Increment: 0.1

## ARSPD4_PIN: Airspeed pin

*Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## ARSPD4_AUTOCAL: This parameter and function is not used by this vehicle. Always set to 0.

*Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD4_TUBE_ORDR: Control pitot tube order

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD4_SKIP_CAL: Skip airspeed offset calibration on startup

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration or requiring a manual calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Do not require offset calibration before flight. Manual calibration should be performed during initial setup.|
|2|Do not calibrate on start up. Manual calibration must be performed once per boot.|

## ARSPD4_PSI_RANGE: The PSI range of the device

*Note: This parameter is for advanced users*

This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD4_BUS: Airspeed I2C bus

*Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected. May not correspond to board's I2C bus number labels. Retry another bus and reboot if airspeed sensor fails to initialize.

|Value|Meaning|
|:---:|:---:|
|0|Bus0|
|1|Bus1|
|2|Bus2|
|3|Bus3|

- RebootRequired: True

## ARSPD4_DEVID: Airspeed ID

*Note: This parameter is for advanced users*

Airspeed sensor ID, taking into account its type, bus and instance

- ReadOnly: True

# ARSPD5 Parameters

## ARSPD5_TYPE: Airspeed type

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
|16|ExternalAHRS|
|17|AUAV-10in|
|18|AUAV-5in|
|19|AUAV-30in|
|100|SITL|

## ARSPD5_USE: Airspeed use

This parameter is not used by this vehicle. Always set to 0.

|Value|Meaning|
|:---:|:---:|
|0|DoNotUse|
|1|Use|
|2|UseWhenZeroThrottle|

## ARSPD5_OFFSET: Airspeed offset

*Note: This parameter is for advanced users*

Airspeed calibration offset

- Increment: 0.1

## ARSPD5_RATIO: Airspeed ratio

*Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.

- Increment: 0.1

## ARSPD5_PIN: Airspeed pin

*Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## ARSPD5_AUTOCAL: This parameter and function is not used by this vehicle. Always set to 0.

*Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD5_TUBE_ORDR: Control pitot tube order

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD5_SKIP_CAL: Skip airspeed offset calibration on startup

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration or requiring a manual calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Do not require offset calibration before flight. Manual calibration should be performed during initial setup.|
|2|Do not calibrate on start up. Manual calibration must be performed once per boot.|

## ARSPD5_PSI_RANGE: The PSI range of the device

*Note: This parameter is for advanced users*

This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD5_BUS: Airspeed I2C bus

*Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected. May not correspond to board's I2C bus number labels. Retry another bus and reboot if airspeed sensor fails to initialize.

|Value|Meaning|
|:---:|:---:|
|0|Bus0|
|1|Bus1|
|2|Bus2|
|3|Bus3|

- RebootRequired: True

## ARSPD5_DEVID: Airspeed ID

*Note: This parameter is for advanced users*

Airspeed sensor ID, taking into account its type, bus and instance

- ReadOnly: True

# ARSPD6 Parameters

## ARSPD6_TYPE: Airspeed type

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
|16|ExternalAHRS|
|17|AUAV-10in|
|18|AUAV-5in|
|19|AUAV-30in|
|100|SITL|

## ARSPD6_USE: Airspeed use

This parameter is not used by this vehicle. Always set to 0.

|Value|Meaning|
|:---:|:---:|
|0|DoNotUse|
|1|Use|
|2|UseWhenZeroThrottle|

## ARSPD6_OFFSET: Airspeed offset

*Note: This parameter is for advanced users*

Airspeed calibration offset

- Increment: 0.1

## ARSPD6_RATIO: Airspeed ratio

*Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.

- Increment: 0.1

## ARSPD6_PIN: Airspeed pin

*Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## ARSPD6_AUTOCAL: This parameter and function is not used by this vehicle. Always set to 0.

*Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.

## ARSPD6_TUBE_ORDR: Control pitot tube order

*Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|Swapped|
|2|Auto Detect|

## ARSPD6_SKIP_CAL: Skip airspeed offset calibration on startup

*Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration or requiring a manual calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Do not require offset calibration before flight. Manual calibration should be performed during initial setup.|
|2|Do not calibrate on start up. Manual calibration must be performed once per boot.|

## ARSPD6_PSI_RANGE: The PSI range of the device

*Note: This parameter is for advanced users*

This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device

## ARSPD6_BUS: Airspeed I2C bus

*Note: This parameter is for advanced users*

Bus number of the I2C bus where the airspeed sensor is connected. May not correspond to board's I2C bus number labels. Retry another bus and reboot if airspeed sensor fails to initialize.

|Value|Meaning|
|:---:|:---:|
|0|Bus0|
|1|Bus1|
|2|Bus2|
|3|Bus3|

- RebootRequired: True

## ARSPD6_DEVID: Airspeed ID

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
|16|ExternalAHRS|
|17|AUAV-10in|
|18|AUAV-5in|
|19|AUAV-30in|
|100|SITL|

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

The pin number that the airspeed sensor is connected to for analog sensors. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## ARSPD_AUTOCAL: This parameter and function is not used by this vehicle. Always set to 0.

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

This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration or requiring a manual calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Do not require offset calibration before flight. Manual calibration should be performed during initial setup.|
|2|Do not calibrate on start up. Manual calibration must be performed once per boot.|

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
|3|Bus3|

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

- Bitmask: 0:BMP085,1:BMP280,2:MS5611,3:MS5607,4:MS5637,5:FBM320,6:DPS280,7:LPS25H,8:Keller,9:MS5837,10:BMP388,11:SPL06,12:MSP,13:BMP581,14:AUAV

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

## BARO1_THST_SCALE: Thrust compensation

*Note: This parameter is for advanced users*

Thrust scaling in Pascals. This value scaled by the normalized thrust is subtracted from the barometer pressure. This is used to adjust linearly based on the thrust output for local pressure difference induced by the props.

- Range: -300 300

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT2_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT2_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT2_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT2_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT2_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT2_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT2_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT2_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT2_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT2_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT2_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT2_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT2_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT2_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT2_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT2_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT2_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT2_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT2_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT2_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT2_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT2_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT2_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT2_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT3_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT3_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT3_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT3_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT3_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT3_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT3_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT3_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT3_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT3_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT3_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT3_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT3_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT3_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT3_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT3_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT3_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT3_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT3_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT3_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT3_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT3_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT3_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT3_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT4_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT4_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT4_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT4_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT4_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT4_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT4_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT4_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT4_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT4_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT4_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT4_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT4_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT4_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT4_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT4_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT4_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT4_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT4_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT4_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT4_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT4_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT4_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT4_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT5_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT5_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT5_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT5_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT5_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT5_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT5_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT5_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT5_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT5_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT5_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT5_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT5_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT5_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT5_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT5_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT5_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT5_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT5_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT5_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT5_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT5_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT5_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT5_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT6_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT6_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT6_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT6_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT6_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT6_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT6_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT6_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT6_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT6_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT6_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT6_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT6_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT6_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT6_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT6_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT6_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT6_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT6_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT6_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT6_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT6_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT6_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT6_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT7_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT7_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT7_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT7_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT7_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT7_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT7_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT7_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT7_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT7_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT7_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT7_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT7_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT7_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT7_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT7_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT7_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT7_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT7_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT7_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT7_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT7_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT7_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT7_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT8_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT8_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT8_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT8_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT8_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT8_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT8_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT8_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT8_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT8_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT8_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT8_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT8_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT8_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT8_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT8_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT8_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT8_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT8_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT8_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT8_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT8_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT8_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT8_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT9_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT9_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT9_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT9_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT9_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT9_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT9_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT9_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT9_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT9_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT9_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT9_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT9_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT9_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT9_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT9_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT9_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT9_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT9_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT9_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT9_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT9_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT9_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT9_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BATTA Parameters

## BATTA_MONITOR: Battery monitoring

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

- RebootRequired: True

## BATTA_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTA_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATTA_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTA_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTA_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTA_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTA_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTA_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTA_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTA_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTA_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTA_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTA_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTA_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTA_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTA_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTA_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTA_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATTA_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTA_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATTA_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## BATTA_VOLT_PIN: Battery Voltage sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTA_CURR_PIN: Battery Current sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTA_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTA_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTA_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATTA_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATTA_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATTA_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTA_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATTA_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATTA_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: 0.1 10

## BATTA_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATTA_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATTA_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATTA_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATTA_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATTA_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATTA_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATTA_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

## BATTA_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATTA_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTA_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTA_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

## BATTA_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTA_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATTA_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATTA_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTA_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTA_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTA_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATTA_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTA_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTA_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTA_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATTA_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATTA_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTA_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BATTB Parameters

## BATTB_MONITOR: Battery monitoring

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

- RebootRequired: True

## BATTB_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTB_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATTB_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTB_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTB_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTB_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTB_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTB_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTB_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTB_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTB_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTB_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTB_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTB_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTB_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTB_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTB_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTB_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATTB_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTB_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATTB_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## BATTB_VOLT_PIN: Battery Voltage sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTB_CURR_PIN: Battery Current sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTB_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTB_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTB_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATTB_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATTB_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATTB_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTB_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATTB_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATTB_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: 0.1 10

## BATTB_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATTB_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATTB_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATTB_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATTB_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATTB_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATTB_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATTB_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

## BATTB_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATTB_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTB_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTB_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

## BATTB_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTB_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATTB_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATTB_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTB_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTB_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTB_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATTB_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTB_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTB_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTB_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATTB_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATTB_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTB_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BATTC Parameters

## BATTC_MONITOR: Battery monitoring

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

- RebootRequired: True

## BATTC_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTC_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATTC_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTC_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTC_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTC_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTC_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTC_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTC_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTC_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTC_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTC_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTC_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTC_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTC_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTC_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTC_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTC_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATTC_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTC_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATTC_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## BATTC_VOLT_PIN: Battery Voltage sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTC_CURR_PIN: Battery Current sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTC_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTC_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTC_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATTC_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATTC_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATTC_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTC_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATTC_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATTC_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: 0.1 10

## BATTC_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATTC_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATTC_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATTC_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATTC_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATTC_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATTC_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATTC_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

## BATTC_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATTC_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTC_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTC_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

## BATTC_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTC_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATTC_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATTC_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTC_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTC_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTC_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATTC_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTC_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTC_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTC_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATTC_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATTC_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTC_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BATTD Parameters

## BATTD_MONITOR: Battery monitoring

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

- RebootRequired: True

## BATTD_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTD_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATTD_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTD_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTD_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTD_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTD_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTD_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTD_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTD_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTD_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTD_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTD_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTD_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTD_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTD_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTD_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTD_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATTD_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTD_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATTD_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## BATTD_VOLT_PIN: Battery Voltage sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTD_CURR_PIN: Battery Current sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTD_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTD_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTD_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATTD_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATTD_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATTD_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTD_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATTD_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATTD_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: 0.1 10

## BATTD_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATTD_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATTD_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATTD_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATTD_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATTD_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATTD_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATTD_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

## BATTD_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATTD_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTD_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTD_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

## BATTD_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTD_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATTD_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATTD_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTD_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTD_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTD_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATTD_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTD_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTD_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTD_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATTD_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATTD_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTD_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BATTE Parameters

## BATTE_MONITOR: Battery monitoring

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

- RebootRequired: True

## BATTE_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTE_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATTE_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTE_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTE_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTE_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTE_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTE_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTE_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTE_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTE_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTE_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTE_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTE_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTE_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTE_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTE_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTE_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATTE_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTE_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATTE_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## BATTE_VOLT_PIN: Battery Voltage sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTE_CURR_PIN: Battery Current sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTE_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTE_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTE_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATTE_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATTE_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATTE_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTE_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATTE_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATTE_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: 0.1 10

## BATTE_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATTE_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATTE_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATTE_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATTE_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATTE_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATTE_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATTE_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

## BATTE_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATTE_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTE_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTE_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

## BATTE_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTE_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATTE_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATTE_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTE_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTE_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTE_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATTE_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTE_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTE_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTE_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATTE_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATTE_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTE_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BATTF Parameters

## BATTF_MONITOR: Battery monitoring

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

- RebootRequired: True

## BATTF_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTF_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATTF_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTF_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTF_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTF_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTF_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTF_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTF_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTF_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTF_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTF_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTF_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTF_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTF_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTF_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTF_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTF_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATTF_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTF_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATTF_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## BATTF_VOLT_PIN: Battery Voltage sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTF_CURR_PIN: Battery Current sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTF_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTF_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTF_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATTF_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATTF_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATTF_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTF_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATTF_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATTF_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: 0.1 10

## BATTF_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATTF_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATTF_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATTF_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATTF_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATTF_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATTF_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATTF_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

## BATTF_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATTF_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTF_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTF_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

## BATTF_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTF_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATTF_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATTF_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTF_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTF_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTF_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATTF_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTF_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTF_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTF_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATTF_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATTF_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTF_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BATTG Parameters

## BATTG_MONITOR: Battery monitoring

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

- RebootRequired: True

## BATTG_CAPACITY: Battery capacity

Capacity of the battery in mAh when full

- Units: mAh

- Increment: 50

## BATTG_SERIAL_NUM: Battery serial number

*Note: This parameter is for advanced users*

Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.

## BATTG_LOW_TIMER: Low voltage timeout

*Note: This parameter is for advanced users*

This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.

- Units: s

- Increment: 1

- Range: 0 120

## BATTG_FS_VOLTSRC: Failsafe voltage source

*Note: This parameter is for advanced users*

Voltage type used for detection of low voltage event

|Value|Meaning|
|:---:|:---:|
|0|Raw Voltage|
|1|Sag Compensated Voltage|

## BATTG_LOW_VOLT: Low battery voltage

Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTG_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTG_FS_LOW_ACT parameter.

- Units: V

- Increment: 0.1

## BATTG_LOW_MAH: Low battery capacity

Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTG_FS_LOW_ACT parameter.

- Units: mAh

- Increment: 50

## BATTG_CRT_VOLT: Critical battery voltage

Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the BATTG_LOW_TIMER parameter then the vehicle will perform the failsafe specified by the BATTG_FS_CRT_ACT parameter.

- Units: V

- Increment: 0.1

## BATTG_CRT_MAH: Battery critical capacity

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATTG_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATTG_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTG_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATTG_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATTG_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATTG_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATTG_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATTG_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## BATTG_VOLT_PIN: Battery Voltage sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTG_CURR_PIN: Battery Current sensing pin

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

- Range: -1 127

- RebootRequired: True

## BATTG_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTG_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.

## BATTG_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.

- Units: A/V

## BATTG_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.

- Units: V

## BATTG_VLT_OFFSET: Voltage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.

- Units: V

## BATTG_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTG_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address

- Range: 0 127

- RebootRequired: True

## BATTG_SUM_MASK: Battery Sum mask

0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.

- Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9

## BATTG_CURR_MULT: Scales reported power monitor current

*Note: This parameter is for advanced users*

Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications

- Range: 0.1 10

## BATTG_FL_VLT_MIN: Empty fuel level voltage

*Note: This parameter is for advanced users*

The voltage seen on the analog pin when the fuel tank is empty. Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

- Units: V

## BATTG_FL_V_MULT: Fuel level voltage multiplier

*Note: This parameter is for advanced users*

Voltage multiplier to determine what the full tank voltage reading is. This is calculated as 1 / (Voltage_Full - Voltage_Empty) Note: For this type of battery monitor, the voltage seen by the analog pin is displayed as battery voltage on a GCS.

- Range: 0.01 10

## BATTG_FL_FLTR: Fuel level filter frequency

*Note: This parameter is for advanced users*

Filter frequency in Hertz where a low pass filter is used. This is used to filter out tank slosh from the fuel level reading. A value of -1 disables the filter and unfiltered voltage is used to determine the fuel level. The suggested values at in the range of 0.2 Hz to 0.5 Hz.

- Range: -1 1

- Units: Hz

- RebootRequired: True

## BATTG_FL_PIN: Fuel level analog pin number

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATTG_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATTG_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATTG_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATTG_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

## BATTG_MAX_VOLT: Maximum Battery Voltage

*Note: This parameter is for advanced users*

Maximum voltage of battery. Provides scaling of current versus voltage

- Range: 7 100

## BATTG_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTG_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTG_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INS2XX sensor will work with.

- Range: 1 400

- Units: A

## BATTG_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTG_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATTG_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATTG_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATTG_I2C_BUS: Battery monitor I2C bus number

*Note: This parameter is for advanced users*

Battery monitor I2C bus number

- Range: 0 3

- RebootRequired: True

## BATTG_I2C_ADDR: Battery monitor I2C address

*Note: This parameter is for advanced users*

Battery monitor I2C address. If this is zero then probe list of supported addresses

- Range: 0 127

- RebootRequired: True

## BATTG_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATTG_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTG_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATTG_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATTG_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATTG_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATTG_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATTG_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

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
|21|INA2XX (INA226 INA228 INA238 INA231 INA260)|
|22|LTC2946|
|23|Torqeedo|
|24|FuelLevelAnalog|
|25|Synthetic Current and Analog Voltage|
|26|INA239_SPI|
|27|EFI|
|28|AD7091R5|
|29|Scripting|
|30|INA3221|
|31|Analog Current Only|
|32|TIBQ76952-I2C (Periph only)|

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

Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the BATT_FS_CRT_ACT parameter.

- Units: mAh

- Increment: 50

## BATT_FS_LOW_ACT: Low battery failsafe action

What action the vehicle should perform if it hits a low battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Land|

## BATT_ARM_VOLT: Required arming voltage

*Note: This parameter is for advanced users*

Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.

- Units: V

- Increment: 0.1

## BATT_ARM_MAH: Required arming remaining capacity

*Note: This parameter is for advanced users*

Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the BATT_ARM_VOLT parameter.

- Units: mAh

- Increment: 50

## BATT_OPTIONS: Battery monitor options

*Note: This parameter is for advanced users*

This sets options to change the behaviour of the battery monitor

- Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS, 7:Allow DroneCAN InfoAux to be from a different CAN node, 8:Battery is for internal autopilot use only, 9:Sum monitor measures minimum voltage instead of average, 10:Allow DroneCAN dynamic node update on hot-swap

## BATT_ESC_INDEX: ESC Telemetry Index to write to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write voltage, current, consumption and temperature data to. Use 0 to disable.

- Range: 0 10

- Increment: 1

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

- Range: -1 127

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

- Range: -1 127

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

- Range: 0.1 10

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

Analog input pin that fuel level sensor is connected to.Analog Airspeed or RSSI ports can be used for Analog input( some autopilots provide others also). Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## BATT_FL_FF: First order term

*Note: This parameter is for advanced users*

First order polynomial fit term

- Range: -10 10

## BATT_FL_FS: Second order term

*Note: This parameter is for advanced users*

Second order polynomial fit term

- Range: -10 10

## BATT_FL_FT: Third order term

*Note: This parameter is for advanced users*

Third order polynomial fit term

- Range: -10 10

## BATT_FL_OFF: Offset term

*Note: This parameter is for advanced users*

Offset polynomial fit term

- Range: -10 10

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

## BATT_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

## BATT_ESC_MASK: ESC mask

If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## BATT_MAX_AMPS: Battery monitor max current

*Note: This parameter is for advanced users*

This controls the maximum current the INA239 sensor will work with.

- Range: 1 400

- Units: A

## BATT_SHUNT: Battery monitor shunt resistor

*Note: This parameter is for advanced users*

This sets the shunt resistor used in the device

- Range: 0.0001 0.01

- Units: Ohm

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

## BATT_CHANNEL: INA3221 channel

*Note: This parameter is for advanced users*

INA3221 channel to return data for

- Range: 1 3

- RebootRequired: True

## BATT_VOLT_PIN: Battery Voltage sensing pin on the AD7091R5 Ic

Sets the analog input pin that should be used for voltage monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT_CURR_PIN: Battery Current sensing pin

Sets the analog input pin that should be used for Current monitoring on AD7091R5.

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|

- Range: -1 127

- RebootRequired: True

## BATT_VOLT_MULT: Voltage Multiplier

*Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).

## BATT_AMP_PERVLT: Amps per volt

Number of amps that a 1V reading on the current sensor corresponds to.

- Units: A/V

## BATT_AMP_OFFSET: AMP offset

Voltage offset at zero current on current sensor

- Units: V

## BATT_VLT_OFFSET: Volage offset

*Note: This parameter is for advanced users*

Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied

- Units: V

# BRD Parameters

## BRD_SER1_RTSCTS: Serial 1 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 1 (telemetry 1). You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|
|3|RS-485 Driver enable RTS pin|

- RebootRequired: True

## BRD_SER2_RTSCTS: Serial 2 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 2 (telemetry 2). You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|
|3|RS-485 Driver enable RTS pin|

## BRD_SER3_RTSCTS: Serial 3 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 3. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|
|3|RS-485 Driver enable RTS pin|

## BRD_SER4_RTSCTS: Serial 4 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 4. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|
|3|RS-485 Driver enable RTS pin|

## BRD_SER5_RTSCTS: Serial 5 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 5. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|
|3|RS-485 Driver enable RTS pin|

## BRD_SER6_RTSCTS: Serial 6 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 6. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|
|3|RS-485 Driver enable RTS pin|

## BRD_SER7_RTSCTS: Serial 7 flow control

*Note: This parameter is for advanced users*

Enable flow control on serial 7. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

- RebootRequired: True

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|Auto|
|3|RS-485 Driver enable RTS pin|

## BRD_SER8_RTSCTS: Serial 8 flow control

Enable flow control on serial 8. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.

## BRD_SAFETY_DEFLT: Sets default state of the safety switch

This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## BRD_SBUS_OUT: SBUS output rate

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

- Bitmask: 0:Output1, 1:Output2, 2:Output3, 3:Output4, 4:Output5, 5:Output6, 6:Output7, 7:Output8, 8:Output9, 9:Output10, 10:Output11, 11:Output12, 12:Output13, 13:Output14, 14:Output15, 15:Output16, 16:Output17, 17:Output18, 18:Output19, 19:Output20, 20:Output21, 21:Output22, 22:Output23, 23:Output24, 24:Output25, 25:Output26, 26:Output27, 27:Output28, 28:Output29, 29:Output30, 30:Output31, 31:Output32

- RebootRequired: True

## BRD_HEAT_TARG: Board heater temperature target

*Note: This parameter is for advanced users*

Board heater target temperature for boards with controllable heating units. Set to -1 to disable the heater, please reboot after setting to -1.

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
|5|PixhawkMini|
|6|Pixhawk2Slim|
|13|Intel Aero FC|
|14|Pixhawk Pro|
|20|AUAV2.1|
|22|MINDPXV2|
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

This sets the voltage max for PWM output pulses. 0 for 3.3V and 1 for 5V output. On boards with an IOMCU that support this parameter this option only affects the 8 main outputs, not the 6 auxiliary outputs. Using 5V output can help to reduce the impact of ESC noise interference corrupting signals to the ESCs.

|Value|Meaning|
|:---:|:---:|
|0|3.3V|
|1|5V|

## BRD_OPTIONS: Board options

*Note: This parameter is for advanced users*

Board specific option flags

- Bitmask: 0:Enable hardware watchdog, 1:Disable MAVftp, 2:Enable set of internal parameters, 3:Enable Debug Pins, 4:Unlock flash on reboot, 5:Write protect firmware flash on reboot, 6:Write protect bootloader flash on reboot, 7:Skip board validation, 8:Disable board arming gpio output change on arm/disarm, 9:Use safety pins as profiled

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

## BRD_SD_MISSION: SDCard Mission size

*Note: This parameter is for advanced users*

This sets the amount of storage in kilobytes reserved on the microsd card in mission.stg for waypoint storage. Each waypoint uses 15 bytes.

- Range: 0 64

- RebootRequired: True

## BRD_SD_FENCE: SDCard Fence size

*Note: This parameter is for advanced users*

This sets the amount of storage in kilobytes reserved on the microsd card in fence.stg for fence storage.

- Range: 0 64

- RebootRequired: True

## BRD_IO_DSHOT: Load DShot FW on IO

*Note: This parameter is for advanced users*

This loads the DShot firmware on the IO co-processor

|Value|Meaning|
|:---:|:---:|
|0|StandardFW|
|1|DshotFW|

- RebootRequired: True

## BRD_IDLE_STATS: Capture and calculate true CPU load using idle threads

*Note: This parameter is for advanced users*

Capture and calculate true CPU load using idle threads

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

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
|3|Mode3|
|4|Mode4|

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
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|
|10|Scripting|
|11|Benewake|
|12|Scripting2|
|13|TOFSenseP|
|14|RadarCAN (NanoRadar/Hexsoon)|

- RebootRequired: True

## CAN_D1_PROTOCOL2: Secondary protocol with 11 bit CAN addressing

*Note: This parameter is for advanced users*

Secondary protocol with 11 bit CAN addressing

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|7|USD1|
|10|Scripting|
|11|Benewake|
|12|Scripting2|
|13|TOFSenseP|
|14|RadarCAN (NanoRadar/Hexsoon)|

- RebootRequired: True

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

# CAND1UC Parameters

## CAN_D1_UC_NODE: Own node ID

*Note: This parameter is for advanced users*

DroneCAN node ID used by the driver itself on this network

- Range: 1 125

## CAN_D1_UC_SRV_BM: Output channels to be transmitted as servo over DroneCAN

Bitmask with one set for channel to be transmitted as a servo command over DroneCAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

## CAN_D1_UC_ESC_BM: Output channels to be transmitted as ESC over DroneCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a ESC command over DroneCAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D1_UC_SRV_RT: Servo output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for servo outputs

- Range: 1 200

- Units: Hz

## CAN_D1_UC_OPTION: DroneCAN options

*Note: This parameter is for advanced users*

Option flags

- Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy,4:SendServoAsPWM,5:SendGNSS,6:UseHimarkServo,7:HobbyWingESC,8:EnableStats,9:EnableFlexDebug,10:SecondaryAllowExtendedFrames

## CAN_D1_UC_NTF_RT: Notify State rate

*Note: This parameter is for advanced users*

Maximum transmit rate for Notify State Message

- Range: 1 200

- Units: Hz

## CAN_D1_UC_ESC_OF: ESC Output channels offset

*Note: This parameter is for advanced users*

Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo outputs 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficient usage of CAN bandwidth

- Range: 0 18

## CAN_D1_UC_POOL: CAN pool size

*Note: This parameter is for advanced users*

Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads

- Range: 1024 16384

## CAN_D1_UC_ESC_RV: Bitmask for output channels for reversible ESCs over DroneCAN.

*Note: This parameter is for advanced users*

Bitmask with one set for each output channel that uses a reversible ESC over DroneCAN. Reversible ESCs use both positive and negative values in RawCommands, with positive commanding the forward direction and negative commanding the reverse direction.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D1_UC_RLY_RT: DroneCAN relay output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for relay outputs, note that this rate is per message each message does 1 relay, so if with more relays will take longer to update at the same rate, a extra message will be sent when a relay changes state

- Range: 0 200

- Units: Hz

## CAN_D1_UC_SER_EN: DroneCAN Serial enable

*Note: This parameter is for advanced users*

Enable DroneCAN virtual serial ports

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## CAN_D1_UC_S1_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN remote node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D1_UC_S1_IDX: DroneCAN Serial1 index

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

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

- RebootRequired: True

## CAN_D1_UC_S1_BD: DroneCAN Serial default baud rate

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- RebootRequired: True

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D1_UC_S1_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## CAN_D1_UC_S2_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN remote node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D1_UC_S2_IDX: Serial port number on remote CAN node

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

- RebootRequired: True

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

## CAN_D1_UC_S2_BD: DroneCAN Serial default baud rate

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- Range: 1 20000000

- RebootRequired: True

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D1_UC_S2_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## CAN_D1_UC_S3_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D1_UC_S3_IDX: Serial port number on remote CAN node

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

- RebootRequired: True

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

## CAN_D1_UC_S3_BD: Serial baud rate on remote CAN node

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- Range: 1 20000000

- RebootRequired: True

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D1_UC_S3_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

# CAND2 Parameters

## CAN_D2_PROTOCOL: Enable use of specific protocol over virtual driver

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|DroneCAN|
|4|PiccoloCAN|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|
|10|Scripting|
|11|Benewake|
|12|Scripting2|
|13|TOFSenseP|
|14|RadarCAN (NanoRadar/Hexsoon)|

- RebootRequired: True

## CAN_D2_PROTOCOL2: Secondary protocol with 11 bit CAN addressing

*Note: This parameter is for advanced users*

Secondary protocol with 11 bit CAN addressing

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|7|USD1|
|10|Scripting|
|11|Benewake|
|12|Scripting2|
|13|TOFSenseP|
|14|RadarCAN (NanoRadar/Hexsoon)|

- RebootRequired: True

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

# CAND2UC Parameters

## CAN_D2_UC_NODE: Own node ID

*Note: This parameter is for advanced users*

DroneCAN node ID used by the driver itself on this network

- Range: 1 125

## CAN_D2_UC_SRV_BM: Output channels to be transmitted as servo over DroneCAN

Bitmask with one set for channel to be transmitted as a servo command over DroneCAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

## CAN_D2_UC_ESC_BM: Output channels to be transmitted as ESC over DroneCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a ESC command over DroneCAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D2_UC_SRV_RT: Servo output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for servo outputs

- Range: 1 200

- Units: Hz

## CAN_D2_UC_OPTION: DroneCAN options

*Note: This parameter is for advanced users*

Option flags

- Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy,4:SendServoAsPWM,5:SendGNSS,6:UseHimarkServo,7:HobbyWingESC,8:EnableStats,9:EnableFlexDebug,10:SecondaryAllowExtendedFrames

## CAN_D2_UC_NTF_RT: Notify State rate

*Note: This parameter is for advanced users*

Maximum transmit rate for Notify State Message

- Range: 1 200

- Units: Hz

## CAN_D2_UC_ESC_OF: ESC Output channels offset

*Note: This parameter is for advanced users*

Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo outputs 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficient usage of CAN bandwidth

- Range: 0 18

## CAN_D2_UC_POOL: CAN pool size

*Note: This parameter is for advanced users*

Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads

- Range: 1024 16384

## CAN_D2_UC_ESC_RV: Bitmask for output channels for reversible ESCs over DroneCAN.

*Note: This parameter is for advanced users*

Bitmask with one set for each output channel that uses a reversible ESC over DroneCAN. Reversible ESCs use both positive and negative values in RawCommands, with positive commanding the forward direction and negative commanding the reverse direction.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D2_UC_RLY_RT: DroneCAN relay output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for relay outputs, note that this rate is per message each message does 1 relay, so if with more relays will take longer to update at the same rate, a extra message will be sent when a relay changes state

- Range: 0 200

- Units: Hz

## CAN_D2_UC_SER_EN: DroneCAN Serial enable

*Note: This parameter is for advanced users*

Enable DroneCAN virtual serial ports

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## CAN_D2_UC_S1_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN remote node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D2_UC_S1_IDX: DroneCAN Serial1 index

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

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

- RebootRequired: True

## CAN_D2_UC_S1_BD: DroneCAN Serial default baud rate

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- RebootRequired: True

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D2_UC_S1_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## CAN_D2_UC_S2_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN remote node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D2_UC_S2_IDX: Serial port number on remote CAN node

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

- RebootRequired: True

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

## CAN_D2_UC_S2_BD: DroneCAN Serial default baud rate

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- Range: 1 20000000

- RebootRequired: True

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D2_UC_S2_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## CAN_D2_UC_S3_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D2_UC_S3_IDX: Serial port number on remote CAN node

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

- RebootRequired: True

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

## CAN_D2_UC_S3_BD: Serial baud rate on remote CAN node

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- Range: 1 20000000

- RebootRequired: True

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D2_UC_S3_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

# CAND3 Parameters

## CAN_D3_PROTOCOL: Enable use of specific protocol over virtual driver

*Note: This parameter is for advanced users*

Enabling this option starts selected protocol that will use this virtual driver

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|DroneCAN|
|4|PiccoloCAN|
|6|EFI_NWPMU|
|7|USD1|
|8|KDECAN|
|10|Scripting|
|11|Benewake|
|12|Scripting2|
|13|TOFSenseP|
|14|RadarCAN (NanoRadar/Hexsoon)|

- RebootRequired: True

## CAN_D3_PROTOCOL2: Secondary protocol with 11 bit CAN addressing

*Note: This parameter is for advanced users*

Secondary protocol with 11 bit CAN addressing

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|7|USD1|
|10|Scripting|
|11|Benewake|
|12|Scripting2|
|13|TOFSenseP|
|14|RadarCAN (NanoRadar/Hexsoon)|

- RebootRequired: True

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

# CAND3UC Parameters

## CAN_D3_UC_NODE: Own node ID

*Note: This parameter is for advanced users*

DroneCAN node ID used by the driver itself on this network

- Range: 1 125

## CAN_D3_UC_SRV_BM: Output channels to be transmitted as servo over DroneCAN

Bitmask with one set for channel to be transmitted as a servo command over DroneCAN

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

## CAN_D3_UC_ESC_BM: Output channels to be transmitted as ESC over DroneCAN

*Note: This parameter is for advanced users*

Bitmask with one set for channel to be transmitted as a ESC command over DroneCAN

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D3_UC_SRV_RT: Servo output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for servo outputs

- Range: 1 200

- Units: Hz

## CAN_D3_UC_OPTION: DroneCAN options

*Note: This parameter is for advanced users*

Option flags

- Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy,4:SendServoAsPWM,5:SendGNSS,6:UseHimarkServo,7:HobbyWingESC,8:EnableStats,9:EnableFlexDebug,10:SecondaryAllowExtendedFrames

## CAN_D3_UC_NTF_RT: Notify State rate

*Note: This parameter is for advanced users*

Maximum transmit rate for Notify State Message

- Range: 1 200

- Units: Hz

## CAN_D3_UC_ESC_OF: ESC Output channels offset

*Note: This parameter is for advanced users*

Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo outputs 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficient usage of CAN bandwidth

- Range: 0 18

## CAN_D3_UC_POOL: CAN pool size

*Note: This parameter is for advanced users*

Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads

- Range: 1024 16384

## CAN_D3_UC_ESC_RV: Bitmask for output channels for reversible ESCs over DroneCAN.

*Note: This parameter is for advanced users*

Bitmask with one set for each output channel that uses a reversible ESC over DroneCAN. Reversible ESCs use both positive and negative values in RawCommands, with positive commanding the forward direction and negative commanding the reverse direction.

- Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32

## CAN_D3_UC_RLY_RT: DroneCAN relay output rate

*Note: This parameter is for advanced users*

Maximum transmit rate for relay outputs, note that this rate is per message each message does 1 relay, so if with more relays will take longer to update at the same rate, a extra message will be sent when a relay changes state

- Range: 0 200

- Units: Hz

## CAN_D3_UC_SER_EN: DroneCAN Serial enable

*Note: This parameter is for advanced users*

Enable DroneCAN virtual serial ports

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## CAN_D3_UC_S1_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN remote node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D3_UC_S1_IDX: DroneCAN Serial1 index

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

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

- RebootRequired: True

## CAN_D3_UC_S1_BD: DroneCAN Serial default baud rate

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- RebootRequired: True

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D3_UC_S1_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## CAN_D3_UC_S2_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN remote node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D3_UC_S2_IDX: Serial port number on remote CAN node

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

- RebootRequired: True

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

## CAN_D3_UC_S2_BD: DroneCAN Serial default baud rate

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- Range: 1 20000000

- RebootRequired: True

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D3_UC_S2_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## CAN_D3_UC_S3_NOD: Serial CAN remote node number

*Note: This parameter is for advanced users*

CAN node number for serial port

- Range: 0 127

- RebootRequired: True

## CAN_D3_UC_S3_IDX: Serial port number on remote CAN node

*Note: This parameter is for advanced users*

Serial port number on remote CAN node

- Range: -1 100

- RebootRequired: True

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

## CAN_D3_UC_S3_BD: Serial baud rate on remote CAN node

*Note: This parameter is for advanced users*

Serial baud rate on remote CAN node

- Range: 1 20000000

- RebootRequired: True

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## CAN_D3_UC_S3_PRO: Serial protocol of DroneCAN serial port

*Note: This parameter is for advanced users*

Serial protocol of DroneCAN serial port

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

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

## CAN_P1_OPTIONS: CAN per-interface options

*Note: This parameter is for advanced users*

CAN per-interface options

- Bitmask: 0:LogAllFrames

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

## CAN_P2_OPTIONS: CAN per-interface options

*Note: This parameter is for advanced users*

CAN per-interface options

- Bitmask: 0:LogAllFrames

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

## CAN_P3_OPTIONS: CAN per-interface options

*Note: This parameter is for advanced users*

CAN per-interface options

- Bitmask: 0:LogAllFrames

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

- Calibration: 1

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

## COMPASS_DISBLMSK: Compass disable driver type mask

*Note: This parameter is for advanced users*

This is a bitmask of driver types to disable. If a driver type is set in this mask then that driver will not try to find a sensor at startup

- Bitmask: 0:HMC5883,1:LSM303D,2:AK8963,3:BMM150,4:LSM9DS1,5:LIS3MDL,6:AK0991x,7:IST8310,8:ICM20948,9:MMC3416,11:DroneCAN,12:QMC5883,14:MAG3110,15:IST8308,16:RM3100,17:MSP,18:ExternalAHRS,19:MMC5XX3,20:QMC5883P,21:BMM350,22:IIS2MDC,23:LIS2MDL

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

## DDS_DOMAIN_ID: DDS DOMAIN ID

Set the ROS_DOMAIN_ID

- Range: 0 232

- RebootRequired: True

## DDS_TIMEOUT_MS: DDS ping timeout

The time in milliseconds the DDS client will wait for a response from the XRCE agent before reattempting.

- Units: ms

- Range: 1 10000

- RebootRequired: True

- Increment: 1

## DDS_MAX_RETRY: DDS ping max attempts

The maximum number of times the DDS client will attempt to ping the XRCE agent before exiting. Set to 0 to allow unlimited retries.

- Range: 0 100

- RebootRequired: True

- Increment: 1

# DDSIP Parameters

## DDS_IP0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## DDS_IP1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## DDS_IP2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## DDS_IP3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

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

- Bitmask: 0:EnforcePreArmChecks, 1:AllowNonGPSPosition, 2:LockUASIDOnFirstBasicIDRx

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
|2|MicroStrain5|
|5|InertialLabs|
|7|MicroStrain7|
|8|SBG|

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

## EAHRS_LOG_RATE: AHRS logging rate

Logging rate for EARHS devices

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
|4|Loweheiser|
|5|DroneCAN|
|6|Currawong-ECU|
|7|Scripting|
|8|Hirth|
|9|MAVLink|

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

# EFITHRLIN Parameters

## EFI_THRLIN_EN: Enable throttle linearisation

*Note: This parameter is for advanced users*

Enable EFI throttle linearisation

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## EFI_THRLIN_COEF1: Throttle linearisation - First Order

*Note: This parameter is for advanced users*

First Order Polynomial Coefficient. (=1, if throttle is first order polynomial trendline)

- Range: -1 1

- RebootRequired: True

## EFI_THRLIN_COEF2: Throttle linearisation - Second Order

*Note: This parameter is for advanced users*

Second Order Polynomial Coefficient (=0, if throttle is second order polynomial trendline)

- Range: -1 1

- RebootRequired: True

## EFI_THRLIN_COEF3: Throttle linearisation - Third Order

*Note: This parameter is for advanced users*

Third Order Polynomial Coefficient. (=0, if throttle is third order polynomial trendline)

- Range: -1 1

- RebootRequired: True

## EFI_THRLIN_OFS: throttle linearization offset

*Note: This parameter is for advanced users*

Offset for throttle linearization

- Range: 0 100

- RebootRequired: True

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

Range finder can be used as the primary height source when below this percentage of its maximum range (see RNGFND*_MAX). This will not work unless Baro or GPS height is selected as the primary height source vis EK2_ALT_SOURCE = 0 or 2 respectively.  This feature should not be used for terrain following as it is designed  for vertical takeoff and landing with climb above  the range finder use height before commencing the mission, and with horizontal position changes below that height being limited to a flat region around the takeoff and landing point.

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

## EK2_OPTIONS: Optional EKF behaviour

*Note: This parameter is for advanced users*

optional EKF2 behaviour. Disabling external navigation prevents use of external vision data in the EKF2 solution

- Bitmask: 0:DisableExternalNavigation

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

This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted. If EK3_GLITCH_RAD set to 0 the velocity innovations will be clipped instead of rejected if they exceed the gate size and a smaller value of EK3_VEL_I_GATE not exceeding 300 is recommended to limit the effect of GPS transient errors.

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

This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted. If EK3_GLITCH_RAD has been set to 0 the horizontal position innovations will be clipped instead of rejected if they exceed the gate size so a smaller value of EK3_POS_I_GATE not exceeding 300 is recommended to limit the effect of GPS transient errors.

- Range: 100 1000

- Increment: 25

## EK3_GLITCH_RAD: GPS glitch radius gate size (m)

*Note: This parameter is for advanced users*

This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position. If EK3_GLITCH_RAD set to 0 the GPS innovations will be clipped instead of rejected if they exceed the gate size set by EK3_VEL_I_GATE and EK3_POS_I_GATE which can be useful if poor quality sensor data is causing GPS rejection and loss of navigation but does make the EKF more susceptible to GPS glitches. If setting EK3_GLITCH_RAD to 0 it is recommended to reduce EK3_VEL_I_GATE and EK3_POS_I_GATE to 300.

- Range: 10 100

- Increment: 5

- Units: m

## EK3_ALT_M_NSE: Altitude measurement noise (m)

*Note: This parameter is for advanced users*

This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors. A larger value for EK3_ALT_M_NSE may be required when operating with EK3_SRCx_POSZ = 0. This parameter also sets the noise for the 'synthetic' zero height measurement that is used when EK3_SRCx_POSZ = 0.

- Range: 0.1 100.0

- Increment: 0.1

- Units: m

## EK3_HGT_I_GATE: Height measurement gate size

*Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.  If EK3_GLITCH_RAD set to 0 the vertical position innovations will be clipped instead of rejected if they exceed the gate size and a smaller value of EK3_HGT_I_GATE not exceeding 300 is recommended to limit the effect of height sensor transient errors.

- Range: 100 1000

- Increment: 25

## EK3_HGT_DELAY: Height measurement delay (msec)

*Note: This parameter is for advanced users*

This is the number of msec that the Height measurements lag behind the inertial measurements.

- Range: 0 250

- Increment: 10

- Units: ms

- RebootRequired: True

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

## EK3_FLOW_MAX: Optical flow rate maximum

*Note: This parameter is for advanced users*

The maximum optical flow rate in rad/sec that will be accepted by the filter.  Flow rates above this value will not be fused.

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

- Units: ms

- RebootRequired: True

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

- Range: 0.00001 0.02

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

Range finder can be used as the primary height source when below this percentage of its maximum range (see RNGFNDx_MAX) and the primary height source is Baro or GPS (see EK3_SRCx_POSZ).  This feature should not be used for terrain following as it is designed for vertical takeoff and landing with climb above the range finder use height before commencing the mission, and with horizontal position changes below that height being limited to a flat region around the takeoff and landing point.

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

- Units: ms

- RebootRequired: True

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

1 byte bitmap of which EKF3 instances run an independent EKF-GSF yaw estimator to provide a backup yaw estimate that doesn't rely on magnetometer data. This estimator uses IMU, GPS and, if available, airspeed data. EKF-GSF yaw estimator data for the primary EKF3 instance will be logged as GSF0 and GSF1 messages. Use of the yaw estimate generated by this algorithm is controlled by the EK3_GSF_USE_MASK and EK3_GSF_RST_MAX parameters. To run the EKF-GSF yaw estimator in ride-along and logging only, set EK3_GSF_USE to 0.

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

Ratio of mass to drag coefficient measured along the X body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared. Set to a positive value > 1.0 to enable. A starting value is the mass in Kg divided by the frontal area. The predicted drag from the rotors is specified separately by the EK3_DRAG_MCOEF parameter.

- Range: 0.0 1000.0

- Units: kg/m/m

## EK3_DRAG_BCOEF_Y: Ballistic coefficient for Y axis drag

*Note: This parameter is for advanced users*

Ratio of mass to drag coefficient measured along the Y body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared. Set to a positive value > 1.0 to enable. A starting value is the mass in Kg divided by the side area. The predicted drag from the rotors is specified separately by the EK3_DRAG_MCOEF parameter.

- Range: 50.0 1000.0

- Units: kg/m/m

## EK3_DRAG_M_NSE: Observation noise for drag acceleration

*Note: This parameter is for advanced users*

This sets the amount of noise used when fusing X and Y acceleration as an observation that enables estimation of wind velocity for multi-rotor vehicles. This feature is enabled by the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters

- Range: 0.1 2.0

- Increment: 0.1

- Units: m/s/s

## EK3_DRAG_MCOEF: Momentum coefficient for propeller drag

*Note: This parameter is for advanced users*

This parameter is used to predict the drag produced by the rotors when flying a multi-copter, enabling estimation of wind drift. The drag produced by this effect scales with speed not speed squared and is produced because some of the air velocity normal to the rotors axis of rotation is lost when passing through the rotor disc which changes the momentum of the airflow causing drag. For unducted rotors the effect is roughly proportional to the area of the propeller blades when viewed side on and changes with different propellers. It is higher for ducted rotors. For example if flying at 15 m/s at sea level conditions produces a rotor induced drag acceleration of 1.5 m/s/s, then EK3_DRAG_MCOEF would be set to 0.1 = (1.5/15.0). Set EK3_MCOEF to a positive value to enable wind estimation using this drag effect. To account for the drag produced by the body which scales with speed squared, see documentation for the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters.

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

## EK3_OPTIONS: Optional EKF behaviour

*Note: This parameter is for advanced users*

EKF optional behaviour. Bit 0 (JammingExpected): Setting JammingExpected will change the EKF behaviour such that if dead reckoning navigation is possible it will require the preflight alignment GPS quality checks controlled by EK3_GPS_CHECK and EK3_CHECK_SCALE to pass before resuming GPS use if GPS lock is lost for more than 2 seconds to prevent bad position estimate. Bit 1 (Manual lane switching): DANGEROUS – If enabled, this disables automatic lane switching. If the active lane becomes unhealthy, no automatic switching will occur. Users must manually set EK3_PRIMARY to change lanes. No health checks will be performed on the selected lane. Use with extreme caution.  Bit 2 (Optflow may use terrain alt): Terrain SRTM data will be used if the vehicle climbs above the rangefinder's range allowing optical flow to be used at higher altitudes.

- Bitmask: 0:JammingExpected, 1:ManualLaneSwitching, 2:Optflow may use terrain alt

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

EKF Source Options. Bit 0: Fuse all velocity sources present in EK3_SRCx_VEL_. Bit 1: Align external navigation position when using optical flow. Bit 3: Use SRC per core. By default, EKF source selection is controlled via the EK3_SRC parameters, allowing only one source to be active at a time across all cores (switchable via MAVLink, Lua, or RC). Enabling this bit maps EKF core 1 to SRC1, core 2 to SRC2, etc., allowing each core to run independently with a dedicated source.

- Bitmask: 0:FuseAllVelocities, 1:AlignExtNavPosWhenUsingOptFlow, 3: UsePerCoreEKFSources

# ESCTLM Parameters

## ESC_TLM_MAV_OFS: ESC Telemetry mavlink offset

Offset to apply to ESC numbers when reporting as ESC_TELEMETRY packets over MAVLink. This allows high numbered motors to be displayed as low numbered ESCs for convenience on GCS displays. A value of 4 would send ESC on output 5 as ESC number 1 in ESC_TELEMETRY packets

- Increment: 1

- Range: 0 31

# FENCE Parameters

## FENCE_ENABLE: Fence enable/disable

Allows you to enable (1) or disable (0) the fence functionality. Fences can still be enabled and disabled via mavlink or an RC option, but these changes are not persisted.

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

## FENCE_OPTIONS: Fence options

When bit 0 is set disable mode change following fence action until fence breach is cleared. When bit 1 is set the allowable flight areas is the union of all polygon and circle fence areas instead of the intersection, which means a fence breach occurs only if you are outside all of the fence areas.

- Bitmask: 0:Disable mode change following fence action until fence breach is cleared, 1:Allow union of inclusion areas, 2:Notify on margin breaches

## FENCE_NTF_FREQ: Fence margin notification frequency in hz

*Note: This parameter is for advanced users*

When bit 2 of FENCE_OPTIONS is set this parameter controls the frequency of margin breach notifications. If set to 0 only new margin breaches are notified.

- Range: 0 10

- Units: Hz

## FENCE_MARGIN_XY: Fence Horizontal Margin

Distance that autopilot's should maintain from the fence in the horizontal plane to avoid a breach. If set to 0 then FENCE_MARGIN is used.

- Units: m

- Range: 0 50

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

# FILT1 Parameters

## FILT1_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT1_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT1_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT1_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FILT2 Parameters

## FILT2_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT2_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT2_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT2_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FILT3 Parameters

## FILT3_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT3_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT3_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT3_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FILT4 Parameters

## FILT4_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT4_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT4_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT4_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FILT5 Parameters

## FILT5_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT5_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT5_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT5_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FILT6 Parameters

## FILT6_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT6_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT6_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT6_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FILT7 Parameters

## FILT7_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT7_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT7_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT7_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FILT8 Parameters

## FILT8_TYPE: Filter Type

Filter Type

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Notch Filter|

- RebootRequired: True

## FILT8_NOTCH_FREQ: Notch Filter center frequency

*Note: This parameter is for advanced users*

Notch Filter center frequency in Hz.

- Range: 10 495

- Units: Hz

## FILT8_NOTCH_Q: Notch Filter quality factor

*Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.

- Range: 1 10

## FILT8_NOTCH_ATT: Notch Filter attenuation

*Note: This parameter is for advanced users*

Notch Filter attenuation in dB.

- Range: 5 50

- Units: dB

# FINS Parameters

## FINS_FREQ_HZ: Fins frequency

This is the oscillation frequency of the fins

- Range: 1 10

## FINS_TURBO_MODE: Enable turbo mode

Enables double speed on high offset.

- Range: 0 1

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
|4|Loweheiser|
|5|CORTEX|

- RebootRequired: True

## GEN_OPTIONS: Generator Options

Bitmask of options for generators

- Bitmask: 0:Suppress Maintenance-Required Warnings

# GENL Parameters

## GEN_L_MNT_TIME: Seconds until maintenance required

*Note: This parameter is for advanced users*

Seconds until maintenance required

## GEN_L_RUNTIME: Total runtime

*Note: This parameter is for advanced users*

Total time this generator has run in seconds

## GEN_L_IDLE_TH_H: High Idle throttle

*Note: This parameter is for advanced users*

throttle value to use when warming up or cooling down

## GEN_L_IDLE_TH: Idle throttle

*Note: This parameter is for advanced users*

throttle value to use when idling

## GEN_L_RUN_TEMP: Run Temperature

*Note: This parameter is for advanced users*

temperature required for generator to start producing power in deg celsius

## GEN_L_IDLE_TEMP: Idle Temperature

*Note: This parameter is for advanced users*

temperature required for generator to return to idle after having run

## GEN_L_OVER_TEMP: Cylinder Head Over Temperature Warning Level

*Note: This parameter is for advanced users*

threshold temperature for the cylinder head above which the mavlink over temperature message gets sent

- Units: degC

# GPS Parameters

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

## GPS_SAVE_CFG: Save GPS configuration

*Note: This parameter is for advanced users*

Determines whether the configuration for this GPS should be written to non-volatile memory on the GPS. Currently working for UBlox 6 series and above.

|Value|Meaning|
|:---:|:---:|
|0|Do not save config|
|1|Save config|
|2|Save only when needed|

## GPS_AUTO_CONFIG: Automatic GPS configuration

*Note: This parameter is for advanced users*

Controls if the autopilot should automatically configure the GPS based on the parameters and default settings

|Value|Meaning|
|:---:|:---:|
|0|Disables automatic configuration|
|1|Enable automatic configuration for Serial GPSes only|
|2|Enable automatic configuration for DroneCAN as well|

## GPS_BLEND_MASK: Multi GPS Blending Mask

*Note: This parameter is for advanced users*

Determines which of the accuracy measures Horizontal position, Vertical Position and Speed are used to calculate the weighting on each GPS receiver when soft switching has been selected by setting GPS_AUTO_SWITCH to 2(Blend)

- Bitmask: 0:Horiz Pos,1:Vert Pos,2:Speed

## GPS_DRV_OPTIONS: driver options

*Note: This parameter is for advanced users*

Additional backend specific options

- Bitmask: 0:Use UART2 for moving baseline on ublox,1:Use base station for GPS yaw on SBF,2:Use baudrate 115200 on ublox,3:Use dedicated CAN port b/w GPSes for moving baseline,4:Use ellipsoid height instead of AMSL, 5:Override GPS satellite health of L5 band from L1 health, 6:Enable RTCM full parse even for a single channel, 7:Disable automatic full RTCM parsing when RTCM seen on more than one channel

## GPS_PRIMARY: Primary GPS

*Note: This parameter is for advanced users*

This GPS will be used when GPS_AUTO_SWITCH is 0 and used preferentially with GPS_AUTO_SWITCH = 4.

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|FirstGPS|
|1|SecondGPS|

## GPS_TYPE: 1st GPS type

*Note: This parameter is for advanced users*

GPS type of 1st GPS.Renamed in 4.6 and later to GPS1_TYPE

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
|26|SBF-DualAntenna|

- RebootRequired: True

## GPS_TYPE2: 2nd GPS type.Renamed in 4.6 to GPS2_TYPE

*Note: This parameter is for advanced users*

GPS type of 2nd GPS

- RebootRequired: True

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
|26|SBF-DualAntenna|

## GPS_GNSS_MODE: GNSS system configuration

*Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured).Renamed in 4.6 and later to GPS1_GNSS_MODE.

- Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS

## GPS_GNSS_MODE2: GNSS system configuration.

*Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the second GPS (all unchecked or zero to leave GPS as configured). Renamed in 4.6 and later to GPS2_GNSS_MODE

- Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS

## GPS_RATE_MS: GPS update rate in milliseconds

*Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.Renamed in 4.6 and later to GPS1_RATE_MS

- Units: ms

|Value|Meaning|
|:---:|:---:|
|100|10Hz|
|125|8Hz|
|200|5Hz|

- Range: 50 200

## GPS_RATE_MS2: GPS 2 update rate in milliseconds

*Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.Renamed in 4.6 and later to GPS2_RATE_MS

- Units: ms

|Value|Meaning|
|:---:|:---:|
|100|10Hz|
|125|8Hz|
|200|5Hz|

- Range: 50 200

## GPS_POS1_X: Antenna X position offset

*Note: This parameter is for advanced users*

X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS1_POS_X.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS1_Y: Antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS1_POS_Y.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS1_Z: Antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS1_POS_Z.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS2_X: Antenna X position offset

*Note: This parameter is for advanced users*

X position of the second GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS2_POS_X.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS2_Y: Antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the second GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS2_POS_Y.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_POS2_Z: Antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the second GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS2_POS_Z.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_DELAY_MS: GPS delay in milliseconds

*Note: This parameter is for advanced users*

Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.Renamed in 4.6 and later to GPS1_DELAY_MS.

- Units: ms

- Range: 0 250

- RebootRequired: True

## GPS_DELAY_MS2: GPS 2 delay in milliseconds

*Note: This parameter is for advanced users*

Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.Renamed in 4.6 and later to GPS2_DELAY_MS.

- Units: ms

- Range: 0 250

- RebootRequired: True

## GPS_COM_PORT: GPS physical COM port

*Note: This parameter is for advanced users*

The physical COM port on the connected device, currently only applies to SBF and GSOF GPS,Renamed in 4.6 and later to GPS1_COM_PORT.

- Range: 0 10

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|COM1(RS232) on GSOF|
|1|COM2(TTL) on GSOF|

- RebootRequired: True

## GPS_COM_PORT2: GPS physical COM port

*Note: This parameter is for advanced users*

The physical COM port on the connected device, currently only applies to SBF and GSOF GPS.Renamed in 4.6 and later to GPS1_COM_PORT.

- Range: 0 10

- Increment: 1

- RebootRequired: True

## GPS_CAN_NODEID1: GPS Node ID 1

*Note: This parameter is for advanced users*

GPS Node id for first-discovered GPS.Renamed in 4.6 and later to GPS1_CAN_NODEID.

- ReadOnly: True

## GPS_CAN_NODEID2: GPS Node ID 2

*Note: This parameter is for advanced users*

GPS Node id for second-discovered GPS.Renamed in 4.6 and later to GPS2_CAN_NODEID.

- ReadOnly: True

# GPS1 Parameters

## GPS1_TYPE: GPS type

*Note: This parameter is for advanced users*

GPS type

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
|10|Septentrio(SBF)|
|11|Trimble(GSOF)|
|13|ERB|
|14|MAVLink|
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
|26|Septentrio-DualAntenna(SBF)|

- RebootRequired: True

## GPS1_GNSS_MODE: GNSS system configuration

*Note: This parameter is for advanced users*

Bitmask for what GNSS system to use (all unchecked or zero to leave GPS as configured)

- Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS,7:NAVIC

## GPS1_RATE_MS: GPS update rate in milliseconds

*Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.

- Units: ms

|Value|Meaning|
|:---:|:---:|
|100|10Hz|
|125|8Hz|
|200|5Hz|

- Range: 50 200

## GPS1_POS_X: Antenna X position offset

*Note: This parameter is for advanced users*

X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS1_POS_Y: Antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS1_POS_Z: Antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS1_DELAY_MS: GPS delay in milliseconds

*Note: This parameter is for advanced users*

Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.

- Units: ms

- Range: 0 250

- RebootRequired: True

## GPS1_COM_PORT: GPS physical COM port

*Note: This parameter is for advanced users*

The physical COM port on the connected device, currently only applies to SBF and GSOF GPS

- Range: 0 10

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|COM1(RS232) on GSOF|
|1|COM2(TTL) on GSOF|

- RebootRequired: True

## GPS1_CAN_NODEID: Detected CAN Node ID for GPS

*Note: This parameter is for advanced users*

GPS Node id for GPS.  Detected node unless CAN_OVRIDE is set

- ReadOnly: True

## GPS1_CAN_OVRIDE: DroneCAN GPS NODE ID

*Note: This parameter is for advanced users*

GPS Node id for GPS. If 0 the gps will be automatically selected on a first-come-first-GPS basis.

# GPS1MB Parameters

## GPS1_MB_TYPE: Moving base type

*Note: This parameter is for advanced users*

Controls the type of moving base used if using moving base.This is renamed in 4.6 and later to GPSx_MB_TYPE.

|Value|Meaning|
|:---:|:---:|
|0|Relative to alternate GPS instance|
|1|RelativeToCustomBase|

- RebootRequired: True

## GPS1_MB_OFS_X: Base antenna X position offset

*Note: This parameter is for advanced users*

X position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive X is forward of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_X.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS1_MB_OFS_Y: Base antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Y is to the right of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Y.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS1_MB_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Z is down from the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Z.

- Units: m

- Range: -5 5

- Increment: 0.01

# GPS2 Parameters

## GPS2_TYPE: GPS type

*Note: This parameter is for advanced users*

GPS type

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
|10|Septentrio(SBF)|
|11|Trimble(GSOF)|
|13|ERB|
|14|MAVLink|
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
|26|Septentrio-DualAntenna(SBF)|

- RebootRequired: True

## GPS2_GNSS_MODE: GNSS system configuration

*Note: This parameter is for advanced users*

Bitmask for what GNSS system to use (all unchecked or zero to leave GPS as configured)

- Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS,7:NAVIC

## GPS2_RATE_MS: GPS update rate in milliseconds

*Note: This parameter is for advanced users*

Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.

- Units: ms

|Value|Meaning|
|:---:|:---:|
|100|10Hz|
|125|8Hz|
|200|5Hz|

- Range: 50 200

## GPS2_POS_X: Antenna X position offset

*Note: This parameter is for advanced users*

X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS2_POS_Y: Antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS2_POS_Z: Antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS2_DELAY_MS: GPS delay in milliseconds

*Note: This parameter is for advanced users*

Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.

- Units: ms

- Range: 0 250

- RebootRequired: True

## GPS2_COM_PORT: GPS physical COM port

*Note: This parameter is for advanced users*

The physical COM port on the connected device, currently only applies to SBF and GSOF GPS

- Range: 0 10

- Increment: 1

|Value|Meaning|
|:---:|:---:|
|0|COM1(RS232) on GSOF|
|1|COM2(TTL) on GSOF|

- RebootRequired: True

## GPS2_CAN_NODEID: Detected CAN Node ID for GPS

*Note: This parameter is for advanced users*

GPS Node id for GPS.  Detected node unless CAN_OVRIDE is set

- ReadOnly: True

## GPS2_CAN_OVRIDE: DroneCAN GPS NODE ID

*Note: This parameter is for advanced users*

GPS Node id for GPS. If 0 the gps will be automatically selected on a first-come-first-GPS basis.

# GPS2MB Parameters

## GPS2_MB_TYPE: Moving base type

*Note: This parameter is for advanced users*

Controls the type of moving base used if using moving base.This is renamed in 4.6 and later to GPSx_MB_TYPE.

|Value|Meaning|
|:---:|:---:|
|0|Relative to alternate GPS instance|
|1|RelativeToCustomBase|

- RebootRequired: True

## GPS2_MB_OFS_X: Base antenna X position offset

*Note: This parameter is for advanced users*

X position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive X is forward of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_X.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS2_MB_OFS_Y: Base antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Y is to the right of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Y.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS2_MB_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Z is down from the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Z.

- Units: m

- Range: -5 5

- Increment: 0.01

# GPSMB1 Parameters

## GPS_MB1_TYPE: Moving base type

*Note: This parameter is for advanced users*

Controls the type of moving base used if using moving base.This is renamed in 4.6 and later to GPSx_MB_TYPE.

|Value|Meaning|
|:---:|:---:|
|0|Relative to alternate GPS instance|
|1|RelativeToCustomBase|

- RebootRequired: True

## GPS_MB1_OFS_X: Base antenna X position offset

*Note: This parameter is for advanced users*

X position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive X is forward of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_X.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB1_OFS_Y: Base antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Y is to the right of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Y.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB1_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Z is down from the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Z.

- Units: m

- Range: -5 5

- Increment: 0.01

# GPSMB2 Parameters

## GPS_MB2_TYPE: Moving base type

*Note: This parameter is for advanced users*

Controls the type of moving base used if using moving base.This is renamed in 4.6 and later to GPSx_MB_TYPE.

|Value|Meaning|
|:---:|:---:|
|0|Relative to alternate GPS instance|
|1|RelativeToCustomBase|

- RebootRequired: True

## GPS_MB2_OFS_X: Base antenna X position offset

*Note: This parameter is for advanced users*

X position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive X is forward of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_X.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB2_OFS_Y: Base antenna Y position offset

*Note: This parameter is for advanced users*

Y position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Y is to the right of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Y.

- Units: m

- Range: -5 5

- Increment: 0.01

## GPS_MB2_OFS_Z: Base antenna Z position offset

*Note: This parameter is for advanced users*

Z position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Z is down from the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Z.

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

This enables optional temperature calibration features. Setting of the Persist bits will save the temperature and/or accelerometer calibration parameters in the bootloader sector on the next update of the bootloader.

- Bitmask: 0:PersistTemps, 1:PersistAccels

## INS_RAW_LOG_OPT: Raw logging options

*Note: This parameter is for advanced users*

Raw logging options bitmask

- Bitmask: 0:Log primary gyro only, 1:Log all gyros, 2:Post filter, 3: Pre and post filter

# INS4 Parameters

## INS4_USE: Use first IMU for attitude, velocity and position estimates

*Note: This parameter is for advanced users*

Use first IMU for attitude, velocity and position estimates

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS4_ACC_ID: Accelerometer ID

*Note: This parameter is for advanced users*

Accelerometer sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS4_ACCSCAL_X: Accelerometer scaling of X axis

*Note: This parameter is for advanced users*

Accelerometer scaling of X axis.  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS4_ACCSCAL_Y: Accelerometer scaling of Y axis

*Note: This parameter is for advanced users*

Accelerometer scaling of Y axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS4_ACCSCAL_Z: Accelerometer scaling of Z axis

*Note: This parameter is for advanced users*

Accelerometer scaling of Z axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS4_ACCOFFS_X: Accelerometer offsets of X axis

*Note: This parameter is for advanced users*

Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS4_ACCOFFS_Y: Accelerometer offsets of Y axis

*Note: This parameter is for advanced users*

Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS4_ACCOFFS_Z: Accelerometer offsets of Z axis

*Note: This parameter is for advanced users*

Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS4_POS_X: IMU accelerometer X position

*Note: This parameter is for advanced users*

X position of the first IMU Accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS4_POS_Y: IMU accelerometer Y position

*Note: This parameter is for advanced users*

Y position of the first IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS4_POS_Z: IMU accelerometer Z position

*Note: This parameter is for advanced users*

Z position of the first IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS4_ACC_CALTEMP: Calibration temperature for accelerometer

*Note: This parameter is for advanced users*

Temperature that the accelerometer was calibrated at

- Units: degC

- Calibration: 1

## INS4_GYR_ID: Gyro ID

*Note: This parameter is for advanced users*

Gyro sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS4_GYROFFS_X: Gyro offsets of X axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS4_GYROFFS_Y: Gyro offsets of Y axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS4_GYROFFS_Z: Gyro offsets of Z axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS4_GYR_CALTEMP: Calibration temperature for gyroscope

*Note: This parameter is for advanced users*

Temperature that the gyroscope was calibrated at

- Units: degC

- Calibration: 1

# INS4TCAL Parameters

## INS4_TCAL_ENABLE: Enable temperature calibration

*Note: This parameter is for advanced users*

Enable the use of temperature calibration parameters for this IMU. For automatic learning set to 2 and also set the INS_TCALn_TMAX to the target temperature, then reboot

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|LearnCalibration|

- RebootRequired: True

## INS4_TCAL_TMIN: Temperature calibration min

*Note: This parameter is for advanced users*

The minimum temperature that the calibration is valid for

- Range: -70 80

- Units: degC

- Calibration: 1

## INS4_TCAL_TMAX: Temperature calibration max

*Note: This parameter is for advanced users*

The maximum temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration

- Range: -70 80

- Units: degC

- Calibration: 1

## INS4_TCAL_ACC1_X: Accelerometer 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC1_Y: Accelerometer 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC1_Z: Accelerometer 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC2_X: Accelerometer 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC2_Y: Accelerometer 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC2_Z: Accelerometer 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC3_X: Accelerometer 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC3_Y: Accelerometer 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_ACC3_Z: Accelerometer 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR1_X: Gyroscope 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR1_Y: Gyroscope 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR1_Z: Gyroscope 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR2_X: Gyroscope 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR2_Y: Gyroscope 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR2_Z: Gyroscope 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR3_X: Gyroscope 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR3_Y: Gyroscope 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS4_TCAL_GYR3_Z: Gyroscope 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

# INS5 Parameters

## INS5_USE: Use first IMU for attitude, velocity and position estimates

*Note: This parameter is for advanced users*

Use first IMU for attitude, velocity and position estimates

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS5_ACC_ID: Accelerometer ID

*Note: This parameter is for advanced users*

Accelerometer sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS5_ACCSCAL_X: Accelerometer scaling of X axis

*Note: This parameter is for advanced users*

Accelerometer scaling of X axis.  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS5_ACCSCAL_Y: Accelerometer scaling of Y axis

*Note: This parameter is for advanced users*

Accelerometer scaling of Y axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS5_ACCSCAL_Z: Accelerometer scaling of Z axis

*Note: This parameter is for advanced users*

Accelerometer scaling of Z axis  Calculated during acceleration calibration routine

- Range: 0.8 1.2

- Calibration: 1

## INS5_ACCOFFS_X: Accelerometer offsets of X axis

*Note: This parameter is for advanced users*

Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS5_ACCOFFS_Y: Accelerometer offsets of Y axis

*Note: This parameter is for advanced users*

Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS5_ACCOFFS_Z: Accelerometer offsets of Z axis

*Note: This parameter is for advanced users*

Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations

- Units: m/s/s

- Range: -3.5 3.5

- Calibration: 1

## INS5_POS_X: IMU accelerometer X position

*Note: This parameter is for advanced users*

X position of the first IMU Accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS5_POS_Y: IMU accelerometer Y position

*Note: This parameter is for advanced users*

Y position of the first IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS5_POS_Z: IMU accelerometer Z position

*Note: This parameter is for advanced users*

Z position of the first IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.

- Units: m

- Range: -5 5

- Increment: 0.01

## INS5_ACC_CALTEMP: Calibration temperature for accelerometer

*Note: This parameter is for advanced users*

Temperature that the accelerometer was calibrated at

- Units: degC

- Calibration: 1

## INS5_GYR_ID: Gyro ID

*Note: This parameter is for advanced users*

Gyro sensor ID, taking into account its type, bus and instance

- ReadOnly: True

## INS5_GYROFFS_X: Gyro offsets of X axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS5_GYROFFS_Y: Gyro offsets of Y axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS5_GYROFFS_Z: Gyro offsets of Z axis

*Note: This parameter is for advanced users*

Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations

- Units: rad/s

- Calibration: 1

## INS5_GYR_CALTEMP: Calibration temperature for gyroscope

*Note: This parameter is for advanced users*

Temperature that the gyroscope was calibrated at

- Units: degC

- Calibration: 1

# INS5TCAL Parameters

## INS5_TCAL_ENABLE: Enable temperature calibration

*Note: This parameter is for advanced users*

Enable the use of temperature calibration parameters for this IMU. For automatic learning set to 2 and also set the INS_TCALn_TMAX to the target temperature, then reboot

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|LearnCalibration|

- RebootRequired: True

## INS5_TCAL_TMIN: Temperature calibration min

*Note: This parameter is for advanced users*

The minimum temperature that the calibration is valid for

- Range: -70 80

- Units: degC

- Calibration: 1

## INS5_TCAL_TMAX: Temperature calibration max

*Note: This parameter is for advanced users*

The maximum temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration

- Range: -70 80

- Units: degC

- Calibration: 1

## INS5_TCAL_ACC1_X: Accelerometer 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC1_Y: Accelerometer 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC1_Z: Accelerometer 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC2_X: Accelerometer 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC2_Y: Accelerometer 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC2_Z: Accelerometer 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC3_X: Accelerometer 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC3_Y: Accelerometer 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_ACC3_Z: Accelerometer 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR1_X: Gyroscope 1st order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR1_Y: Gyroscope 1st order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR1_Z: Gyroscope 1st order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 1st order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR2_X: Gyroscope 2nd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR2_Y: Gyroscope 2nd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR2_Z: Gyroscope 2nd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 2nd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR3_X: Gyroscope 3rd order temperature coefficient X axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR3_Y: Gyroscope 3rd order temperature coefficient Y axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

## INS5_TCAL_GYR3_Z: Gyroscope 3rd order temperature coefficient Z axis

*Note: This parameter is for advanced users*

This is the 3rd order temperature coefficient from a temperature calibration

- Calibration: 1

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

Harmonic Notch Filter base center frequency in Hz. This is the center frequency for static notches, the center frequency for Throttle based notches at the reference thrust value, and the minimum limit of center frequency variation for all other notch types. This should always be set lower than half the backend gyro rate (which is typically 1Khz).

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

- Bitmask: 0:  1st harmonic, 1:  2nd harmonic, 2:  3rd harmonic, 3:  4th harmonic, 4:  5th harmonic, 5:  6th harmonic, 6:  7th harmonic, 7:  8th harmonic, 8:  9th harmonic, 9:  10th harmonic, 10: 11th harmonic, 11: 12th harmonic, 12: 13th harmonic, 13: 14th harmonic, 14: 15th harmonic, 15: 16th harmonic

- RebootRequired: True

## INS_HNTC2_REF: Harmonic Notch Filter reference value

*Note: This parameter is for advanced users*

A reference value of zero disables dynamic updates on the Harmonic Notch Filter and a positive value enables dynamic updates on the Harmonic Notch Filter.  For throttle-based scaling, this parameter is the reference value associated with the specified frequency to facilitate frequency scaling of the Harmonic Notch Filter. For RPM and ESC telemetry based tracking, this parameter is set to 1 to enable the Harmonic Notch Filter using the RPM sensor or ESC telemetry set to measure rotor speed.  The sensor data is converted to Hz automatically for use in the Harmonic Notch Filter.  This reference value may also be used to scale the sensor data, if required.  For example, rpm sensor data is required to measure heli motor RPM. Therefore the reference value can be used to scale the RPM sensor to the rotor RPM.

- Range: 0.0 1.0

- RebootRequired: True

## INS_HNTC2_MODE: Harmonic Notch Filter dynamic frequency tracking mode

*Note: This parameter is for advanced users*

Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based harmonic notch cannot be used on fixed wing only planes. It can for Copters, QuaadPlane(while in VTOL modes), and Rovers.

- Range: 0 5

|Value|Meaning|
|:---:|:---:|
|0|Fixed|
|1|Throttle|
|2|RPM Sensor|
|3|ESC Telemetry|
|4|Dynamic FFT|
|5|Second RPM Sensor|

## INS_HNTC2_OPTS: Harmonic Notch Filter options

*Note: This parameter is for advanced users*

Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Multi-Source attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.

- Bitmask: 0:Double notch,1:Multi-Source,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch, 5:Use min freq on RPM source failure, 6:Quintuple notch

- RebootRequired: True

## INS_HNTC2_FM_RAT: Throttle notch min frequency ratio

*Note: This parameter is for advanced users*

The minimum ratio below the configured frequency to take throttle based notch filters when flying at a throttle level below the reference throttle. Note that lower frequency notch filters will have more phase lag. If you want throttle based notch filtering to be effective at a throttle up to 30% below the configured notch frequency then set this parameter to 0.7. The default of 1.0 means the notch will not go below the frequency in the FREQ parameter.

- Range: 0.1 1.0

# INSHNTC3 Parameters

## INS_HNTC3_ENABLE: Harmonic Notch Filter enable

*Note: This parameter is for advanced users*

Harmonic Notch Filter enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_HNTC3_FREQ: Harmonic Notch Filter base frequency

*Note: This parameter is for advanced users*

Harmonic Notch Filter base center frequency in Hz. This is the center frequency for static notches, the center frequency for Throttle based notches at the reference thrust value, and the minimum limit of center frequency variation for all other notch types. This should always be set lower than half the backend gyro rate (which is typically 1Khz).

- Range: 10 495

- Units: Hz

## INS_HNTC3_BW: Harmonic Notch Filter bandwidth

*Note: This parameter is for advanced users*

Harmonic Notch Filter bandwidth in Hz. This is typically set to half the base frequency. The ratio of base frequency to bandwidth determines the notch quality factor and is fixed across harmonics.

- Range: 5 250

- Units: Hz

## INS_HNTC3_ATT: Harmonic Notch Filter attenuation

*Note: This parameter is for advanced users*

Harmonic Notch Filter attenuation in dB. Values greater than 40dB will typically produce a hard notch rather than a modest attenuation of motor noise.

- Range: 5 50

- Units: dB

## INS_HNTC3_HMNCS: Harmonic Notch Filter harmonics

*Note: This parameter is for advanced users*

Bitmask of harmonic frequencies to apply Harmonic Notch Filter to. This option takes effect on the next reboot. A value of 0 disables this filter. The first harmonic refers to the base frequency.

- Bitmask: 0:  1st harmonic, 1:  2nd harmonic, 2:  3rd harmonic, 3:  4th harmonic, 4:  5th harmonic, 5:  6th harmonic, 6:  7th harmonic, 7:  8th harmonic, 8:  9th harmonic, 9:  10th harmonic, 10: 11th harmonic, 11: 12th harmonic, 12: 13th harmonic, 13: 14th harmonic, 14: 15th harmonic, 15: 16th harmonic

- RebootRequired: True

## INS_HNTC3_REF: Harmonic Notch Filter reference value

*Note: This parameter is for advanced users*

A reference value of zero disables dynamic updates on the Harmonic Notch Filter and a positive value enables dynamic updates on the Harmonic Notch Filter.  For throttle-based scaling, this parameter is the reference value associated with the specified frequency to facilitate frequency scaling of the Harmonic Notch Filter. For RPM and ESC telemetry based tracking, this parameter is set to 1 to enable the Harmonic Notch Filter using the RPM sensor or ESC telemetry set to measure rotor speed.  The sensor data is converted to Hz automatically for use in the Harmonic Notch Filter.  This reference value may also be used to scale the sensor data, if required.  For example, rpm sensor data is required to measure heli motor RPM. Therefore the reference value can be used to scale the RPM sensor to the rotor RPM.

- Range: 0.0 1.0

- RebootRequired: True

## INS_HNTC3_MODE: Harmonic Notch Filter dynamic frequency tracking mode

*Note: This parameter is for advanced users*

Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based harmonic notch cannot be used on fixed wing only planes. It can for Copters, QuaadPlane(while in VTOL modes), and Rovers.

- Range: 0 5

|Value|Meaning|
|:---:|:---:|
|0|Fixed|
|1|Throttle|
|2|RPM Sensor|
|3|ESC Telemetry|
|4|Dynamic FFT|
|5|Second RPM Sensor|

## INS_HNTC3_OPTS: Harmonic Notch Filter options

*Note: This parameter is for advanced users*

Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Multi-Source attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.

- Bitmask: 0:Double notch,1:Multi-Source,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch, 5:Use min freq on RPM source failure, 6:Quintuple notch

- RebootRequired: True

## INS_HNTC3_FM_RAT: Throttle notch min frequency ratio

*Note: This parameter is for advanced users*

The minimum ratio below the configured frequency to take throttle based notch filters when flying at a throttle level below the reference throttle. Note that lower frequency notch filters will have more phase lag. If you want throttle based notch filtering to be effective at a throttle up to 30% below the configured notch frequency then set this parameter to 0.7. The default of 1.0 means the notch will not go below the frequency in the FREQ parameter.

- Range: 0.1 1.0

# INSHNTC4 Parameters

## INS_HNTC4_ENABLE: Harmonic Notch Filter enable

*Note: This parameter is for advanced users*

Harmonic Notch Filter enable

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## INS_HNTC4_FREQ: Harmonic Notch Filter base frequency

*Note: This parameter is for advanced users*

Harmonic Notch Filter base center frequency in Hz. This is the center frequency for static notches, the center frequency for Throttle based notches at the reference thrust value, and the minimum limit of center frequency variation for all other notch types. This should always be set lower than half the backend gyro rate (which is typically 1Khz).

- Range: 10 495

- Units: Hz

## INS_HNTC4_BW: Harmonic Notch Filter bandwidth

*Note: This parameter is for advanced users*

Harmonic Notch Filter bandwidth in Hz. This is typically set to half the base frequency. The ratio of base frequency to bandwidth determines the notch quality factor and is fixed across harmonics.

- Range: 5 250

- Units: Hz

## INS_HNTC4_ATT: Harmonic Notch Filter attenuation

*Note: This parameter is for advanced users*

Harmonic Notch Filter attenuation in dB. Values greater than 40dB will typically produce a hard notch rather than a modest attenuation of motor noise.

- Range: 5 50

- Units: dB

## INS_HNTC4_HMNCS: Harmonic Notch Filter harmonics

*Note: This parameter is for advanced users*

Bitmask of harmonic frequencies to apply Harmonic Notch Filter to. This option takes effect on the next reboot. A value of 0 disables this filter. The first harmonic refers to the base frequency.

- Bitmask: 0:  1st harmonic, 1:  2nd harmonic, 2:  3rd harmonic, 3:  4th harmonic, 4:  5th harmonic, 5:  6th harmonic, 6:  7th harmonic, 7:  8th harmonic, 8:  9th harmonic, 9:  10th harmonic, 10: 11th harmonic, 11: 12th harmonic, 12: 13th harmonic, 13: 14th harmonic, 14: 15th harmonic, 15: 16th harmonic

- RebootRequired: True

## INS_HNTC4_REF: Harmonic Notch Filter reference value

*Note: This parameter is for advanced users*

A reference value of zero disables dynamic updates on the Harmonic Notch Filter and a positive value enables dynamic updates on the Harmonic Notch Filter.  For throttle-based scaling, this parameter is the reference value associated with the specified frequency to facilitate frequency scaling of the Harmonic Notch Filter. For RPM and ESC telemetry based tracking, this parameter is set to 1 to enable the Harmonic Notch Filter using the RPM sensor or ESC telemetry set to measure rotor speed.  The sensor data is converted to Hz automatically for use in the Harmonic Notch Filter.  This reference value may also be used to scale the sensor data, if required.  For example, rpm sensor data is required to measure heli motor RPM. Therefore the reference value can be used to scale the RPM sensor to the rotor RPM.

- Range: 0.0 1.0

- RebootRequired: True

## INS_HNTC4_MODE: Harmonic Notch Filter dynamic frequency tracking mode

*Note: This parameter is for advanced users*

Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based harmonic notch cannot be used on fixed wing only planes. It can for Copters, QuaadPlane(while in VTOL modes), and Rovers.

- Range: 0 5

|Value|Meaning|
|:---:|:---:|
|0|Fixed|
|1|Throttle|
|2|RPM Sensor|
|3|ESC Telemetry|
|4|Dynamic FFT|
|5|Second RPM Sensor|

## INS_HNTC4_OPTS: Harmonic Notch Filter options

*Note: This parameter is for advanced users*

Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Multi-Source attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.

- Bitmask: 0:Double notch,1:Multi-Source,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch, 5:Use min freq on RPM source failure, 6:Quintuple notch

- RebootRequired: True

## INS_HNTC4_FM_RAT: Throttle notch min frequency ratio

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

Harmonic Notch Filter base center frequency in Hz. This is the center frequency for static notches, the center frequency for Throttle based notches at the reference thrust value, and the minimum limit of center frequency variation for all other notch types. This should always be set lower than half the backend gyro rate (which is typically 1Khz).

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

- Bitmask: 0:  1st harmonic, 1:  2nd harmonic, 2:  3rd harmonic, 3:  4th harmonic, 4:  5th harmonic, 5:  6th harmonic, 6:  7th harmonic, 7:  8th harmonic, 8:  9th harmonic, 9:  10th harmonic, 10: 11th harmonic, 11: 12th harmonic, 12: 13th harmonic, 13: 14th harmonic, 14: 15th harmonic, 15: 16th harmonic

- RebootRequired: True

## INS_HNTCH_REF: Harmonic Notch Filter reference value

*Note: This parameter is for advanced users*

A reference value of zero disables dynamic updates on the Harmonic Notch Filter and a positive value enables dynamic updates on the Harmonic Notch Filter.  For throttle-based scaling, this parameter is the reference value associated with the specified frequency to facilitate frequency scaling of the Harmonic Notch Filter. For RPM and ESC telemetry based tracking, this parameter is set to 1 to enable the Harmonic Notch Filter using the RPM sensor or ESC telemetry set to measure rotor speed.  The sensor data is converted to Hz automatically for use in the Harmonic Notch Filter.  This reference value may also be used to scale the sensor data, if required.  For example, rpm sensor data is required to measure heli motor RPM. Therefore the reference value can be used to scale the RPM sensor to the rotor RPM.

- Range: 0.0 1.0

- RebootRequired: True

## INS_HNTCH_MODE: Harmonic Notch Filter dynamic frequency tracking mode

*Note: This parameter is for advanced users*

Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based harmonic notch cannot be used on fixed wing only planes. It can for Copters, QuaadPlane(while in VTOL modes), and Rovers.

- Range: 0 5

|Value|Meaning|
|:---:|:---:|
|0|Fixed|
|1|Throttle|
|2|RPM Sensor|
|3|ESC Telemetry|
|4|Dynamic FFT|
|5|Second RPM Sensor|

## INS_HNTCH_OPTS: Harmonic Notch Filter options

*Note: This parameter is for advanced users*

Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Multi-Source attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.

- Bitmask: 0:Double notch,1:Multi-Source,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch, 5:Use min freq on RPM source failure, 6:Quintuple notch

- RebootRequired: True

## INS_HNTCH_FM_RAT: Throttle notch min frequency ratio

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

# KDE Parameters

## KDE_NPOLE: Number of motor poles

Sets the number of motor poles to calculate the correct RPM value

# LOG Parameters

## LOG_BACKEND_TYPE: AP_Logger Backend Storage type

Bitmap of what Logger backend types to enable. Block-based logging is available on SITL and boards with dataflash chips. Multiple backends can be selected.

- Bitmask: 0:File,1:MAVLink,2:Block

## LOG_FILE_BUFSIZE: Logging File and Block Backend buffer size max (in kibibytes)

The File and Block backends use a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging but increases memory usage.  This buffer size may be reduced to free up available memory

- Units: KiB

- Range: 4 200

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

- Range: 2 1000

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

## LOG_MAX_FILES: Maximum number of log files

*Note: This parameter is for advanced users*

This sets the maximum number of log file that will be written on dataflash or sd card before starting to rotate log number. Limit is capped at 500 logs.

- Range: 2 500

- Increment: 1

- RebootRequired: True

# MAV Parameters

## MAV_SYSID: MAVLink system ID of this vehicle

*Note: This parameter is for advanced users*

Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network.

- Range: 1 255

## MAV_GCS_SYSID: My ground station number

*Note: This parameter is for advanced users*

This sets what MAVLink source system IDs are accepted for GCS failsafe handling, RC overrides and manual control. When MAV_GCS_SYSID_HI is less than MAV_GCS_SYSID then only this value is considered to be a GCS. When MAV_GCS_SYSID_HI is greater than or equal to MAV_GCS_SYSID then the range of values between MAV_GCS_SYSID and MAV_GCS_SYSID_HI (inclusive) are all treated as valid GCS MAVLink system IDs

- Range: 1 255

- Increment: 1

## MAV_GCS_SYSID_HI: ground station system ID, maximum

*Note: This parameter is for advanced users*

Upper limit of MAVLink source system IDs considered to be from the GCS. When this is less than MAV_GCS_SYSID then only MAV_GCS_SYSID is used as GCS ID. When this is greater than or equal to MAV_GCS_SYSID then the range of values from MAV_GCS_SYSID to MAV_GCS_SYSID_HI (inclusive) is treated as a GCS ID.

- Range: 0 255

- Increment: 1

## MAV_OPTIONS: MAVLink Options

*Note: This parameter is for advanced users*

Alters various behaviour of the MAVLink interface

- Bitmask: 0:Accept MAVLink only from system IDs given by MAV_SYSID_GCS and MAV_SYSID_GCS_HI

## MAV_TELEM_DELAY: Telemetry startup delay

*Note: This parameter is for advanced users*

The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up

- Units: s

- Range: 0 30

- Increment: 1

# MAV1 Parameters

## MAV1_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV1_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV2 Parameters

## MAV2_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV2_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV3 Parameters

## MAV3_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV3_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV4 Parameters

## MAV4_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV4_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV5 Parameters

## MAV5_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV5_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV6 Parameters

## MAV6_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV6_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV7 Parameters

## MAV7_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV7_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV8 Parameters

## MAV8_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV8_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV9 Parameters

## MAV9_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV9_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV10 Parameters

## MAV10_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV10_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV11 Parameters

## MAV11_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV11_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV12 Parameters

## MAV12_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV12_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV13 Parameters

## MAV13_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV13_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV14 Parameters

## MAV14_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV14_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV15 Parameters

## MAV15_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV15_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV16 Parameters

## MAV16_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV16_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV17 Parameters

## MAV17_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV17_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV18 Parameters

## MAV18_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV18_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV19 Parameters

## MAV19_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV19_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV20 Parameters

## MAV20_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV20_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV21 Parameters

## MAV21_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV21_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV22 Parameters

## MAV22_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV22_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV23 Parameters

## MAV23_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV23_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV24 Parameters

## MAV24_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV24_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV25 Parameters

## MAV25_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV25_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV26 Parameters

## MAV26_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV26_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV27 Parameters

## MAV27_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV27_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV28 Parameters

## MAV28_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV28_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV29 Parameters

## MAV29_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV29_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV30 Parameters

## MAV30_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV30_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV31 Parameters

## MAV31_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV31_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

# MAV32 Parameters

## MAV32_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_EXT_STAT: Extended status stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_RC_CHAN: RC Channel stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_RAW_CTRL: Raw Control stream rate

*Note: This parameter is for advanced users*

MAVLink Raw Control stream rate of SERVO_OUT

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_POSITION: Position stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_EXTRA1: Extra data type 1 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_EXTRA2: Extra data type 2 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of VFR_HUD

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_EXTRA3: Extra data type 3 stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_PARAMS: Parameter stream rate

*Note: This parameter is for advanced users*

MAVLink Stream rate of PARAM_VALUE

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_ADSB: ADSB stream rate

*Note: This parameter is for advanced users*

MAVLink ADSB stream rate

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## MAV32_OPTIONS: Bitmask for configuring this telemetry channel

Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.

- RebootRequired: True

- Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC

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

A bitmask to set some MSP specific options: EnableTelemetryMode-allows "push" mode telemetry when only rx line of OSD ic connected to autopilot,  EnableBTFLFonts-uses indexes corresponding to Betaflight fonts if OSD uses those instead of ArduPilot fonts. EnableINAVFonts uses INAV fonts and overrides EnableBTFLFonts if that option is enabled.

- Bitmask: 0:EnableTelemetryMode, 1: unused, 2:EnableBTFLFonts, 3:EnableINAVFonts

# NET Parameters

## NET_ENABLE: Networking Enable

*Note: This parameter is for advanced users*

Networking Enable

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

- RebootRequired: True

## NET_NETMASK: IP Subnet mask

*Note: This parameter is for advanced users*

Allows setting static subnet mask. The value is a count of consecutive bits. Examples: 24 = 255.255.255.0, 16 = 255.255.0.0

- Range: 0 32

- RebootRequired: True

## NET_DHCP: DHCP client

*Note: This parameter is for advanced users*

Enable/Disable DHCP client

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

- RebootRequired: True

## NET_TESTS: Test enable flags

*Note: This parameter is for advanced users*

Enable/Disable networking tests

- Bitmask: 0:UDP echo test,1:TCP echo test, 2:TCP discard test, 3:TCP reflect test

- RebootRequired: True

## NET_OPTIONS: Networking options

*Note: This parameter is for advanced users*

Networking options

- Bitmask: 0:EnablePPP Ethernet gateway, 1:Enable CAN1 multicast endpoint, 2:Enable CAN2 multicast endpoint, 3:Enable CAN1 multicast bridged, 4:Enable CAN2 multicast bridged, 5:DisablePPPTimeout, 6:DisablePPPEchoLimit, 7:Capture to file

- RebootRequired: True

# NETGWADDR Parameters

## NET_GWADDR0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_GWADDR1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_GWADDR2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_GWADDR3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

# NETIPADDR Parameters

## NET_IPADDR0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_IPADDR1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_IPADDR2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_IPADDR3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

# NETMACADDR Parameters

## NET_MACADDR0: MAC Address 1st byte

*Note: This parameter is for advanced users*

MAC address 1st byte

- Range: 0 255

- RebootRequired: True

## NET_MACADDR1: MAC Address 2nd byte

*Note: This parameter is for advanced users*

MAC address 2nd byte

- Range: 0 255

- RebootRequired: True

## NET_MACADDR2: MAC Address 3rd byte

*Note: This parameter is for advanced users*

MAC address 3rd byte

- Range: 0 255

- RebootRequired: True

## NET_MACADDR3: MAC Address 4th byte

*Note: This parameter is for advanced users*

MAC address 4th byte

- Range: 0 255

- RebootRequired: True

## NET_MACADDR4: MAC Address 5th byte

*Note: This parameter is for advanced users*

MAC address 5th byte

- Range: 0 255

- RebootRequired: True

## NET_MACADDR5: MAC Address 6th byte

*Note: This parameter is for advanced users*

MAC address 6th byte

- Range: 0 255

- RebootRequired: True

# NETP1 Parameters

## NET_P1_TYPE: Port type

*Note: This parameter is for advanced users*

Port type for network serial port. For the two client types a valid destination IP address must be set. For the two server types either 0.0.0.0 or a local address can be used. The UDP client type will use broadcast if the IP is set to 255.255.255.255 and will use UDP multicast if the IP is in the multicast address range.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UDP client|
|2|UDP server|
|3|TCP client|
|4|TCP server|

- RebootRequired: True

## NET_P1_PROTOCOL: Protocol

*Note: This parameter is for advanced users*

Networked serial port protocol

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## NET_P1_PORT: Port number

*Note: This parameter is for advanced users*

Port number

- Range: 0 65535

- RebootRequired: True

# NETP1IP Parameters

## NET_P1_IP0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P1_IP1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P1_IP2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_P1_IP3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

# NETP2 Parameters

## NET_P2_TYPE: Port type

*Note: This parameter is for advanced users*

Port type for network serial port. For the two client types a valid destination IP address must be set. For the two server types either 0.0.0.0 or a local address can be used. The UDP client type will use broadcast if the IP is set to 255.255.255.255 and will use UDP multicast if the IP is in the multicast address range.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UDP client|
|2|UDP server|
|3|TCP client|
|4|TCP server|

- RebootRequired: True

## NET_P2_PROTOCOL: Protocol

*Note: This parameter is for advanced users*

Networked serial port protocol

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## NET_P2_PORT: Port number

*Note: This parameter is for advanced users*

Port number

- Range: 0 65535

- RebootRequired: True

# NETP2IP Parameters

## NET_P2_IP0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P2_IP1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P2_IP2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_P2_IP3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

# NETP3 Parameters

## NET_P3_TYPE: Port type

*Note: This parameter is for advanced users*

Port type for network serial port. For the two client types a valid destination IP address must be set. For the two server types either 0.0.0.0 or a local address can be used. The UDP client type will use broadcast if the IP is set to 255.255.255.255 and will use UDP multicast if the IP is in the multicast address range.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UDP client|
|2|UDP server|
|3|TCP client|
|4|TCP server|

- RebootRequired: True

## NET_P3_PROTOCOL: Protocol

*Note: This parameter is for advanced users*

Networked serial port protocol

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## NET_P3_PORT: Port number

*Note: This parameter is for advanced users*

Port number

- Range: 0 65535

- RebootRequired: True

# NETP3IP Parameters

## NET_P3_IP0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P3_IP1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P3_IP2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_P3_IP3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

# NETP4 Parameters

## NET_P4_TYPE: Port type

*Note: This parameter is for advanced users*

Port type for network serial port. For the two client types a valid destination IP address must be set. For the two server types either 0.0.0.0 or a local address can be used. The UDP client type will use broadcast if the IP is set to 255.255.255.255 and will use UDP multicast if the IP is in the multicast address range.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|UDP client|
|2|UDP server|
|3|TCP client|
|4|TCP server|

- RebootRequired: True

## NET_P4_PROTOCOL: Protocol

*Note: This parameter is for advanced users*

Networked serial port protocol

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## NET_P4_PORT: Port number

*Note: This parameter is for advanced users*

Port number

- Range: 0 65535

- RebootRequired: True

# NETP4IP Parameters

## NET_P4_IP0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P4_IP1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_P4_IP2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_P4_IP3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

# NETREMPPPIP Parameters

## NET_REMPPP_IP0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_REMPPP_IP1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_REMPPP_IP2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_REMPPP_IP3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

# NETTESTIP Parameters

## NET_TEST_IP0: IPv4 Address 1st byte

IPv4 address. Example: 192.xxx.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_TEST_IP1: IPv4 Address 2nd byte

IPv4 address. Example: xxx.168.xxx.xxx

- Range: 0 255

- RebootRequired: True

## NET_TEST_IP2: IPv4 Address 3rd byte

IPv4 address. Example: xxx.xxx.144.xxx

- Range: 0 255

- RebootRequired: True

## NET_TEST_IP3: IPv4 Address 4th byte

IPv4 address. Example: xxx.xxx.xxx.14

- Range: 0 255

- RebootRequired: True

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

- Bitmask: 0:Built-in LED, 1:Internal ToshibaLED, 2:External ToshibaLED, 3:External PCA9685, 4:Oreo LED, 5:DroneCAN, 6:NCP5623 External, 7:NCP5623 Internal, 8:NeoPixel, 9:ProfiLED, 10:Scripting, 11:DShot, 12:ProfiLED_SPI, 13:LP5562 External, 14: LP5562 Internal, 15:IS31FL3195 External, 16: IS31FL3195 Internal, 17: DiscreteRGB, 18: NeoPixelRGB, 19:ProfiLED_IOMCU

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

# RC Parameters

## RC_OVERRIDE_TIME: RC override timeout

*Note: This parameter is for advanced users*

Timeout after which RC overrides will no longer be used, and RC input will resume, 0 will disable RC overrides, -1 will never timeout, and continue using overrides until they are disabled

- Range: 0.0 120.0

- Units: s

## RC_OPTIONS: RC options

*Note: This parameter is for advanced users*

RC input options

- Bitmask: 0:Ignore RC Receiver, 1:Ignore MAVLink Overrides, 2:Ignore Receiver Failsafe bit but allow other RC failsafes if setup, 3:FPort Pad, 4:Log RC input bytes, 5:Arming check throttle for 0 input, 6:Skip the arming check for neutral Roll/Pitch/Yaw sticks, 7:Allow Switch reverse, 8:Use passthrough for CRSF telemetry, 9:Suppress CRSF mode/rate message for ELRS systems,10:Enable multiple receiver support, 11:Use Link Quality for RSSI with CRSF, 12:Annotate CRSF flight mode with * on disarm, 13: Use 420kbaud for ELRS protocol

## RC_PROTOCOLS: RC protocols enabled

*Note: This parameter is for advanced users*

Bitmask of enabled RC protocols. Allows narrowing the protocol detection to only specific types of RC receivers which can avoid issues with incorrect detection. Set to 1 to enable all protocols.

- Bitmask: 0:All,1:PPM,2:IBUS,3:SBUS,4:SBUS_NI,5:DSM,6:SUMD,7:SRXL,8:SRXL2,9:CRSF,10:ST24,11:FPORT,12:FPORT2,13:FastSBUS,14:DroneCAN,15:Ghost,16:MAVRadio

## RC_FS_TIMEOUT: RC Failsafe timeout

RC failsafe will trigger this many seconds after loss of RC

- Range: 0.1 10.0

- Units: s

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
|18|LAND Mode|
|46|RC Override Enable|
|65|GPS Disable|
|81|Disarm|
|90|EKF Source Set|
|100|KillIMU1|
|101|KillIMU2|
|103|EKF lane switch attempt|
|104|EKF yaw reset|
|110|KillIMU3|
|111|Loweheiser starter|
|112|SwitchExternalAHRS|
|153|ArmDisarm|
|164|Pause Stream Logging|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|171|Calibrate Compasses|
|172|Battery MPPT Enable|
|174|Camera Image Tracking|
|175|Camera Lens|
|177|Mount LRF enable|
|185|Mount Roll/Pitch Lock|
|218|Loweheiser throttle|
|300|Scripting1|
|301|Scripting2|
|302|Scripting3|
|303|Scripting4|
|304|Scripting5|
|305|Scripting6|
|306|Scripting7|
|307|Scripting8|
|308|Scripting9|
|309|Scripting10|
|310|Scripting11|
|311|Scripting12|
|312|Scripting13|
|313|Scripting14|
|314|Scripting15|
|315|Scripting16|
|316|Stop-Restart Scripting|

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
|7|DroneCAN|

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

- Range: -1 127

## RPM1_ESC_MASK: Bitmask of ESC telemetry channels to average

*Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry. RPM telemetry of the selected channels will be averaged

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

## RPM1_ESC_INDEX: ESC Telemetry Index to write RPM to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## RPM1_DC_ID: DroneCAN Sensor ID

*Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend

- Range: -1 10

- Increment: 1

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
|7|DroneCAN|

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

- Range: -1 127

## RPM2_ESC_MASK: Bitmask of ESC telemetry channels to average

*Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry. RPM telemetry of the selected channels will be averaged

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

## RPM2_ESC_INDEX: ESC Telemetry Index to write RPM to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## RPM2_DC_ID: DroneCAN Sensor ID

*Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend

- Range: -1 10

- Increment: 1

# RPM3 Parameters

## RPM3_TYPE: RPM type

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
|7|DroneCAN|

## RPM3_SCALING: RPM scaling

Scaling factor between sensor reading and RPM.

- Increment: 0.001

## RPM3_MAX: Maximum RPM

Maximum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM3_MIN: Minimum RPM

Minimum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM3_MIN_QUAL: Minimum Quality

*Note: This parameter is for advanced users*

Minimum data quality to be used

- Increment: 0.1

## RPM3_PIN: Input pin number

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

- Range: -1 127

## RPM3_ESC_MASK: Bitmask of ESC telemetry channels to average

*Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry. RPM telemetry of the selected channels will be averaged

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

## RPM3_ESC_INDEX: ESC Telemetry Index to write RPM to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## RPM3_DC_ID: DroneCAN Sensor ID

*Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend

- Range: -1 10

- Increment: 1

# RPM4 Parameters

## RPM4_TYPE: RPM type

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
|7|DroneCAN|

## RPM4_SCALING: RPM scaling

Scaling factor between sensor reading and RPM.

- Increment: 0.001

## RPM4_MAX: Maximum RPM

Maximum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM4_MIN: Minimum RPM

Minimum RPM to report. Only used on type = GPIO.

- Increment: 1

## RPM4_MIN_QUAL: Minimum Quality

*Note: This parameter is for advanced users*

Minimum data quality to be used

- Increment: 0.1

## RPM4_PIN: Input pin number

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

- Range: -1 127

## RPM4_ESC_MASK: Bitmask of ESC telemetry channels to average

*Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry. RPM telemetry of the selected channels will be averaged

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16

## RPM4_ESC_INDEX: ESC Telemetry Index to write RPM to

*Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to. Use 0 to disable.

- Range: 0 10

- Increment: 1

## RPM4_DC_ID: DroneCAN Sensor ID

*Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend

- Range: -1 10

- Increment: 1

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

Pin used to read the RSSI voltage or PWM value. Analog Airspeed ports can be used for Analog inputs (some autopilots provide others also), Non-IOMCU Servo/MotorOutputs can be used for PWM input when configured as "GPIOs". Values for some autopilots are given as examples. Search wiki for "Analog pins" for analog pin or "GPIOs", if PWM input type, to determine pin number.

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

- Range: -1 127

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

- Range: 50 400

- RebootRequired: True

- Units: Hz

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

- Bitmask: 0: No Scripts to run message if all scripts have stopped, 1: Runtime messages for memory usage and execution time, 2: Suppress logging scripts to dataflash, 3: log runtime memory usage and execution time, 4: Disable pre-arm check, 5: Save CRC of current scripts to loaded and running checksum parameters enabling pre-arm, 6: Disable heap expansion on allocation failure

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

## SCR_LD_CHECKSUM: Loaded script checksum

*Note: This parameter is for advanced users*

Required XOR of CRC32 checksum of loaded scripts, vehicle will not arm with incorrect scripts loaded, -1 disables

## SCR_RUN_CHECKSUM: Running script checksum

*Note: This parameter is for advanced users*

Required XOR of CRC32 checksum of running scripts, vehicle will not arm with incorrect scripts running, -1 disables

## SCR_THD_PRIORITY: Scripting thread priority

*Note: This parameter is for advanced users*

This sets the priority of the scripting thread. This is normally set to a low priority to prevent scripts from interfering with other parts of the system. Advanced users can change this priority if scripting needs to be prioritised for realtime applications. WARNING: changing this parameter can impact the stability of your flight controller. The scipting thread priority in this parameter is chosen based on a set of system level priorities for other subsystems. It is strongly recommended that you use the lowest priority that is sufficient for your application. Note that all scripts run at the same priority, so if you raise this priority you must carefully audit all lua scripts for behaviour that does not interfere with the operation of the system.

|Value|Meaning|
|:---:|:---:|
|0|Normal|
|1|IO Priority|
|2|Storage Priority|
|3|UART Priority|
|4|I2C Priority|
|5|SPI Priority|
|6|Timer Priority|
|7|Main Priority|
|8|Boost Priority|

- RebootRequired: True

## SCR_SDEV_EN: Scripting serial device enable

*Note: This parameter is for advanced users*

Enable scripting serial devices

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- RebootRequired: True

## SCR_SDEV1_PROTO: Serial protocol of scripting serial device

*Note: This parameter is for advanced users*

Serial protocol of scripting serial device

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SCR_SDEV2_PROTO: Serial protocol of scripting serial device

*Note: This parameter is for advanced users*

Serial protocol of scripting serial device

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SCR_SDEV3_PROTO: Serial protocol of scripting serial device

*Note: This parameter is for advanced users*

Serial protocol of scripting serial device

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

- Range: 1 20000000

## SERIAL0_PROTOCOL: Console protocol selection

Control what protocol to use on the console.

|Value|Meaning|
|:---:|:---:|
|1|MAVLink1|
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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

- Range: 1 20000000

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL2_BAUD: Telemetry 2 Baud Rate

The baud rate of the Telem2 port. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL3_BAUD: Serial 3 (GPS) Baud Rate

The baud rate used for the Serial 3 (GPS). Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL4_BAUD: Serial 4 Baud Rate

The baud rate used for Serial4. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL5_BAUD: Serial 5 Baud Rate

The baud rate used for Serial5. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL6_BAUD: Serial 6 Baud Rate

The baud rate used for Serial6. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## SERIAL1_OPTIONS: Telem1 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

- RebootRequired: True

## SERIAL2_OPTIONS: Telem2 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

- RebootRequired: True

## SERIAL3_OPTIONS: Serial3 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

- RebootRequired: True

## SERIAL4_OPTIONS: Serial4 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

- RebootRequired: True

## SERIAL5_OPTIONS: Serial5 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

- RebootRequired: True

## SERIAL6_OPTIONS: Serial6 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

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

This sets one side of pass-through between two serial ports. Once both sides are set then all data received on either port will be passed to the other port. This parameter is normally reset to -1 on reboot, disabling passthrough. If SERIAL_PASSTIMO is set to -1 then it is not reset on reboot.

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

This sets a timeout for serial pass-through in seconds. When the pass-through is enabled by setting the SERIAL_PASS1 and SERIAL_PASS2 parameters then it remains in effect until no data comes from the first port for SERIAL_PASSTIMO seconds. This allows the port to revent to its normal usage (such as MAVLink connection to a GCS) when it is no longer needed. A value of 0 means no timeout. A value of -1 means no timeout and the SERIAL_PASS2 parameter is not reset on reboot.

- Range: -1 120

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL7_BAUD: Serial 7 Baud Rate

The baud rate used for Serial7. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## SERIAL7_OPTIONS: Serial7 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL8_BAUD: Serial 8 Baud Rate

The baud rate used for Serial8. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## SERIAL8_OPTIONS: Serial8 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

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
|45|DDS XRCE|
|46|IMUDATA|
|48|PPP|
|49|i-BUS Telemetry|
|50|IOMCU|

## SERIAL9_BAUD: Serial 9 Baud Rate

The baud rate used for Serial8. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.

- Range: 1 20000000

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
|1500|1.5MBaud|
|2000|2MBaud|
|12500000|12.5MBaud|

## SERIAL9_OPTIONS: Serial9 options

*Note: This parameter is for advanced users*

Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.  NOTE that two bits have moved from this parameter into MAVn_OPTIONS!

- Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7), 11: DisableFIFO, 12: Ignore Streamrate (moved to MAVn_OPTIONS >4.7)

- RebootRequired: True

# SERVO Parameters

## SERVO_RATE: Servo default output rate

*Note: This parameter is for advanced users*

Default output rate in Hz for all PWM outputs.

- Range: 25 400

- Units: Hz

## SERVO_DSHOT_RATE: Servo DShot output rate

*Note: This parameter is for advanced users*

DShot output rate for all outputs as a multiple of the loop rate. 0 sets the output rate to be fixed at 1Khz for low loop rates. This value should never be set below 500Hz.

|Value|Meaning|
|:---:|:---:|
|0|1Khz|
|1|loop-rate|
|2|double loop-rate|
|3|triple loop-rate|
|4|quadruple loop rate|

## SERVO_DSHOT_ESC: Servo DShot ESC type

*Note: This parameter is for advanced users*

DShot ESC type for all outputs. The ESC type affects the range of DShot commands available and the bit widths used. None means that no dshot commands will be executed. Some ESC types support Extended DShot Telemetry (EDT) which allows telemetry other than RPM data to be returned when using bi-directional dshot. If you enable EDT you must install EDT capable firmware for correct operation.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|BLHeli32/Kiss/AM32|
|2|BLHeli_S/BlueJay|
|3|BLHeli32/AM32/Kiss+EDT|
|4|BLHeli_S/BlueJay+EDT|

## SERVO_GPIO_MASK: Servo GPIO mask

*Note: This parameter is for advanced users*

Bitmask of outputs which will be available as GPIOs. Any output with either the function set to -1 or with the corresponding bit set in this mask will be available for use as a GPIO pin

- Bitmask: 0:Servo 1, 1:Servo 2, 2:Servo 3, 3:Servo 4, 4:Servo 5, 5:Servo 6, 6:Servo 7, 7:Servo 8, 8:Servo 9, 9:Servo 10, 10:Servo 11, 11:Servo 12, 12:Servo 13, 13:Servo 14, 14:Servo 15, 15:Servo 16, 16:Servo 17, 17:Servo 18, 18:Servo 19, 19:Servo 20, 20:Servo 21, 21:Servo 22, 22:Servo 23, 23:Servo 24, 24:Servo 25, 25:Servo 26, 26:Servo 27, 27:Servo 28, 28:Servo 29, 29:Servo 30, 30:Servo 31, 31:Servo 32

- RebootRequired: True

## SERVO_RC_FS_MSK: Servo RC Failsafe Mask

*Note: This parameter is for advanced users*

Bitmask of scaled passthru output channels which will be set to their trim value during rc failsafe instead of holding their last position before failsafe.

- Bitmask: 0:RCIN1Scaled, 1:RCIN2Scaled, 2:RCIN3Scaled, 3:RCIN4Scaled, 4:RCIN5Scaled, 5:RCIN6Scaled, 6:RCIN7Scaled, 7:RCIN8Scaled, 8:RCIN9Scaled, 9:RCIN10Scaled, 10:RCIN11Scaled, 11:SRCIN12Scaled, 12:RCIN13Scaled, 13:RCIN14Scaled, 14:RCIN15Scaled, 15:RCIN16Scaled

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
|180|CameraZoom|

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

Mask of channels which are dynamically reversible. This is used to configure ESCs in '3D' mode, allowing for the motor to spin in either direction. Note that setting an ESC as reversible with this option on AM32 will result in the forward direction of the ESC changing. You can combine with parameter with the SERVO_BLH_RVMASK parameter to maintain the same direction when the ESC is in 3D mode as it has in unidirectional (non-3D) mode.

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## SERVO_BLH_BDMASK: BLHeli bitmask of bi-directional dshot channels

*Note: This parameter is for advanced users*

Mask of channels which support bi-directional dshot telemetry. This is used for ESCs which have firmware that supports bi-directional dshot allowing fast rpm telemetry values to be returned for the harmonic notch.

- Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32

- RebootRequired: True

## SERVO_BLH_RVMASK: BLHeli bitmask of reversed channels

*Note: This parameter is for advanced users*

Mask of channels which are reversed. This is used to configure ESCs to reverse motor direction. Note that when combined with SERVO_BLH_3DMASK this will change what direction is considered to be forward.

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

## SERVO_VOLZ_RANGE: Range of travel

Range to map between 1000 and 2000 PWM. Default value of 200 gives full +-100 deg range of extended position command. This results in 0.2 deg movement per US change in PWM. If the full range is not needed it can be reduced to increase resolution. 40 deg range gives 0.04 deg movement per US change in PWM, this is higher resolution than possible with the VOLZ protocol so further reduction in range will not improve resolution. Reduced range does allow PWMs outside the 1000 to 2000 range, with 40 deg range 750 PWM results in a angle of -30 deg, 2250 would be +30 deg. This is still limited by the 200 deg maximum range of the actuator.

- Units: deg

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

Seconds since January 1st 2016 (Unix epoch+1451606400) since statistics reset (set to 0 to reset statistics, other set values will be ignored)

- Units: s

- ReadOnly: True

## STAT_FLTCNT: Total Flight Count

Total number of flights

- ReadOnly: True

## STAT_DISTFLWN: Total Distance Flown

Estimate of total distance flown since statistics reset

- Units: m

- ReadOnly: True

# TEMP Parameters

## TEMP_LOG: Logging

Enables temperature sensor logging

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Log all instances|
|2|Log only instances with sensor source set to None|

# TEMP1 Parameters

## TEMP1_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP1_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP1_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP1_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP1_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP1_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP1_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP1_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP1_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP1_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP1_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP1_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP2 Parameters

## TEMP2_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP2_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP2_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP2_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP2_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP2_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP2_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP2_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP2_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP2_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP2_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP2_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP3 Parameters

## TEMP3_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP3_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP3_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP3_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP3_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP3_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP3_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP3_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP3_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP3_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP3_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP3_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP4 Parameters

## TEMP4_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP4_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP4_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP4_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP4_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP4_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP4_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP4_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP4_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP4_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP4_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP4_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP5 Parameters

## TEMP5_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP5_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP5_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP5_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP5_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP5_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP5_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP5_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP5_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP5_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP5_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP5_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP6 Parameters

## TEMP6_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP6_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP6_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP6_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP6_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP6_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP6_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP6_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP6_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP6_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP6_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP6_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP7 Parameters

## TEMP7_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP7_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP7_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP7_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP7_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP7_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP7_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP7_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP7_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP7_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP7_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP7_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP8 Parameters

## TEMP8_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP8_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP8_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP8_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP8_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP8_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP8_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP8_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP8_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP8_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP8_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP8_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP9 Parameters

## TEMP9_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

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
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP9_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP9_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP9_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP9_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP9_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP9_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP9_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP9_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP9_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP9_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP9_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP10 Parameters

## TEMP10_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

- RebootRequired: True

## TEMP10_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP10_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP10_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP10_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP10_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP10_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP10_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP10_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP10_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP10_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP10_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP10_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP10_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP10_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP11 Parameters

## TEMP11_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

- RebootRequired: True

## TEMP11_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP11_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP11_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP11_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP11_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP11_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP11_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP11_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP11_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP11_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP11_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP11_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP11_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP11_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP12 Parameters

## TEMP12_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

- RebootRequired: True

## TEMP12_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP12_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP12_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP12_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP12_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP12_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP12_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP12_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP12_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP12_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP12_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP12_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP12_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP12_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP13 Parameters

## TEMP13_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

- RebootRequired: True

## TEMP13_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP13_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP13_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP13_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP13_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP13_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP13_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP13_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP13_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP13_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP13_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP13_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP13_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP13_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP14 Parameters

## TEMP14_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

- RebootRequired: True

## TEMP14_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP14_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP14_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP14_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP14_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP14_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP14_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP14_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP14_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP14_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP14_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP14_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP14_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP14_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

# TEMP15 Parameters

## TEMP15_TYPE: Temperature Sensor Type

Enables temperature sensors

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|TSYS01|
|2|MCP9600|
|3|MAX31865 2 or 4 wire|
|4|TSYS03|
|5|Analog|
|6|DroneCAN|
|7|MLX90614|
|8|SHT3x|
|9|MAX31865 3 wire|

- RebootRequired: True

## TEMP15_BUS: Temperature sensor bus

*Note: This parameter is for advanced users*

Temperature sensor bus number, typically used to select from multiple I2C buses

- Range: 0 3

- RebootRequired: True

## TEMP15_ADDR: Temperature sensor address

*Note: This parameter is for advanced users*

Temperature sensor address, typically used for I2C address

- Range: 0 127

- RebootRequired: True

## TEMP15_SRC: Sensor Source

Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|ESC|
|2|Motor|
|3|Battery Index|
|4|Battery ID/SerialNumber|
|5|CAN based Pitot tube|
|6|DroneCAN-out on AP_Periph|
|7|Servo motor|
|8|Servo PCB|

## TEMP15_SRC_ID: Sensor Source Identification

Sensor Source Identification is used to replace a specific instance of a system component's temperature report with the temp sensor's. Examples: TEMP_SRC = 1 (ESC), TEMP_SRC_ID = 1 will set the temp of ESC1. TEMP_SRC = 3 (BatteryIndex),TEMP_SRC_ID = 2 will set the temp of BATT2. TEMP_SRC = 4 (BatteryId/SerialNum),TEMP_SRC_ID=42 will set the temp of all batteries that have param BATTn_SERIAL = 42.

## TEMP15_PIN: Temperature sensor analog voltage sensing pin

Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".

|Value|Meaning|
|:---:|:---:|
|-1|Disabled|
|2|Pixhawk/Pixracer/Navio2/Pixhawk2_PM1|
|5|Navigator|
|13|Pixhawk2_PM2/CubeOrange_PM2|
|14|CubeOrange|
|16|Durandal|
|100|PX4-v1|

- Range: -1 127

## TEMP15_A0: Temperature sensor analog 0th polynomial coefficient

a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP15_A1: Temperature sensor analog 1st polynomial coefficient

a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP15_A2: Temperature sensor analog 2nd polynomial coefficient

a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP15_A3: Temperature sensor analog 3rd polynomial coefficient

a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP15_A4: Temperature sensor analog 4th polynomial coefficient

a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP15_A5: Temperature sensor analog 5th polynomial coefficient

a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5

## TEMP15_MSG_ID: Temperature sensor DroneCAN message ID

Sets the message device ID this backend listens for

- Range: 0 65535

## TEMP15_RTD_NOM: Nominal RTD resistance

Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.

## TEMP15_RTD_REF: RTD reference resistance

Reference resistance used to calculate temperature, in ohms

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

## VISO_QUAL_MIN: Visual odometry minimum quality

*Note: This parameter is for advanced users*

Visual odometry will only be sent to EKF if over this value. -1 to always send (even bad values), 0 to send if good or unknown

- Units: %

- Range: -1 100

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
|6|1G3 Band A|
|7|1G3 Band B|
|8|Band X|
|9|3G3 Band A|
|10|3G3 Band B|

## VTX_FREQ: Video Transmitter Frequency

Video Transmitter Frequency. The frequency is derived from the setting of BAND and CHANNEL

- ReadOnly: True

- Range: 1000 6000

## VTX_OPTIONS: Video Transmitter Options

*Note: This parameter is for advanced users*

Video Transmitter Options. Pitmode puts the VTX in a low power state. Unlocked enables certain restricted frequencies and power levels. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels. One stop-bit may be required for VTXs that erroneously mimic iNav behaviour.

- Bitmask: 0:Pitmode,1:Pitmode until armed,2:Pitmode when disarmed,3:Unlocked,4:Add leading zero byte to requests,5:Use 1 stop-bit in SmartAudio,6:Ignore CRC in SmartAudio,7:Ignore status updates in CRSF and blindly set VTX options

## VTX_MAX_POWER: Video Transmitter Max Power Level

Video Transmitter Maximum Power Level. Different VTXs support different power levels, this prevents the power aux switch from requesting too high a power level. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting.

- Range: 25 1000