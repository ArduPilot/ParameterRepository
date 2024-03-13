
This is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your autopilot to control vehicle behaviour. This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code. Some parameters may only be available for developers, and are enabled at compile-time.

# ArduPlane Parameters

## FORMAT_VERSION: Eeprom format version number

*Note: This parameter is for advanced users*

This value is incremented when changes are made to the eeprom format

## SYSID_THISMAV: MAVLink system ID of this vehicle

*Note: This parameter is for advanced users*

Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network

- Range: 1 255

## SYSID_MYGCS: Ground station MAVLink system ID

*Note: This parameter is for advanced users*

The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.

- Range: 1 255

## AUTOTUNE_LEVEL: Autotune level

Level of aggressiveness of pitch and roll PID gains. Lower values result in a 'softer' tune. Level 6 recommended for most planes. A value of 0 means to keep the current values of RMAX and TCONST for the controllers, tuning only the PID values

- Range: 0 10

- Increment: 1

## TELEM_DELAY: Telemetry startup delay

The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up

- Units: s

- Range: 0 30

- Increment: 1

## GCS_PID_MASK: GCS PID tuning mask

*Note: This parameter is for advanced users*

bitmask of PIDs to send MAVLink PID_TUNING messages for

- Bitmask: 0:Roll,1:Pitch,2:Yaw,3:Steering,4:Landing

## KFF_RDDRMIX: Rudder Mix

Amount of rudder to add during aileron movement. Increase if nose initially yaws away from roll. Reduces adverse yaw.

- Range: 0 1

- Increment: 0.01

## KFF_THR2PTCH: Throttle to Pitch Mix

*Note: This parameter is for advanced users*

Pitch up to add in proportion to throttle. 100% throttle will add this number of degrees to the pitch target.

- Range: 0 5

- Increment: 0.01

## STAB_PITCH_DOWN: Low throttle pitch down trim

*Note: This parameter is for advanced users*

Degrees of down pitch added when throttle is below TRIM_THROTTLE in FBWA and AUTOTUNE modes. Scales linearly so full value is added when THR_MIN is reached. Helps to keep airspeed higher in glides or landing approaches and prevents accidental stalls. 2 degrees recommended for most planes.

- Range: 0 15

- Increment: 0.1

- Units: deg

## GLIDE_SLOPE_MIN: Glide slope minimum

*Note: This parameter is for advanced users*

This controls the minimum altitude change for a waypoint before a glide slope will be used instead of an immediate altitude change. The default value is 15 meters, which helps to smooth out waypoint missions where small altitude changes happen near waypoints. If you don't want glide slopes to be used in missions then you can set this to zero, which will disable glide slope calculations. Otherwise you can set it to a minimum number of meters of altitude error to the destination waypoint before a glide slope will be used to change altitude.

- Range: 0 1000

- Increment: 1

- Units: m

## GLIDE_SLOPE_THR: Glide slope threshold

*Note: This parameter is for advanced users*

This controls the height above the glide slope the plane may be before rebuilding a glide slope. This is useful for smoothing out an autotakeoff

- Range: 0 100

- Increment: 1

- Units: m

## STICK_MIXING: Stick Mixing

*Note: This parameter is for advanced users*

When enabled, this adds user stick input to the control surfaces in auto modes, allowing the user to have some degree of flight control without changing modes.  There are two types of stick mixing available. If you set STICK_MIXING to 1 then it will use "fly by wire" mixing, which controls the roll and pitch in the same way that the FBWA mode does. This is the safest option if you usually fly ArduPlane in FBWA or FBWB mode. If you set STICK_MIXING to 2 then it will enable direct mixing mode, which is what the STABILIZE mode uses. That will allow for much more extreme maneuvers while in AUTO mode. If you set STICK_MIXING to 3 then it will apply to the yaw while in quadplane modes only, such as while doing an automatic VTOL takeoff or landing.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|FBWMixing|
|2|DirectMixing|
|3|VTOL Yaw only|

## TKOFF_THR_MINSPD: Takeoff throttle min speed

Minimum GPS ground speed in m/s used by the speed check that un-suppresses throttle in auto-takeoff. This can be be used for catapult launches where you want the motor to engage only after the plane leaves the catapult, but it is preferable to use the TKOFF_THR_MINACC and TKOFF_THR_DELAY parameters for catapult launches due to the errors associated with GPS measurements. For hand launches with a pusher prop it is strongly advised that this parameter be set to a value no less than 4 m/s to provide additional protection against premature motor start. Note that the GPS velocity will lag the real velocity by about 0.5 seconds. The ground speed check is delayed by the TKOFF_THR_DELAY parameter.

- Units: m/s

- Range: 0 30

- Increment: 0.1

## TKOFF_THR_MINACC: Takeoff throttle min acceleration

Minimum forward acceleration in m/s/s before arming the ground speed check in auto-takeoff. This is meant to be used for hand launches. Setting this value to 0 disables the acceleration test which means the ground speed check will always be armed which could allow GPS velocity jumps to start the engine. For hand launches and bungee launches this should be set to around 15. Also see TKOFF_ACCEL_CNT paramter for control of full "shake to arm".

- Units: m/s/s

- Range: 0 30

- Increment: 0.1

## TKOFF_THR_DELAY: Takeoff throttle delay

This parameter sets the time delay (in 1/10ths of a second) that the ground speed check is delayed after the forward acceleration check controlled by TKOFF_THR_MINACC has passed. For hand launches with pusher propellers it is essential that this is set to a value of no less than 2 (0.2 seconds) to ensure that the aircraft is safely clear of the throwers arm before the motor can start. For bungee launches a larger value can be used (such as 30) to give time for the bungee to release from the aircraft before the motor is started.

- Units: ds

- Range: 0 127

- Increment: 1

## TKOFF_TDRAG_ELEV: Takeoff tail dragger elevator

This parameter sets the amount of elevator to apply during the initial stage of a takeoff. It is used to hold the tail wheel of a taildragger on the ground during the initial takeoff stage to give maximum steering. This option should be combined with the TKOFF_TDRAG_SPD1 option and the GROUND_STEER_ALT option along with tuning of the ground steering controller. A value of zero means to bypass the initial "tail hold" stage of takeoff. Set to zero for hand and catapult launch. For tail-draggers you should normally set this to 100, meaning full up elevator during the initial stage of takeoff. For most tricycle undercarriage aircraft a value of zero will work well, but for some tricycle aircraft a small negative value (say around -20 to -30) will apply down elevator which will hold the nose wheel firmly on the ground during initial acceleration. Only use a negative value if you find that the nosewheel doesn't grip well during takeoff. Too much down elevator on a tricycle undercarriage may cause instability in steering as the plane pivots around the nosewheel. Add down elevator 10 percent at a time.

- Units: %

- Range: -100 100

- Increment: 1

## TKOFF_TDRAG_SPD1: Takeoff tail dragger speed1

This parameter sets the airspeed at which to stop holding the tail down and transition to rudder control of steering on the ground. When TKOFF_TDRAG_SPD1 is reached the pitch of the aircraft will be held level until TKOFF_ROTATE_SPD is reached, at which point the takeoff pitch specified in the mission will be used to "rotate" the pitch for takeoff climb. Set TKOFF_TDRAG_SPD1 to zero to go straight to rotation. This should be set to zero for hand launch and catapult launch. It should also be set to zero for tricycle undercarriages unless you are using the method above to genetly hold the nose wheel down. For tail dragger aircraft it should be set just below the stall speed.

- Units: m/s

- Range: 0 30

- Increment: 0.1

## TKOFF_ROTATE_SPD: Takeoff rotate speed

This parameter sets the airspeed at which the aircraft will "rotate", setting climb pitch specified in the mission. If TKOFF_ROTATE_SPD is zero then the climb pitch will be used as soon as takeoff is started. For hand launch and catapult launches a TKOFF_ROTATE_SPD of zero should be set. For all ground launches TKOFF_ROTATE_SPD should be set above the stall speed, usually by about 10 to 30 percent

- Units: m/s

- Range: 0 30

- Increment: 0.1

## TKOFF_THR_SLEW: Takeoff throttle slew rate

This parameter sets the slew rate for the throttle during auto takeoff. When this is zero the THR_SLEWRATE parameter is used during takeoff. For rolling takeoffs it can be a good idea to set a lower slewrate for takeoff to give a slower acceleration which can improve ground steering control. The value is a percentage throttle change per second, so a value of 20 means to advance the throttle over 5 seconds on takeoff. Values below 20 are not recommended as they may cause the plane to try to climb out with too little throttle. A value of -1 means no limit on slew rate in takeoff.

- Units: %/s

- Range: -1 127

- Increment: 1

## TKOFF_PLIM_SEC: Takeoff pitch limit reduction

*Note: This parameter is for advanced users*

This parameter reduces the pitch minimum limit of an auto-takeoff just a few seconds before it reaches the target altitude. This reduces overshoot by allowing the flight controller to start leveling off a few seconds before reaching the target height. When set to zero, the mission pitch min is enforced all the way to and through the target altitude, otherwise the pitch min slowly reduces to zero in the final segment. This is the pitch_min, not the demand. The flight controller should still be commanding to gain altitude to finish the takeoff but with this param it is not forcing it higher than it wants to be.

- Units: s

- Range: 0 10

- Increment: 0.5

## TKOFF_FLAP_PCNT: Takeoff flap percentage

*Note: This parameter is for advanced users*

The amount of flaps (as a percentage) to apply in automatic takeoff

- Range: 0 100

- Units: %

- Increment: 1

## LEVEL_ROLL_LIMIT: Level flight roll limit

This controls the maximum bank angle in degrees during flight modes where level flight is desired, such as in the final stages of landing, and during auto takeoff. This should be a small angle (such as 5 degrees) to prevent a wing hitting the runway during takeoff or landing. Setting this to zero will completely disable heading hold on auto takeoff and final landing approach.

- Units: deg

- Range: 0 45

- Increment: 1

## USE_REV_THRUST: Bitmask for when to allow negative reverse thrust

*Note: This parameter is for advanced users*

This controls when to use reverse thrust. If set to zero then reverse thrust is never used. If set to a non-zero value then the bits correspond to flight stages where reverse thrust may be used. The most commonly used value for USE_REV_THRUST is 2, which means AUTO_LAND only. That enables reverse thrust in the landing stage of AUTO mode. Another common choice is 1, which means to use reverse thrust in all auto flight stages. Reverse thrust is always used in MANUAL mode if enabled with THR_MIN < 0. In non-autothrottle controlled modes, if reverse thrust is not used, then THR_MIN is effectively set to 0 for that mode.

|Value|Meaning|
|:---:|:---:|
|0|MANUAL ONLY|
|1|AutoAlways|
|2|AutoLanding|

- Bitmask: 0:AUTO_ALWAYS,1:AUTO_LAND,2:AUTO_LOITER_TO_ALT,3:AUTO_LOITER_ALL,4:AUTO_WAYPOINTS,5:LOITER,6:RTL,7:CIRCLE,8:CRUISE,9:FBWB,10:GUIDED,11:AUTO_LANDING_PATTERN,12:FBWA,13:ACRO,14:STABILIZE,15:THERMAL

## ALT_OFFSET: Altitude offset

*Note: This parameter is for advanced users*

This is added to the target altitude in automatic flight. It can be used to add a global altitude offset to a mission

- Units: m

- Range: -32767 32767

- Increment: 1

## WP_RADIUS: Waypoint Radius

Defines the maximum distance from a waypoint that when crossed indicates the waypoint may be complete. To avoid the aircraft looping around the waypoint in case it misses by more than the WP_RADIUS an additional check is made to see if the aircraft has crossed a "finish line" passing through the waypoint and perpendicular to the flight path from the previous waypoint. If that finish line is crossed then the waypoint is considered complete. Note that the navigation controller may decide to turn later than WP_RADIUS before a waypoint, based on how sharp the turn is and the speed of the aircraft. It is safe to set WP_RADIUS much larger than the usual turn radius of your aircraft and the navigation controller will work out when to turn. If you set WP_RADIUS too small then you will tend to overshoot the turns.

- Units: m

- Range: 1 32767

- Increment: 1

## WP_MAX_RADIUS: Waypoint Maximum Radius

Sets the maximum distance to a waypoint for the waypoint to be considered complete. This overrides the "cross the finish line" logic that is normally used to consider a waypoint complete. For normal AUTO behaviour this parameter should be set to zero. Using a non-zero value is only recommended when it is critical that the aircraft does approach within the given radius, and should loop around until it has done so. This can cause the aircraft to loop forever if its turn radius is greater than the maximum radius set.

- Units: m

- Range: 0 32767

- Increment: 1

## WP_LOITER_RAD: Waypoint Loiter Radius

Defines the distance from the waypoint center, the plane will maintain during a loiter. If you set this value to a negative number then the default loiter direction will be counter-clockwise instead of clockwise.

- Units: m

- Range: -32767 32767

- Increment: 1

## RTL_RADIUS: RTL loiter radius

Defines the radius of the loiter circle when in RTL mode. If this is zero then WP_LOITER_RAD is used. If the radius is negative then a counter-clockwise is used. If positive then a clockwise loiter is used.

- Units: m

- Range: -32767 32767

- Increment: 1

## STALL_PREVENTION: Enable stall prevention

Enables roll limits at low airspeed in roll limiting flight modes. Roll limits based on aerodynamic load factor in turns and scale on ARSPD_FBW_MIN that must be set correctly. Without airspeed sensor, uses synthetic airspeed from wind speed estimate that may both be inaccurate.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ARSPD_FBW_MIN: Minimum Airspeed

Minimum airspeed demanded in automatic throttle modes. Should be set to 20% higher than level flight stall speed.

- Units: m/s

- Range: 5 100

- Increment: 1

## ARSPD_FBW_MAX: Maximum Airspeed

Maximum airspeed demanded in automatic throttle modes. Should be set slightly less than level flight speed at THR_MAX and also at least 50% above ARSPD_FBW_MIN to allow for accurate TECS altitude control.

- Units: m/s

- Range: 5 100

- Increment: 1

## FBWB_ELEV_REV: Fly By Wire elevator reverse

Reverse sense of elevator in FBWB and CRUISE modes. When set to 0 up elevator (pulling back on the stick) means to lower altitude. When set to 1, up elevator means to raise altitude.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## TERRAIN_FOLLOW: Use terrain following

This enables terrain following for CRUISE mode, FBWB mode, RTL and for rally points. To use this option you also need to set TERRAIN_ENABLE to 1, which enables terrain data fetching from the GCS, and you need to have a GCS that supports sending terrain data to the aircraft. When terrain following is enabled then CRUISE and FBWB mode will hold height above terrain rather than height above home. In RTL the return to launch altitude will be considered to be a height above the terrain. Rally point altitudes will be taken as height above the terrain. This option does not affect mission items, which have a per-waypoint flag for whether they are height above home or height above the terrain. To use terrain following missions you need a ground station which can set the waypoint type to be a terrain height waypoint when creating the mission.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

- Bitmask: 0: Enable all modes, 1:FBWB, 2:Cruise, 3:Auto, 4:RTL, 5:Avoid_ADSB, 6:Guided, 7:Loiter, 8:Circle, 9:QRTL, 10:QLand, 11:Qloiter

## TERRAIN_LOOKAHD: Terrain lookahead

This controls how far ahead the terrain following code looks to ensure it stays above upcoming terrain. A value of zero means no lookahead, so the controller will track only the terrain directly below the aircraft. The lookahead will never extend beyond the next waypoint when in AUTO mode.

- Range: 0 10000

- Units: m

## FBWB_CLIMB_RATE: Fly By Wire B altitude change rate

This sets the rate in m/s at which FBWB and CRUISE modes will change its target altitude for full elevator deflection. Note that the actual climb rate of the aircraft can be lower than this, depending on your airspeed and throttle control settings. If you have this parameter set to the default value of 2.0, then holding the elevator at maximum deflection for 10 seconds would change the target altitude by 20 meters.

- Range: 1 10

- Units: m/s

- Increment: 0.1

## THR_MIN: Minimum Throttle

Minimum throttle percentage used in all modes except manual, provided THR_PASS_STAB is not set. Negative values allow reverse thrust if hardware supports it.

- Units: %

- Range: -100 100

- Increment: 1

## THR_MAX: Maximum Throttle

Maximum throttle percentage used in all modes except manual, provided THR_PASS_STAB is not set.

- Units: %

- Range: 0 100

- Increment: 1

## TKOFF_THR_MAX: Maximum Throttle for takeoff

*Note: This parameter is for advanced users*

The maximum throttle setting during automatic takeoff. If this is zero then THR_MAX is used for takeoff as well.

- Units: %

- Range: 0 100

- Increment: 1

## THR_SLEWRATE: Throttle slew rate

Maximum change in throttle percentage per second. Lower limit  based on 1 microsend of servo increase per loop. Divide SCHED_LOOP_RATE by approximately 10 to determine minimum achievable value.

- Units: %/s

- Range: 0 127

- Increment: 1

## FLAP_SLEWRATE: Flap slew rate

*Note: This parameter is for advanced users*

maximum percentage change in flap output per second. A setting of 25 means to not change the flap by more than 25% of the full flap range in one second. A value of 0 means no rate limiting.

- Units: %/s

- Range: 0 100

- Increment: 1

## THR_SUPP_MAN: Throttle suppress manual passthru

*Note: This parameter is for advanced users*

When throttle is suppressed in auto mode it is normally forced to zero. If you enable this option, then while suppressed it will be manual throttle. This is useful on petrol engines to hold the idle throttle manually while waiting for takeoff

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## THR_PASS_STAB: Throttle passthru in stabilize

*Note: This parameter is for advanced users*

If this is set then when in STABILIZE, FBWA or ACRO modes the throttle is a direct passthru from the transmitter. This means the THR_MIN and THR_MAX settings are not used in these modes. This is useful for petrol engines where you setup a throttle cut switch that suppresses the throttle below the normal minimum.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## THR_FAILSAFE: Throttle and RC Failsafe Enable

0 disables the failsafe. 1 enables failsafe on loss of RC input. This is detected either by throttle values below THR_FS_VALUE, loss of receiver valid pulses/data, or by the FS bit in receivers that provide it, like SBUS. A programmable failsafe action will occur and RC inputs, if present, will be ignored. A value of 2 means that the RC inputs won't be used when RC failsafe is detected by any of the above methods, but it won't trigger an RC failsafe action.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|EnabledNoFailsafe|

## THR_FS_VALUE: Throttle Failsafe Value

The PWM level on the throttle input channel below which throttle failsafe triggers. Note that this should be well below the normal minimum for your throttle channel.

- Range: 925 2200

- Increment: 1

## TRIM_THROTTLE: Throttle cruise percentage

Target percentage of throttle to apply for flight in automatic throttle modes and throttle percentage that maintains TRIM_ARSPD_CM. Caution: low battery voltages at the end of flights may require higher throttle to maintain airspeed.

- Units: %

- Range: 0 100

- Increment: 1

## THROTTLE_NUDGE: Throttle nudge enable

When enabled, this uses the throttle input in auto-throttle modes to 'nudge' the throttle or airspeed to higher or lower values. When you have an airspeed sensor the nudge affects the target airspeed, so that throttle inputs above 50% will increase the target airspeed from TRIM_ARSPD_CM up to a maximum of ARSPD_FBW_MAX. When no airspeed sensor is enabled the throttle nudge will push up the target throttle for throttle inputs above 50%.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## FS_SHORT_ACTN: Short failsafe action

The action to take on a short (FS_SHORT_TIMEOUT) failsafe event. A short failsafe event can be triggered either by loss of RC control (see THR_FS_VALUE) or by loss of GCS control (see FS_GCS_ENABL). If in CIRCLE or RTL mode this parameter is ignored. A short failsafe event in stabilization and manual modes will cause a change to CIRCLE mode if FS_SHORT_ACTN is 0 or 1, a change to FBWA mode with zero throttle if FS_SHORT_ACTN is 2, and a change to FBWB mode if FS_SHORT_ACTN is 4. In all other modes (AUTO, GUIDED and LOITER) a short failsafe event will cause no mode change if FS_SHORT_ACTN is set to 0, will cause a change to CIRCLE mode if set to 1, will change to FBWA mode with zero throttle if set to 2, or will change to FBWB if set to 4. Please see the documentation for FS_LONG_ACTN for the behaviour after FS_LONG_TIMEOUT seconds of failsafe.

|Value|Meaning|
|:---:|:---:|
|0|CIRCLE/no change(if already in AUTO|GUIDED|LOITER)|
|1|CIRCLE|
|2|FBWA at zero throttle|
|3|Disable|
|4|FBWB|

## FS_SHORT_TIMEOUT: Short failsafe timeout

The time in seconds that a failsafe condition has to persist before a short failsafe event will occur. This defaults to 1.5 seconds

- Units: s

- Range: 1 100

- Increment: 0.5

## FS_LONG_ACTN: Long failsafe action

The action to take on a long (FS_LONG_TIMEOUT seconds) failsafe event. If the aircraft was in a stabilization or manual mode when failsafe started and a long failsafe occurs then it will change to RTL mode if FS_LONG_ACTN is 0 or 1, and will change to FBWA if FS_LONG_ACTN is set to 2. If the aircraft was in an auto mode (such as AUTO or GUIDED) when the failsafe started then it will continue in the auto mode if FS_LONG_ACTN is set to 0, will change to RTL mode if FS_LONG_ACTN is set to 1 and will change to FBWA mode if FS_LONG_ACTN is set to 2. If FS_LONG_ACTION is set to 3, the parachute will be deployed (make sure the chute is configured and enabled).

|Value|Meaning|
|:---:|:---:|
|0|Continue|
|1|ReturnToLaunch|
|2|Glide|
|3|Deploy Parachute|

## FS_LONG_TIMEOUT: Long failsafe timeout

The time in seconds that a failsafe condition has to persist before a long failsafe event will occur. This defaults to 5 seconds.

- Units: s

- Range: 1 300

- Increment: 0.5

## FS_GCS_ENABL: GCS failsafe enable

Enable ground control station telemetry failsafe. Failsafe will trigger after FS_LONG_TIMEOUT seconds of no MAVLink heartbeat messages. There are three possible enabled settings. Setting FS_GCS_ENABL to 1 means that GCS failsafe will be triggered when the aircraft has not received a MAVLink HEARTBEAT message. Setting FS_GCS_ENABL to 2 means that GCS failsafe will be triggered on either a loss of HEARTBEAT messages, or a RADIO_STATUS message from a MAVLink enabled 3DR radio indicating that the ground station is not receiving status updates from the aircraft, which is indicated by the RADIO_STATUS.remrssi field being zero (this may happen if you have a one way link due to asymmetric noise on the ground station and aircraft radios).Setting FS_GCS_ENABL to 3 means that GCS failsafe will be triggered by Heartbeat(like option one), but only in AUTO mode. WARNING: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station. If this option is enabled on an electric plane then you should enable ARMING_REQUIRED.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Heartbeat|
|2|HeartbeatAndREMRSSI|
|3|HeartbeatAndAUTO|

## FLTMODE_CH: Flightmode channel

*Note: This parameter is for advanced users*

RC Channel to use for flight mode control

- Range: 1 16

- Increment: 1

## FLTMODE1: FlightMode1

Flight mode for switch position 1 (910 to 1230 and above 2049)

|Value|Meaning|
|:---:|:---:|
|0|Manual|
|1|CIRCLE|
|2|STABILIZE|
|3|TRAINING|
|4|ACRO|
|5|FBWA|
|6|FBWB|
|7|CRUISE|
|8|AUTOTUNE|
|10|Auto|
|11|RTL|
|12|Loiter|
|13|TAKEOFF|
|14|AVOID_ADSB|
|15|Guided|
|17|QSTABILIZE|
|18|QHOVER|
|19|QLOITER|
|20|QLAND|
|21|QRTL|
|22|QAUTOTUNE|
|23|QACRO|
|24|THERMAL|
|25|Loiter to QLand|

## FLTMODE2: FlightMode2

Flight mode for switch position 2 (1231 to 1360)

|Value|Meaning|
|:---:|:---:|
|0|Manual|
|1|CIRCLE|
|2|STABILIZE|
|3|TRAINING|
|4|ACRO|
|5|FBWA|
|6|FBWB|
|7|CRUISE|
|8|AUTOTUNE|
|10|Auto|
|11|RTL|
|12|Loiter|
|13|TAKEOFF|
|14|AVOID_ADSB|
|15|Guided|
|17|QSTABILIZE|
|18|QHOVER|
|19|QLOITER|
|20|QLAND|
|21|QRTL|
|22|QAUTOTUNE|
|23|QACRO|
|24|THERMAL|
|25|Loiter to QLand|

## FLTMODE3: FlightMode3

Flight mode for switch position 3 (1361 to 1490)

|Value|Meaning|
|:---:|:---:|
|0|Manual|
|1|CIRCLE|
|2|STABILIZE|
|3|TRAINING|
|4|ACRO|
|5|FBWA|
|6|FBWB|
|7|CRUISE|
|8|AUTOTUNE|
|10|Auto|
|11|RTL|
|12|Loiter|
|13|TAKEOFF|
|14|AVOID_ADSB|
|15|Guided|
|17|QSTABILIZE|
|18|QHOVER|
|19|QLOITER|
|20|QLAND|
|21|QRTL|
|22|QAUTOTUNE|
|23|QACRO|
|24|THERMAL|
|25|Loiter to QLand|

## FLTMODE4: FlightMode4

Flight mode for switch position 4 (1491 to 1620)

|Value|Meaning|
|:---:|:---:|
|0|Manual|
|1|CIRCLE|
|2|STABILIZE|
|3|TRAINING|
|4|ACRO|
|5|FBWA|
|6|FBWB|
|7|CRUISE|
|8|AUTOTUNE|
|10|Auto|
|11|RTL|
|12|Loiter|
|13|TAKEOFF|
|14|AVOID_ADSB|
|15|Guided|
|17|QSTABILIZE|
|18|QHOVER|
|19|QLOITER|
|20|QLAND|
|21|QRTL|
|22|QAUTOTUNE|
|23|QACRO|
|24|THERMAL|
|25|Loiter to QLand|

## FLTMODE5: FlightMode5

Flight mode for switch position 5 (1621 to 1749)

|Value|Meaning|
|:---:|:---:|
|0|Manual|
|1|CIRCLE|
|2|STABILIZE|
|3|TRAINING|
|4|ACRO|
|5|FBWA|
|6|FBWB|
|7|CRUISE|
|8|AUTOTUNE|
|10|Auto|
|11|RTL|
|12|Loiter|
|13|TAKEOFF|
|14|AVOID_ADSB|
|15|Guided|
|17|QSTABILIZE|
|18|QHOVER|
|19|QLOITER|
|20|QLAND|
|21|QRTL|
|22|QAUTOTUNE|
|23|QACRO|
|24|THERMAL|
|25|Loiter to QLand|

## FLTMODE6: FlightMode6

Flight mode for switch position 6 (1750 to 2049)

|Value|Meaning|
|:---:|:---:|
|0|Manual|
|1|CIRCLE|
|2|STABILIZE|
|3|TRAINING|
|4|ACRO|
|5|FBWA|
|6|FBWB|
|7|CRUISE|
|8|AUTOTUNE|
|10|Auto|
|11|RTL|
|12|Loiter|
|13|TAKEOFF|
|14|AVOID_ADSB|
|15|Guided|
|17|QSTABILIZE|
|18|QHOVER|
|19|QLOITER|
|20|QLAND|
|21|QRTL|
|22|QAUTOTUNE|
|23|QACRO|
|24|THERMAL|
|25|Loiter to QLand|

## INITIAL_MODE: Initial flight mode

*Note: This parameter is for advanced users*

This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver.

|Value|Meaning|
|:---:|:---:|
|0|Manual|
|1|CIRCLE|
|2|STABILIZE|
|3|TRAINING|
|4|ACRO|
|5|FBWA|
|6|FBWB|
|7|CRUISE|
|8|AUTOTUNE|
|10|Auto|
|11|RTL|
|12|Loiter|
|13|TAKEOFF|
|14|AVOID_ADSB|
|15|Guided|
|17|QSTABILIZE|
|18|QHOVER|
|19|QLOITER|
|20|QLAND|
|21|QRTL|
|22|QAUTOTUNE|
|23|QACRO|
|24|THERMAL|
|25|Loiter to QLand|

## LIM_ROLL_CD: Maximum Bank Angle

Maximum bank angle commanded in modes with stabilized limits. Increase this value for sharper turns, but decrease to prevent accelerated stalls.

- Units: cdeg

- Range: 0 9000

- Increment: 10

## LIM_PITCH_MAX: Maximum Pitch Angle

Maximum pitch up angle commanded in modes with stabilized limits.

- Units: cdeg

- Range: 0 9000

- Increment: 10

## LIM_PITCH_MIN: Minimum Pitch Angle

Maximum pitch down angle commanded in modes with stabilized limits

- Units: cdeg

- Range: -9000 0

- Increment: 10

## ACRO_ROLL_RATE: ACRO mode roll rate

The maximum roll rate at full stick deflection in ACRO mode

- Units: deg/s

- Range: 10 500

- Increment: 1

## ACRO_PITCH_RATE: ACRO mode pitch rate

The maximum pitch rate at full stick deflection in ACRO mode

- Units: deg/s

- Range: 10 500

- Increment: 1

## ACRO_YAW_RATE: ACRO mode yaw rate

The maximum yaw rate at full stick deflection in ACRO mode. If this is zero then rudder is directly controlled by rudder stick input. This option is only available if you also set YAW_RATE_ENABLE to 1.

- Units: deg/s

- Range: 0 500

- Increment: 1

## ACRO_LOCKING: ACRO mode attitude locking

Enable attitude locking when sticks are released

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## GROUND_STEER_ALT: Ground steer altitude

Altitude at which to use the ground steering controller on the rudder. If non-zero then the STEER2SRV controller will be used to control the rudder for altitudes within this limit of the home altitude.

- Units: m

- Range: -100 100

- Increment: 0.1

## GROUND_STEER_DPS: Ground steer rate

*Note: This parameter is for advanced users*

Ground steering rate in degrees per second for full rudder stick deflection

- Units: deg/s

- Range: 10 360

- Increment: 1

## MIXING_GAIN: Mixing Gain

The gain for the Vtail and elevon output mixers. The default is 0.5, which ensures that the mixer doesn't saturate, allowing both input channels to go to extremes while retaining control over the output. Hardware mixers often have a 1.0 gain, which gives more servo throw, but can saturate. If you don't have enough throw on your servos with VTAIL_OUTPUT or ELEVON_OUTPUT enabled then you can raise the gain using MIXING_GAIN. The mixer allows outputs in the range 900 to 2100 microseconds.

- Range: 0.5 1.2

## RUDDER_ONLY: Rudder only aircraft

Enable rudder only mode. The rudder will control attitude in attitude controlled modes (such as FBWA). You should setup your transmitter to send roll stick inputs to the RCMAP_YAW channel (normally channel 4). The rudder servo should be attached to the RCMAP_YAW channel as well. Note that automatic ground steering will be disabled for rudder only aircraft. You should also set KFF_RDDRMIX to 1.0. You will also need to setup the YAW2SRV_DAMP yaw damping appropriately for your aircraft. A value of 0.5 for YAW2SRV_DAMP is a good starting point.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## MIXING_OFFSET: Mixing Offset

The offset for the Vtail and elevon output mixers, as a percentage. This can be used in combination with MIXING_GAIN to configure how the control surfaces respond to input. The response to aileron or elevator input can be increased by setting this parameter to a positive or negative value. A common usage is to enter a positive value to increase the aileron response of the elevons of a flying wing. The default value of zero will leave the aileron-input response equal to the elevator-input response.

- Units: d%

- Range: -1000 1000

## DSPOILR_RUD_RATE: Differential spoilers rudder rate

Sets the amount of deflection that the rudder output will apply to the differential spoilers, as a percentage. The default value of 100 results in full rudder applying full deflection. A value of 0 will result in the differential spoilers exactly following the elevons (no rudder effect).

- Units: %

- Range: -100 100

## SYS_NUM_RESETS: Num Resets

*Note: This parameter is for advanced users*

Number of APM board resets

- ReadOnly: True

## LOG_BITMASK: Log bitmask

*Note: This parameter is for advanced users*

Bitmap of what on-board log types to enable. This value is made up of the sum of each of the log types you want to be saved. It is usually best just to enable all basic log types by setting this to 65535.

- Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:Performance,4:Control Tuning,5:Navigation Tuning,7:IMU,8:Mission Commands,9:Battery Monitor,10:Compass,11:TECS,12:Camera,13:RC Input-Output,14:Rangefinder,19:Raw IMU,20:Fullrate Attitude,21:Video Stabilization

## TRIM_ARSPD_CM: Target airspeed

Target airspeed in cm/s in automatic throttle modes. Value is as an indicated (calibrated/apparent) airspeed.

- Units: cm/s

## SCALING_SPEED: speed used for speed scaling calculations

*Note: This parameter is for advanced users*

Airspeed in m/s to use when calculating surface speed scaling. Note that changing this value will affect all PID values

- Units: m/s

- Range: 0 50

- Increment: 0.1

## MIN_GNDSPD_CM: Minimum ground speed

*Note: This parameter is for advanced users*

Minimum ground speed in cm/s when under airspeed control

- Units: cm/s

## TRIM_PITCH_CD: Pitch angle offset

*Note: This parameter is for advanced users*

Offset applied to AHRS pitch used for in-flight pitch trimming. Correct ground leveling is better than changing this parameter.

- Units: cdeg

- Range: -4500 4500

- Increment: 10

## ALT_HOLD_RTL: RTL altitude

Target altitude above home for RTL mode. Maintains current altitude if set to -1. Rally point altitudes are used if plane does not return to home.

- Units: cm

## ALT_HOLD_FBWCM: Minimum altitude for FBWB mode

This is the minimum altitude in centimeters that FBWB and CRUISE modes will allow. If you attempt to descend below this altitude then the plane will level off. A value of zero means no limit.

- Units: cm

## FLAP_1_PERCNT: Flap 1 percentage

*Note: This parameter is for advanced users*

The percentage change in flap position when FLAP_1_SPEED is reached. Use zero to disable flaps

- Range: 0 100

- Increment: 1

- Units: %

## FLAP_1_SPEED: Flap 1 speed

*Note: This parameter is for advanced users*

The speed in meters per second at which to engage FLAP_1_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED

- Range: 0 100

- Increment: 1

- Units: m/s

## FLAP_2_PERCNT: Flap 2 percentage

*Note: This parameter is for advanced users*

The percentage change in flap position when FLAP_2_SPEED is reached. Use zero to disable flaps

- Range: 0 100

- Units: %

- Increment: 1

## FLAP_2_SPEED: Flap 2 speed

*Note: This parameter is for advanced users*

The speed in meters per second at which to engage FLAP_2_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED

- Range: 0 100

- Units: m/s

- Increment: 1

## OVERRIDE_CHAN: IO override channel

*Note: This parameter is for advanced users*

If set to a non-zero value then this is an RC input channel number to use for giving IO manual control in case the main FMU microcontroller on a board with a IO co-processor fails. When this RC input channel goes above 1750 the FMU microcontroller will no longer be involved in controlling the servos and instead the IO microcontroller will directly control the servos. Note that IO manual control will be automatically activated if the FMU crashes for any reason. This parameter allows you to test for correct manual behaviour without actually crashing the FMU. This parameter is can be set to a non-zero value either for ground testing purposes or for giving the effect of an external override control board. Note that you may set OVERRIDE_CHAN to the same channel as FLTMODE_CH to get IO based override when in flight mode 6. Note that when override is triggered due to a FMU crash the 6 auxiliary output channels on the FMU will no longer be updated, so all the flight controls you need must be assigned to the first 8 channels on boards with an IOMCU.

- Range: 0 16

- Increment: 1

## RTL_AUTOLAND: RTL auto land

Automatically begin landing sequence after arriving at RTL location. This requires the addition of a DO_LAND_START mission item, which acts as a marker for the start of a landing sequence. The closest landing sequence will be chosen to the current location. If this is set to 0 and there is a DO_LAND_START mission item then you will get an arming check failure. You can set to a value of 3 to avoid the arming check failure and use the DO_LAND_START for go-around without it changing RTL behaviour. For a value of 1 a rally point will be used instead of HOME if in range (see rally point documentation).

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Fly HOME then land|
|2|Go directly to landing sequence|
|3|OnlyForGoAround|

## CRASH_ACC_THRESH: Crash Deceleration Threshold

*Note: This parameter is for advanced users*

X-Axis deceleration threshold to notify the crash detector that there was a possible impact which helps disarm the motor quickly after a crash. This value should be much higher than normal negative x-axis forces during normal flight, check flight log files to determine the average IMU.x values for your aircraft and motor type. Higher value means less sensative (triggers on higher impact). For electric planes that don't vibrate much during fight a value of 25 is good (that's about 2.5G). For petrol/nitro planes you'll want a higher value. Set to 0 to disable the collision detector.

- Units: m/s/s

- Range: 10 127

- Increment: 1

## CRASH_DETECT: Crash Detection

*Note: This parameter is for advanced users*

Automatically detect a crash during AUTO flight and perform the bitmask selected action(s). Disarm will turn off motor for safety and to help against burning out ESC and motor. Set to 0 to disable crash detection.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|

- Bitmask: 0:Disarm

## RNGFND_LANDING: Enable rangefinder for landing

This enables the use of a rangefinder for automatic landing. The rangefinder will be used both on the landing approach and for final flare

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## SYSID_ENFORCE: GCS sysid enforcement

*Note: This parameter is for advanced users*

This controls whether packets from other than the expected GCS system ID will be accepted

|Value|Meaning|
|:---:|:---:|
|0|NotEnforced|
|1|Enforced|

## RUDD_DT_GAIN: rudder differential thrust gain

gain control from rudder to differential thrust

- Range: 0 100

- Units: %

- Increment: 1

## MANUAL_RCMASK: Manual R/C pass-through mask

*Note: This parameter is for advanced users*

Mask of R/C channels to pass directly to corresponding output channel when in MANUAL mode. When in any mode except MANUAL the channels selected with this option behave normally. This parameter is designed to allow for complex mixing strategies to be used for MANUAL flight using transmitter based mixing. Note that when this option is used you need to be very careful with pre-flight checks to ensure that the output is correct both in MANUAL and non-MANUAL modes.

- Bitmask: 0:Chan1,1:Chan2,2:Chan3,3:Chan4,4:Chan5,5:Chan6,6:Chan7,7:Chan8,8:Chan9,9:Chan10,10:Chan11,11:Chan12,12:Chan13,13:Chan14,14:Chan15,15:Chan16

## HOME_RESET_ALT: Home reset altitude threshold

*Note: This parameter is for advanced users*

When the aircraft is within this altitude of the home waypoint, while disarmed it will automatically update the home position. Set to 0 to continously reset it.

|Value|Meaning|
|:---:|:---:|
|-1|Never reset|
|0|Always reset|

- Range: -1 127

- Units: m

## FLIGHT_OPTIONS: Flight mode options

*Note: This parameter is for advanced users*

Flight mode specific options

- Bitmask: 0:Rudder mixing in direct flight modes only (Manual / Stabilize / Acro),1:Use centered throttle in Cruise or FBWB to indicate trim airspeed, 2:Disable attitude check for takeoff arming, 3:Force target airspeed to trim airspeed in Cruise or FBWB, 4: Climb to ALT_HOLD_RTL before turning for RTL, 5: Enable yaw damper in acro mode, 6: Surpress speed scaling during auto takeoffs to be 1 or less to prevent oscillations without airpseed sensor., 7:EnableDefaultAirspeed for takeoff, 8: Remove the TRIM_PITCH_CD on the GCS horizon, 9: Remove the TRIM_PITCH_CD on the OSD horizon, 10: Adjust mid-throttle to be TRIM_THROTTLE in non-auto throttle modes except MANUAL, 11:Disable suppression of fixed wing rate gains in ground mode, 12: Enable FBWB style loiter altitude control

## TKOFF_ACCEL_CNT: Takeoff throttle acceleration count

This is the number of acceleration events to require for arming with TKOFF_THR_MINACC. The default is 1, which means a single forward acceleration above TKOFF_THR_MINACC will arm. By setting this higher than 1 you can require more forward/backward movements to arm.

- Range: 1 10

## DSPOILER_CROW_W1: Differential spoiler crow flaps outer weight

*Note: This parameter is for advanced users*

This is amount of deflection applied to the two outer surfaces for differential spoilers for flaps to give crow flaps. It is a number from 0 to 100. At zero no crow flaps are applied. A recommended starting value is 25.

- Range: 0 100

- Units: %

- Increment: 1

## DSPOILER_CROW_W2: Differential spoiler crow flaps inner weight

*Note: This parameter is for advanced users*

This is amount of deflection applied to the two inner surfaces for differential spoilers for flaps to give crow flaps. It is a number from 0 to 100. At zero no crow flaps are applied. A recommended starting value is 45.

- Range: 0 100

- Units: %

- Increment: 1

## TKOFF_TIMEOUT: Takeoff timeout

This is the timeout for an automatic takeoff. If this is non-zero and the aircraft does not reach a ground speed of at least 4 m/s within this number of seconds then the takeoff is aborted and the vehicle disarmed. If the value is zero then no timeout applies.

- Range: 0 120

- Increment: 1

- Units: s

## DSPOILER_OPTS: Differential spoiler and crow flaps options

*Note: This parameter is for advanced users*

Differential spoiler and crow flaps options

|Value|Meaning|
|:---:|:---:|
|0|none|
|1|D spoilers have pitch input|
|2|use both control surfaces on each wing for roll|
|4|Progressive crow flaps only first (0-50% flap in) then crow flaps (50 - 100% flap in)|

- Bitmask: 0:pitch control, 1:full span, 2:Progressive crow

## DSPOILER_AILMTCH: Differential spoiler aileron matching

*Note: This parameter is for advanced users*

This scales down the inner flaps so less than full downwards range can be used for differential spoiler and full span ailerons, 100 is use full range, upwards travel is unaffected

- Range: 0 100

- Units: %

- Increment: 1

## FWD_BAT_VOLT_MAX: Forward throttle battery voltage compensation maximum voltage

*Note: This parameter is for advanced users*

Forward throttle battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust). Recommend 4.2 * cell count, 0 = Disabled. Recommend THR_MAX is set to no more than 100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX, THR_MIN is set to no less than -100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX and climb descent rate limits are set accordingly.

- Range: 6 35

- Units: V

- Increment: 0.1

## FWD_BAT_VOLT_MIN: Forward throttle battery voltage compensation minimum voltage

*Note: This parameter is for advanced users*

Forward throttle battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled. Recommend THR_MAX is set to no more than 100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX, THR_MIN is set to no less than -100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX and climb descent rate limits are set accordingly.

- Range: 6 35

- Units: V

- Increment: 0.1

## FWD_BAT_IDX: Forward throttle battery compensation index

*Note: This parameter is for advanced users*

Which battery monitor should be used for doing compensation for the forward throttle

|Value|Meaning|
|:---:|:---:|
|0|First battery|
|1|Second battery|

## FS_EKF_THRESH: EKF failsafe variance threshold

*Note: This parameter is for advanced users*

Allows setting the maximum acceptable compass and velocity variance used to check navigation health in VTOL modes

|Value|Meaning|
|:---:|:---:|
|0.6|Strict|
|0.8|Default|
|1.0|Relaxed|

## RTL_CLIMB_MIN: RTL minimum climb

The vehicle will climb this many m during the initial climb portion of the RTL. During this time the roll will be limited to LEVEL_ROLL_LIMIT degrees.

- Units: m

- Range: 0 30

- Increment: 1

## MAN_EXPO_ROLL: Manual control expo for roll

Percentage exponential for roll input in MANUAL, ACRO and TRAINING modes

- Range: 0 100

- Increment: 1

## MAN_EXPO_PITCH: Manual input expo for pitch

Percentage exponential for pitch input in MANUAL, ACRO and TRAINING modes

- Range: 0 100

- Increment: 1

## MAN_EXPO_RUDDER: Manual input expo for rudder

Percentage exponential for rudder input in MANUAL, ACRO and TRAINING modes

- Range: 0 100

- Increment: 1

## ONESHOT_MASK: Oneshot output mask

*Note: This parameter is for advanced users*

Mask of output channels to use oneshot on

- Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15

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

## ARMING_REQUIRE: Require Arming Motors 

*Note: This parameter is for advanced users*

Arming disabled until some requirements are met. If 0, there are no requirements (arm immediately).  If 1, require rudder stick or GCS arming before arming motors and sends the minimum throttle PWM value to the throttle channel when disarmed.  If 2, require rudder stick or GCS arming and send 0 PWM to throttle channel when disarmed. See the ARMING_CHECK_* parameters to see what checks are done before arming. Note, if setting this parameter to 0 a reboot is required to arm the plane.  Also note, even with this parameter at 0, if ARMING_CHECK parameter is not also zero the plane may fail to arm throttle at boot due to a pre-arm check failure.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|THR_MIN PWM when disarmed|
|2|0 PWM when disarmed|

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

- Bitmask: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC Channels,7:Board voltage,8:Battery Level,9:Airspeed,10:Logging Available,11:Hardware safety switch,12:GPS Configuration,13:System,14:Mission,15:Rangefinder,16:Camera,17:AuxAuth,19:FFT

## ARMING_OPTIONS: Arming options

*Note: This parameter is for advanced users*

Options that can be applied to change arming behaviour

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Disable prearm display|

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

Bitmask of options to use with airspeed. 0:Disable use based on airspeed/groundspeed mismatch (see ARSPD_WIND_MAX), 1:Automatically reenable use based on airspeed/groundspeed mismatch recovery (see ARSPD_WIND_MAX) 2:Disable voltage correction, 3:Check that the airspeed is statistically consistent with the navigation EKF vehicle and wind velocity estimates using EKF3 (requires AHRS_EKF_TYPE = 3)

- Bitmask: 0:SpeedMismatchDisable, 1:AllowSpeedMismatchRecovery, 2:DisableVoltageCorrection, 3:UseEkf3Consistency

## ARSPD_WIND_MAX: Maximum airspeed and ground speed difference

*Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy. Using ARSPD_OPTION this health value can be used to disable the sensor.

- Units: m/s

## ARSPD_WIND_WARN: Airspeed and ground speed difference that gives a warning

*Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will issue a warning. If 0 ARSPD_WIND_MAX is used.

- Units: m/s

## ARSPD_WIND_GATE: Re-enable Consistency Check Gate Size

*Note: This parameter is for advanced users*

Number of standard deviations applied to the re-enable EKF consistency check that is used when ARSPD_OPTIONS bit position 3 is set. Larger values will make the re-enabling of the airspeed sensor faster, but increase the likelihood of re-enabling a degraded sensor. The value can be tuned by using the ARSP.TR log message by setting ARSP_WIND_GATE to a value that is higher than the value for ARSP.TR observed with a healthy airspeed sensor. Occasional transients in ARSP.TR above the value set by ARSP_WIND_GATE can be tolerated provided they are less than 5 seconds in duration and less than 10% duty cycle.

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

## BATT2_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT2_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT3_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT3_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT4_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT4_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT5_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT5_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT6_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT6_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT7_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT7_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT8_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT8_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT9_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT9_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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

## BATT_WATT_MAX: Maximum allowed power (Watts)

*Note: This parameter is for advanced users*

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.

- Units: W

- Increment: 1

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
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|6|Loiter to QLand|

## BATT_FS_CRT_ACT: Critical battery failsafe action

What action the vehicle should perform if it hits a critical battery failsafe

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RTL|
|2|Land|
|3|Terminate|
|4|QLand|
|5|Parachute|
|6|Loiter to QLand|

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
|4|ModeRTL|
|9|Camera Trigger|
|11|Fence|
|16|ModeAuto|
|22|Parachute Release|
|24|Auto Mission Reset|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Plane Sound|
|31|Motor Emergency Stop|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|38|ADSB Avoidance En|
|41|ArmDisarm (4.1 and lower)|
|43|InvertedFlight|
|46|RC Override Enable|
|51|ModeManual|
|52|ModeACRO|
|55|ModeGuided|
|56|ModeLoiter|
|58|Clear Waypoints|
|62|Compass Learn|
|64|Reverse Throttle|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|72|ModeCircle|
|77|ModeTakeoff|
|78|RunCam Control|
|79|RunCam OSD Control|
|81|Disarm|
|82|QAssist 3pos|
|84|Air Mode|
|85|Generator|
|86|Non Auto Terrain Follow Disable|
|87|Crow Select|
|88|Soaring Enable|
|89|Landing Flare|
|90|EKF Pos Source|
|91|Airspeed Ratio Calibration|
|92|FBWA|
|94|VTX Power|
|95|FBWA taildragger takeoff mode|
|96|trigger re-reading of mode switch|
|98|ModeTraining|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|106|Disable Airspeed Use|
|107|EnableFixedWingAutotune|
|108|ModeQRTL|
|150|CRUISE|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with Quadplane AirMode (4.2 and higher)|
|155|set roll pitch and yaw trim to current servo and RC|
|157|Force FS Action to FBWA|
|158|Optflow Calibration|
|160|Weathervane Enable|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|208|Flap|
|209|Forward Throttle|
|210|Airbrakes|
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
|4|ModeRTL|
|9|Camera Trigger|
|11|Fence|
|16|ModeAuto|
|22|Parachute Release|
|24|Auto Mission Reset|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Plane Sound|
|31|Motor Emergency Stop|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|38|ADSB Avoidance En|
|41|ArmDisarm (4.1 and lower)|
|43|InvertedFlight|
|46|RC Override Enable|
|51|ModeManual|
|52|ModeACRO|
|55|ModeGuided|
|56|ModeLoiter|
|58|Clear Waypoints|
|62|Compass Learn|
|64|Reverse Throttle|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|72|ModeCircle|
|77|ModeTakeoff|
|78|RunCam Control|
|79|RunCam OSD Control|
|81|Disarm|
|82|QAssist 3pos|
|84|Air Mode|
|85|Generator|
|86|Non Auto Terrain Follow Disable|
|87|Crow Select|
|88|Soaring Enable|
|89|Landing Flare|
|90|EKF Pos Source|
|91|Airspeed Ratio Calibration|
|92|FBWA|
|94|VTX Power|
|95|FBWA taildragger takeoff mode|
|96|trigger re-reading of mode switch|
|98|ModeTraining|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|106|Disable Airspeed Use|
|107|EnableFixedWingAutotune|
|108|ModeQRTL|
|150|CRUISE|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with Quadplane AirMode (4.2 and higher)|
|155|set roll pitch and yaw trim to current servo and RC|
|157|Force FS Action to FBWA|
|158|Optflow Calibration|
|160|Weathervane Enable|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|208|Flap|
|209|Forward Throttle|
|210|Airbrakes|
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
|4|ModeRTL|
|9|Camera Trigger|
|11|Fence|
|16|ModeAuto|
|22|Parachute Release|
|24|Auto Mission Reset|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Plane Sound|
|31|Motor Emergency Stop|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|38|ADSB Avoidance En|
|41|ArmDisarm (4.1 and lower)|
|43|InvertedFlight|
|46|RC Override Enable|
|51|ModeManual|
|52|ModeACRO|
|55|ModeGuided|
|56|ModeLoiter|
|58|Clear Waypoints|
|62|Compass Learn|
|64|Reverse Throttle|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|72|ModeCircle|
|77|ModeTakeoff|
|78|RunCam Control|
|79|RunCam OSD Control|
|81|Disarm|
|82|QAssist 3pos|
|84|Air Mode|
|85|Generator|
|86|Non Auto Terrain Follow Disable|
|87|Crow Select|
|88|Soaring Enable|
|89|Landing Flare|
|90|EKF Pos Source|
|91|Airspeed Ratio Calibration|
|92|FBWA|
|94|VTX Power|
|95|FBWA taildragger takeoff mode|
|96|trigger re-reading of mode switch|
|98|ModeTraining|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|106|Disable Airspeed Use|
|107|EnableFixedWingAutotune|
|108|ModeQRTL|
|150|CRUISE|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with Quadplane AirMode (4.2 and higher)|
|155|set roll pitch and yaw trim to current servo and RC|
|157|Force FS Action to FBWA|
|158|Optflow Calibration|
|160|Weathervane Enable|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|208|Flap|
|209|Forward Throttle|
|210|Airbrakes|
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
|4|ModeRTL|
|9|Camera Trigger|
|11|Fence|
|16|ModeAuto|
|22|Parachute Release|
|24|Auto Mission Reset|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Plane Sound|
|31|Motor Emergency Stop|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|38|ADSB Avoidance En|
|41|ArmDisarm (4.1 and lower)|
|43|InvertedFlight|
|46|RC Override Enable|
|51|ModeManual|
|52|ModeACRO|
|55|ModeGuided|
|56|ModeLoiter|
|58|Clear Waypoints|
|62|Compass Learn|
|64|Reverse Throttle|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|72|ModeCircle|
|77|ModeTakeoff|
|78|RunCam Control|
|79|RunCam OSD Control|
|81|Disarm|
|82|QAssist 3pos|
|84|Air Mode|
|85|Generator|
|86|Non Auto Terrain Follow Disable|
|87|Crow Select|
|88|Soaring Enable|
|89|Landing Flare|
|90|EKF Pos Source|
|91|Airspeed Ratio Calibration|
|92|FBWA|
|94|VTX Power|
|95|FBWA taildragger takeoff mode|
|96|trigger re-reading of mode switch|
|98|ModeTraining|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|106|Disable Airspeed Use|
|107|EnableFixedWingAutotune|
|108|ModeQRTL|
|150|CRUISE|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with Quadplane AirMode (4.2 and higher)|
|155|set roll pitch and yaw trim to current servo and RC|
|157|Force FS Action to FBWA|
|158|Optflow Calibration|
|160|Weathervane Enable|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|208|Flap|
|209|Forward Throttle|
|210|Airbrakes|
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
|1|RTL|
|6|Guided|
|7|GuidedThrottlePass|

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

## FENCE_RET_RALLY: Fence Return to Rally

Should the vehicle return to fence return point or rally point

|Value|Meaning|
|:---:|:---:|
|0|Fence Return Point|
|1|Nearest Rally Point|

- Range: 0 1

- Increment: 1

## FENCE_RET_ALT: Fence Return Altitude

Altitude the vehicle will transit to when a fence breach occurs

- Units: m

- Range: 0 32767

- Increment: 1

## FENCE_AUTOENABLE: Fence Auto-Enable

Auto-enable of fence

|Value|Meaning|
|:---:|:---:|
|0|AutoEnableOff|
|1|AutoEnableOnTakeoff|
|2|AutoEnableDisableFloorOnLanding|
|3|AutoEnableOnlyWhenArmed|

- Range: 0 3

- Increment: 1

## FENCE_OPTIONS: Fence options

0:Disable mode change following fence action until fence breach is cleared

- Bitmask: 0:Disable mode change following fence action until fence breach is cleared

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

# GUIDED Parameters

## GUIDED_P: PID Proportional Gain

P Gain which produces an output value that is proportional to the current error value

## GUIDED_I: PID Integral Gain

I Gain which produces an output that is proportional to both the magnitude and the duration of the error

## GUIDED_D: PID Derivative Gain

D Gain which produces an output that is proportional to the rate of change of the error

## GUIDED_FF: FF FeedForward Gain

FF Gain which produces an output value that is proportional to the demanded input

## GUIDED_IMAX: PID Integral Maximum

The maximum/minimum value that the I term can output

## GUIDED_FLTT: PID Target filter frequency in Hz

Target filter frequency in Hz

- Units: Hz

## GUIDED_FLTE: PID Error filter frequency in Hz

Error filter frequency in Hz

- Units: Hz

## GUIDED_FLTD: PID Derivative term filter frequency in Hz

Derivative filter frequency in Hz

- Units: Hz

## GUIDED_SMAX: Slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

# ICE Parameters

## ICE_ENABLE: Enable ICEngine control

*Note: This parameter is for advanced users*

This enables internal combustion engine control

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## ICE_START_CHAN: Input channel for engine start

This is an RC input channel for requesting engine start. Engine will try to start when channel is at or above 1700. Engine will stop when channel is at or below 1300. Between 1301 and 1699 the engine will not change state unless a MAVLink command or mission item commands a state change, or the vehicle is disarmed. See ICE_STARTCHN_MIN parameter to change engine stop PWM value and/or to enable debouncing of the START_CH to avoid accidental engine kills due to noise on channel.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|Chan1|
|2|Chan2|
|3|Chan3|
|4|Chan4|
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

## ICE_STARTER_TIME: Time to run starter

This is the number of seconds to run the starter when trying to start the engine

- Units: s

- Range: 0.1 5

## ICE_START_DELAY: Time to wait between starts

Delay between start attempts

- Units: s

- Range: 1 10

## ICE_RPM_THRESH: RPM threshold

This is the measured RPM above which the engine is considered to be running

- Range: 100 100000

## ICE_PWM_IGN_ON: PWM value for ignition on

This is the value sent to the ignition channel when on

- Range: 1000 2000

## ICE_PWM_IGN_OFF: PWM value for ignition off

This is the value sent to the ignition channel when off

- Range: 1000 2000

## ICE_PWM_STRT_ON: PWM value for starter on

This is the value sent to the starter channel when on

- Range: 1000 2000

## ICE_PWM_STRT_OFF: PWM value for starter off

This is the value sent to the starter channel when off

- Range: 1000 2000

## ICE_RPM_CHAN: RPM instance channel to use

This is which of the RPM instances to use for detecting the RPM of the engine

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RPM1|
|2|RPM2|

## ICE_START_PCT: Throttle percentage for engine start

This is the percentage throttle output for engine start

- Range: 0 100

## ICE_IDLE_PCT: Throttle percentage for engine idle

This is the minimum percentage throttle output while running, this includes being disarmed, but not safe

- Range: 0 100

## ICE_IDLE_RPM: RPM Setpoint for Idle Governor

*Note: This parameter is for advanced users*

This configures the RPM that will be commanded by the idle governor. Set to -1 to disable

## ICE_IDLE_DB: Deadband for Idle Governor

This configures the deadband that is tolerated before adjusting the idle setpoint

## ICE_IDLE_SLEW: Slew Rate for idle control

This configures the slewrate used to adjust the idle setpoint in percentage points per second

## ICE_OPTIONS: ICE options

Options for ICE control. The DisableIgnitionRCFailsafe option will cause the ignition to be set off on any R/C failsafe. If ThrottleWhileDisarmed is set then throttle control will be allowed while disarmed for planes when in MANUAL mode.

- Bitmask: 0:DisableIgnitionRCFailsafe,1:DisableRedineGovernor,2:ThrottleWhileDisarmed

## ICE_STARTCHN_MIN: Input channel for engine start minimum PWM

This is a minimum PWM value for engine start channel for an engine stop to be commanded. Setting this value will avoid RC input glitches with low PWM values from causing an unwanted engine stop. A value of zero means any PWM below 1300 triggers an engine stop.

- Range: 0 1300

## ICE_REDLINE_RPM: RPM of the redline limit for the engine

*Note: This parameter is for advanced users*

Maximum RPM for the engine provided by the manufacturer. A value of 0 disables this feature. See ICE_OPTIONS to enable or disable the governor.

- Range: 0 2000000

- Units: RPM

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

# LAND Parameters

## LAND_SLOPE_RCALC: Landing slope re-calc threshold

*Note: This parameter is for advanced users*

This parameter is used when using a rangefinder during landing for altitude correction from baro drift (RNGFND_LANDING=1) and the altitude correction indicates your altitude is lower than the intended slope path. This value is the threshold of the correction to re-calculate the landing approach slope. Set to zero to keep the original slope all the way down and any detected baro drift will be corrected by pitching/throttling up to snap back to resume the original slope path. Otherwise, when a rangefinder altitude correction exceeds this threshold it will trigger a slope re-calculate to give a shallower slope. This also smoothes out the approach when flying over objects such as trees. Recommend a value of 2m.

- Range: 0 5

- Units: m

- Increment: 0.5

## LAND_ABORT_DEG: Landing auto-abort slope threshold

*Note: This parameter is for advanced users*

This parameter is used when using a rangefinder during landing for altitude correction from baro drift (RNGFND_LANDING=1) and the altitude correction indicates your actual altitude is higher than the intended slope path. Normally it would pitch down steeply but that can result in a crash with high airspeed so this allows remembering the baro offset and self-abort the landing and come around for another landing with the correct baro offset applied for a perfect slope. An auto-abort go-around will only happen once, next attempt will not auto-abort again. This operation happens entirely automatically in AUTO mode. This value is the delta degrees threshold to trigger the go-around compared to the original slope. Example: if set to 5 deg and the mission planned slope is 15 deg then if the new slope is 21 then it will go-around. Set to 0 to disable. Requires LAND_SLOPE_RCALC > 0.

- Range: 0 90

- Units: deg

- Increment: 0.1

## LAND_PITCH_CD: Landing Pitch

*Note: This parameter is for advanced users*

Used in autoland to give the minimum pitch in the final stage of landing (after the flare). This parameter can be used to ensure that the final landing attitude is appropriate for the type of undercarriage on the aircraft. Note that it is a minimum pitch only - the landing code will control pitch above this value to try to achieve the configured landing sink rate.

- Units: cdeg

- Range: -2000 2000

- Increment: 10

## LAND_FLARE_ALT: Landing flare altitude

*Note: This parameter is for advanced users*

Altitude in autoland at which to lock heading and flare to the LAND_PITCH_CD pitch. Note that this option is secondary to LAND_FLARE_SEC. For a good landing it preferable that the flare is triggered by LAND_FLARE_SEC.

- Units: m

- Range: 0 30

- Increment: 0.1

## LAND_FLARE_SEC: Landing flare time

*Note: This parameter is for advanced users*

Vertical time before landing point at which to lock heading and flare with the motor stopped. This is vertical time, and is calculated based solely on the current height above the ground and the current descent rate.  Set to 0 if you only wish to flare based on altitude (see LAND_FLARE_ALT).

- Units: s

- Range: 0 10

- Increment: 0.1

## LAND_PF_ALT: Landing pre-flare altitude

*Note: This parameter is for advanced users*

Altitude to trigger pre-flare flight stage where LAND_PF_ARSPD controls airspeed. The pre-flare flight stage trigger works just like LAND_FLARE_ALT but higher. Disabled when LAND_PF_ARSPD is 0.

- Units: m

- Range: 0 30

- Increment: 0.1

## LAND_PF_SEC: Landing pre-flare time

*Note: This parameter is for advanced users*

Vertical time to ground to trigger pre-flare flight stage where LAND_PF_ARSPD controls airspeed. This pre-flare flight stage trigger works just like LAND_FLARE_SEC but earlier. Disabled when LAND_PF_ARSPD is 0.

- Units: s

- Range: 0 10

- Increment: 0.1

## LAND_PF_ARSPD: Landing pre-flare airspeed

*Note: This parameter is for advanced users*

Desired airspeed during pre-flare flight stage. This is useful to reduce airspeed just before the flare. Use 0 to disable.

- Units: m/s

- Range: 0 30

- Increment: 0.1

## LAND_THR_SLEW: Landing throttle slew rate

This parameter sets the slew rate for the throttle during auto landing. When this is zero the THR_SLEWRATE parameter is used during landing. The value is a percentage throttle change per second, so a value of 20 means to advance the throttle over 5 seconds on landing. Values below 50 are not recommended as it may cause a stall when airspeed is low and you can not throttle up fast enough.

- Units: %

- Range: 0 127

- Increment: 1

## LAND_DISARMDELAY: Landing disarm delay

*Note: This parameter is for advanced users*

After a landing has completed using a LAND waypoint, automatically disarm after this many seconds have passed. Use 0 to not disarm.

- Units: s

- Increment: 1

- Range: 0 127

## LAND_THEN_NEUTRL: Set servos to neutral after landing

*Note: This parameter is for advanced users*

When enabled, after an autoland and auto-disarm via LAND_DISARMDELAY happens then set all servos to neutral. This is helpful when an aircraft has a rough landing upside down or a crazy angle causing the servos to strain.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Servos to Neutral|
|2|Servos to Zero PWM|

## LAND_ABORT_THR: Landing abort using throttle

*Note: This parameter is for advanced users*

Allow a landing abort to trigger with an input throttle >= 90%. This works with or without stick-mixing enabled.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## LAND_FLAP_PERCNT: Landing flap percentage

*Note: This parameter is for advanced users*

The amount of flaps (as a percentage) to apply in the landing approach and flare of an automatic landing

- Range: 0 100

- Units: %

- Increment: 1

## LAND_OPTIONS: Landing options bitmask

*Note: This parameter is for advanced users*

Bitmask of options to use with landing.

- Bitmask: 0: honor min throttle during landing flare

## LAND_TYPE: Auto-landing type

Specifies the auto-landing type to use

|Value|Meaning|
|:---:|:---:|
|0|Standard Glide Slope|
|1|Deepstall|

# LANDDS Parameters

## LAND_DS_V_FWD: Deepstall forward velocity

*Note: This parameter is for advanced users*

The forward velocity of the aircraft while stalled

- Range: 0 20

- Units: m/s

## LAND_DS_SLOPE_A: Deepstall slope a

*Note: This parameter is for advanced users*

The a component of distance = a*wind + b

## LAND_DS_SLOPE_B: Deepstall slope b

*Note: This parameter is for advanced users*

The a component of distance = a*wind + b

## LAND_DS_APP_EXT: Deepstall approach extension

*Note: This parameter is for advanced users*

The horizontal distance from which the aircraft will approach before the stall

- Range: 10 200

- Units: m

## LAND_DS_V_DWN: Deepstall velocity down

*Note: This parameter is for advanced users*

The downward velocity of the aircraft while stalled

- Range: 0 20

- Units: m/s

## LAND_DS_SLEW_SPD: Deepstall slew speed

*Note: This parameter is for advanced users*

The speed at which the elevator slews to deepstall

- Range: 0 2

- Units: s

## LAND_DS_ELEV_PWM: Deepstall elevator PWM

*Note: This parameter is for advanced users*

The PWM value in microseconds for the elevator at full deflection in deepstall

- Range: 900 2100

- Units: PWM

## LAND_DS_ARSP_MAX: Deepstall enabled airspeed

*Note: This parameter is for advanced users*

The maximum aispeed where the deepstall steering controller is allowed to have control

- Range: 5 20

- Units: m/s

## LAND_DS_ARSP_MIN: Deepstall minimum derating airspeed

*Note: This parameter is for advanced users*

Deepstall lowest airspeed where the deepstall controller isn't allowed full control

- Range: 5 20

- Units: m/s

## LAND_DS_L1: Deepstall L1 period

*Note: This parameter is for advanced users*

Deepstall L1 navigational controller period

- Range: 5 50

- Units: s

## LAND_DS_L1_I: Deepstall L1 I gain

*Note: This parameter is for advanced users*

Deepstall L1 integratior gain

- Range: 0 1

## LAND_DS_YAW_LIM: Deepstall yaw rate limit

*Note: This parameter is for advanced users*

The yaw rate limit while navigating in deepstall

- Range: 0 90

- Units: deg/s

## LAND_DS_L1_TCON: Deepstall L1 time constant

*Note: This parameter is for advanced users*

Time constant for deepstall L1 control

- Range: 0 1

- Units: s

## LAND_DS_P: P gain

P gain

## LAND_DS_I: I gain

I gain

## LAND_DS_D: D gain

D gain

## LAND_DS_IMAX: IMax

Maximum integrator value

## LAND_DS_ABORTALT: Deepstall minimum abort altitude

*Note: This parameter is for advanced users*

The minimum altitude which the aircraft must be above to abort a deepstall landing

- Range: 0 50

- Units: m

## LAND_DS_AIL_SCL: Aileron landing gain scalaing

*Note: This parameter is for advanced users*

A scalar to reduce or increase the aileron control

- Range: 0 2.0

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

- Bitmask: 0:Clear Mission on reboot, 1:Use distance to land calc on battery failsafe,2:ContinueAfterLand

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

# NAVL1 Parameters

## NAVL1_PERIOD: L1 control period

Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.

- Units: s

- Range: 1 60

- Increment: 1

## NAVL1_DAMPING: L1 control damping ratio

*Note: This parameter is for advanced users*

Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.

- Range: 0.6 1.0

- Increment: 0.05

## NAVL1_XTRACK_I: L1 control crosstrack integrator gain

*Note: This parameter is for advanced users*

Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.

- Range: 0 0.1

- Increment: 0.01

## NAVL1_LIM_BANK: Loiter Radius Bank Angle Limit

*Note: This parameter is for advanced users*

The sealevel bank angle limit for a continous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly

- Units: deg

- Range: 0 89

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

# PTCH Parameters

## PTCH2SRV_TCONST: Pitch Time Constant

*Note: This parameter is for advanced users*

Time constant in seconds from demanded to achieved pitch angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.

- Range: 0.4 1.0

- Units: s

- Increment: 0.1

## PTCH2SRV_RMAX_UP: Pitch up max rate

*Note: This parameter is for advanced users*

This sets the maximum nose up pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.

- Range: 0 100

- Units: deg/s

- Increment: 1

## PTCH2SRV_RMAX_DN: Pitch down max rate

*Note: This parameter is for advanced users*

This sets the maximum nose down pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.

- Range: 0 100

- Units: deg/s

- Increment: 1

## PTCH2SRV_RLL: Roll compensation

Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.

- Range: 0.7 1.5

- Increment: 0.05

## PTCH_RATE_P: Pitch axis rate controller P gain

Pitch axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output

- Range: 0.08 0.35

- Increment: 0.005

## PTCH_RATE_I: Pitch axis rate controller I gain

Pitch axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate

- Range: 0.01 0.6

- Increment: 0.01

## PTCH_RATE_IMAX: Pitch axis rate controller I gain maximum

Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## PTCH_RATE_D: Pitch axis rate controller D gain

Pitch axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate

- Range: 0.001 0.03

- Increment: 0.001

## PTCH_RATE_FF: Pitch axis rate controller feed forward

Pitch axis rate controller feed forward

- Range: 0 3.0

- Increment: 0.001

## PTCH_RATE_FLTT: Pitch axis rate controller target frequency in Hz

Pitch axis rate controller target frequency in Hz

- Range: 2 50

- Increment: 1

- Units: Hz

## PTCH_RATE_FLTE: Pitch axis rate controller error frequency in Hz

Pitch axis rate controller error frequency in Hz

- Range: 2 50

- Increment: 1

- Units: Hz

## PTCH_RATE_FLTD: Pitch axis rate controller derivative frequency in Hz

Pitch axis rate controller derivative frequency in Hz

- Range: 0 50

- Increment: 1

- Units: Hz

## PTCH_RATE_SMAX: Pitch slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

# Q Parameters

## Q_ENABLE: Enable QuadPlane

This enables QuadPlane functionality, assuming multicopter motors start on output 5. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|
|2|Enable VTOL AUTO|

- RebootRequired: True

## Q_ANGLE_MAX: Angle Max

*Note: This parameter is for advanced users*

Maximum lean angle in all VTOL flight modes

- Units: cdeg

- Increment: 10

- Range: 1000 8000

## Q_TRANSITION_MS: Transition time

*Note: This parameter is for advanced users*

Transition time in milliseconds after minimum airspeed is reached

- Units: ms

- Range: 1 30000

## Q_VELZ_MAX: Pilot maximum vertical speed up

The maximum ascending vertical velocity the pilot may request in cm/s

- Units: cm/s

- Range: 50 500

- Increment: 10

## Q_VELZ_MAX_DN: Pilot maximum vertical speed down

The maximum vertical velocity the pilot may request in cm/s going down. If 0, uses Q_VELZ_MAX value.

- Units: cm/s

- Range: 50 500

- Increment: 10

## Q_ACCEL_Z: Pilot vertical acceleration

The vertical acceleration used when pilot is controlling the altitude

- Units: cm/s/s

- Range: 50 500

- Increment: 10

## Q_RC_SPEED: RC output speed in Hz

This is the PWM refresh rate in Hz for QuadPlane quad motors

- Units: Hz

- Range: 50 500

- Increment: 10

## Q_ASSIST_SPEED: Quadplane assistance speed

This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes. Zero means no assistance except during transition. Note that if this is set to zero then other Q_ASSIST features are also disabled. A higher value will lead to more false positives which can waste battery. A lower value will result in less false positive, but will result in assistance taking longer to trigger. If unsure then set to 3 m/s below the minimum airspeed you will fly at. If you don't have an airspeed sensor then use 5 m/s below the minimum airspeed you fly at. If you want to disable the arming check Q_ASSIST_SPEED then set to -1.

- Units: m/s

- Range: 0 100

- Increment: 0.1

## Q_YAW_RATE_MAX: Maximum yaw rate

This is the maximum yaw rate for pilot input on rudder stick in degrees/second

- Units: deg/s

- Range: 50 500

- Increment: 1

## Q_LAND_SPEED: Land speed

The descent speed for the final stage of landing in cm/s

- Units: cm/s

- Range: 30 200

- Increment: 10

## Q_LAND_FINAL_ALT: Land final altitude

The altitude at which we should switch to Q_LAND_SPEED descent rate

- Units: m

- Range: 0.5 50

- Increment: 0.1

## Q_TRAN_PIT_MAX: Transition max pitch

Maximum pitch during transition to auto fixed wing flight

- Range: 0 30

- Units: deg

- Increment: 1

## Q_FRAME_CLASS: Frame Class

Controls major frame class for multicopter component

|Value|Meaning|
|:---:|:---:|
|0|Undefined|
|1|Quad|
|2|Hexa|
|3|Octa|
|4|OctaQuad|
|5|Y6|
|7|Tri|
|10|Single/Dual|
|12|DodecaHexa|
|14|Deca|
|15|Scripting Matrix|
|17|Dynamic Scripting Matrix|

## Q_FRAME_TYPE: Frame Type (+, X or V)

Controls motor mixing for multicopter component

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
|16|MOTOR_FRAME_TYPE_NYT_PLUS|
|17|MOTOR_FRAME_TYPE_NYT_X|
|18|BetaFlightXReversed|

## Q_VFWD_GAIN: Forward velocity hold gain

Controls use of forward motor in vtol modes. If this is zero then the forward motor will not be used for position control in VTOL modes. A value of 0.05 is a good place to start if you want to use the forward motor for position control. No forward motor will be used in QSTABILIZE or QHOVER modes. Use QLOITER for position hold with the forward motor.

- Range: 0 0.5

- Increment: 0.01

## Q_RTL_ALT: QRTL return altitude

The altitude which QRTL mode heads to initially

- Units: m

- Range: 1 200

- Increment: 1

## Q_RTL_MODE: VTOL RTL mode

If this is set to 1 then an RTL will change to QRTL when within RTL_RADIUS meters of the RTL destination, VTOL approach: vehicle will RTL at RTL alt and circle with a radius of Q_FW_LND_APR_RAD down to Q_RTL_ALT and then transition into the wind and QRTL, see 'AUTO VTOL Landing', QRTL Always: do a QRTL instead of RTL

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|
|2|VTOL approach|
|3|QRTL Always|

## Q_GUIDED_MODE: Enable VTOL in GUIDED mode

This enables use of VTOL in guided mode. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## Q_ESC_CAL: ESC Calibration

This is used to calibrate the throttle range of the VTOL motors. Please read https://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|ThrottleInput|
|2|FullInput|

## Q_VFWD_ALT: Forward velocity alt cutoff

Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.

- Units: m

- Range: 0 10

- Increment: 0.25

## Q_LAND_ICE_CUT: Cut IC engine on landing

This controls stopping an internal combustion engine in the final landing stage of a VTOL. This is important for aircraft where the forward thrust engine may experience prop-strike if left running during landing. This requires the engine controls are enabled using the ICE_* parameters.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## Q_ASSIST_ANGLE: Quadplane assistance angle

This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least Q_ASSIST_DELAY seconds. Set to zero to disable angle assistance.

- Units: deg

- Range: 0 90

- Increment: 1

## Q_MAV_TYPE: MAVLink type identifier

This controls the mavlink type given in HEARTBEAT messages. For some GCS types a particular setting will be needed for correct operation.

|Value|Meaning|
|:---:|:---:|
|0|AUTO|
|1|FIXED_WING|
|2|QUADROTOR|
|3|COAXIAL|
|4|HELICOPTER|
|7|AIRSHIP|
|8|FREE_BALLOON|
|9|ROCKET|
|10|GROUND_ROVER|
|11|SURFACE_BOAT|
|12|SUBMARINE|
|16|FLAPPING_WING|
|17|KITE|
|19|VTOL_DUOROTOR|
|20|VTOL_QUADROTOR|
|21|VTOL_TILTROTOR|

## Q_OPTIONS: quadplane options

Level Transition:Keep wings within LEVEL_ROLL_LIMIT and only use forward motor(s) for climb during transition, Allow FW Takeoff: If bit is not set then NAV_TAKEOFF command on quadplanes will instead perform a NAV_VTOL takeoff, Allow FW Land:If bit is not set then NAV_LAND command on quadplanes will instead perform a NAV_VTOL_LAND, Vtol Takeoff Frame: command NAV_VTOL_TAKEOFF altitude is as set by the command's reference frame rather than a delta above current location, Always use FW spiral approach:Always use Use a fixed wing spiral approach for VTOL landings, USE QRTL:instead of QLAND for rc failsafe when in VTOL modes, Use Governor:Use ICE Idle Governor in MANUAL for forward motor, Force Qassist: on always,Mtrs_Only_Qassist: in tailsitters only, uses VTOL motors and not flying surfaces for QASSIST, Airmode_On_Arm:Airmode enabled when arming by aux switch, Disarmed Yaw Tilt:Enable motor tilt for yaw when disarmed, Delay Spoolup:Delay VTOL spoolup for 2 seconds after arming, ThrLandControl: enable throttle stick control of landing rate, DisableApproach: Disable use of approach and airbrake stages in VTOL landing, EnableLandResposition: enable pilot controlled repositioning in AUTO land. Descent will pause while repositioning. ARMVTOL: Arm only in VTOL or AUTO modes. CompleteTransition: to fixed wing if Q_TRANS_FAIL timer times out instead of QLAND. Force RTL mode: forces RTL mode on rc failsafe in VTOL modes overriding bit 5(USE_QRTL).

- Bitmask: 0:Level Transition,1:Allow FW Takeoff,2:Allow FW Land,3:Vtol Takeoff Frame,4:Always use FW spiral approach,5:Use QRTL,6:Use Governor,7:Force Qassist,8:Mtrs_Only_Qassist,10:Disarmed Yaw Tilt,11:Delay Spoolup,12:disable Qassist based on synthetic airspeed,13:Disable Ground Effect Compensation,14:Ignore forward flight angle limits in Qmodes,15:ThrLandControl,16:DisableApproach,17:EnableLandReposition,18:ARMVtol, 19: CompleteTransition if Q_TRANS_FAIL, 20: Force RTL mode on VTOL failsafes overriding bit 5(USE QRTL), 21:Tilt rotor tilt motors up when disarmed in FW modes (except manual) to prevent ground strikes

## Q_TRANS_DECEL: Transition deceleration

This is deceleration rate that will be used in calculating the stopping distance when transitioning from fixed wing flight to multicopter flight.

- Units: m/s/s

- Increment: 0.1

- Range: 0.2 5

## Q_TRIM_PITCH: Quadplane AHRS trim pitch

*Note: This parameter is for advanced users*

This sets the compensation for the pitch angle trim difference between calibrated AHRS level and vertical flight pitch. NOTE! this is relative to calibrated AHRS trim, not forward flight trim which includes TRIM_PITCH_CD. For tailsitters, this is relative to a baseline of 90 degrees in AHRS.

- Units: deg

- Range: -10 +10

- Increment: 0.1

- RebootRequired: True

## Q_FW_LND_APR_RAD: Quadplane fixed wing landing approach radius

*Note: This parameter is for advanced users*

This provides the radius used, when using a fixed wing landing approach. If set to 0 then the WP_LOITER_RAD will be selected.

- Units: m

- Range: 0 200

- Increment: 5

## Q_TRANS_FAIL: Quadplane transition failure time

*Note: This parameter is for advanced users*

Maximum time allowed for forward transitions, exceeding this time will cancel the transition and the aircraft will immediately change to the mode set by Q_TRANS_FAIL_ACT or finish the transition depending on Q_OPTIONS bit 19. 0 for no limit.

- Units: s

- Range: 0 20

- Increment: 1

## Q_THROTTLE_EXPO: Throttle expo strength

*Note: This parameter is for advanced users*

Amount of curvature in throttle curve: 0 is linear, 1 is cubic

- Range: 0 1

- Increment: .1

## Q_ACRO_RLL_RATE: QACRO mode roll rate

The maximum roll rate at full stick deflection in QACRO mode

- Units: deg/s

- Range: 10 500

- Increment: 1

## Q_ACRO_PIT_RATE: QACRO mode pitch rate

The maximum pitch rate at full stick deflection in QACRO mode

- Units: deg/s

- Range: 10 500

- Increment: 1

## Q_ACRO_YAW_RATE: QACRO mode yaw rate

The maximum yaw rate at full stick deflection in QACRO mode

- Units: deg/s

- Range: 10 500

- Increment: 1

## Q_TKOFF_FAIL_SCL: Takeoff time failure scalar

*Note: This parameter is for advanced users*

Scalar for how long past the expected takeoff time a takeoff should be considered as failed and the vehicle will switch to QLAND. If set to 0 there is no limit on takeoff time.

- Range: 1.1 5.0

- Increment: 5.1

## Q_TKOFF_ARSP_LIM: Takeoff airspeed limit

*Note: This parameter is for advanced users*

Airspeed limit during takeoff. If the airspeed exceeds this level the vehicle will switch to QLAND. This is useful for ensuring that you don't takeoff into excessively strong wind. If set to 0 there is no limit on airspeed during takeoff.

- Units: m/s

- Range: 0 20

- Increment: 1

## Q_ASSIST_ALT: Quadplane assistance altitude

This is the altitude below which quadplane assistance will be triggered. This acts the same way as Q_ASSIST_ANGLE and Q_ASSIST_SPEED, but triggers if the aircraft drops below the given altitude while the VTOL motors are not running. A value of zero disables this feature. The altutude is calculated as being above ground level. The height above ground is given from a Lidar used if available and RNGFND_LANDING=1. Otherwise it comes from terrain data if TERRAIN_FOLLOW=1 and comes from height above home otherwise.

- Units: m

- Range: 0 120

- Increment: 1

## Q_ASSIST_DELAY: Quadplane assistance delay

This is delay between the assistance thresholds being met and the assistance starting.

- Units: s

- Range: 0 2

- Increment: 0.1

## Q_FWD_MANTHR_MAX: VTOL manual forward throttle max percent

Maximum value for manual forward throttle; used with RC option FWD_THR (209)

- Range: 0 100

## Q_BACKTRANS_MS: SLT and Tiltrotor back transition pitch limit duration

Pitch angle will increase from 0 to angle max over this duration when switching into VTOL flight in a postion control mode. 0 Disables.

- Units: ms

- Range: 0 10000

## Q_TRANS_FAIL_ACT: Quadplane transition failure action

This sets the mode that is changed to when Q_TRANS_FAIL time elapses, if set. See also Q_OPTIONS bit 19: CompleteTransition if Q_TRANS_FAIL

|Value|Meaning|
|:---:|:---:|
|-1|Warn only|
|0|QLand|
|1|QRTL|

## Q_LAND_ALTCHG: Land detection altitude change threshold

The maximum altitude change allowed during land detection. You can raise this value if you find that landing detection takes a long time to complete. It is the maximum change in altitude over a period of 4 seconds for landing to be detected

- Units: m

- Range: 0.1 0.6

- Increment: 0.05

## Q_NAVALT_MIN: Minimum navigation altitude

*Note: This parameter is for advanced users*

This is the altitude in meters above which navigation begins in auto takeoff. Below this altitude the target roll and pitch will be zero. A value of zero disables the feature

- Range: 0 5

## Q_PLT_Y_RATE: Pilot controlled yaw rate

Pilot controlled yaw rate max. Used in all pilot controlled modes except QAcro

- Units: deg/s

- Range: 1 360

## Q_PLT_Y_EXPO: Pilot controlled yaw expo

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

## Q_PLT_Y_RATE_TC: Pilot yaw rate control input time constant

Pilot yaw rate control input time constant. Low numbers lead to sharper response, higher numbers to softer response.

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

# QAUTOTUNE Parameters

## Q_AUTOTUNE_AXES: Autotune axis bitmask

1-byte bitmap of axes to autotune

- Bitmask: 0:Roll,1:Pitch,2:Yaw

## Q_AUTOTUNE_AGGR: Autotune aggressiveness

Autotune aggressiveness. Defines the bounce back used to detect size of the D term.

- Range: 0.05 0.10

## Q_AUTOTUNE_MIN_D: AutoTune minimum D

Defines the minimum D gain

- Range: 0.001 0.006

# QA Parameters

## Q_A_SLEW_YAW: Yaw target slew rate

*Note: This parameter is for advanced users*

Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes

- Units: cdeg/s

- Range: 500 18000

- Increment: 100

## Q_A_ACCEL_Y_MAX: Acceleration Max for Yaw

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

## Q_A_RATE_FF_ENAB: Rate Feedforward Enable

*Note: This parameter is for advanced users*

Controls whether body-frame rate feedfoward is enabled or disabled

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## Q_A_ACCEL_R_MAX: Acceleration Max for Roll

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

## Q_A_ACCEL_P_MAX: Acceleration Max for Pitch

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

## Q_A_ANGLE_BOOST: Angle Boost

*Note: This parameter is for advanced users*

Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Enabled|

## Q_A_ANG_RLL_P: Roll axis angle controller P gain

Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate

- Range: 3.000 12.000

## Q_A_ANG_PIT_P: Pitch axis angle controller P gain

Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate

- Range: 3.000 12.000

## Q_A_ANG_YAW_P: Yaw axis angle controller P gain

Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate

- Range: 3.000 12.000

## Q_A_ANG_LIM_TC: Angle Limit (to maintain altitude) Time Constant

*Note: This parameter is for advanced users*

Angle Limit (to maintain altitude) Time Constant

- Range: 0.5 10.0

## Q_A_RATE_R_MAX: Angular Velocity Max for Roll

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

## Q_A_RATE_P_MAX: Angular Velocity Max for Pitch

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

## Q_A_RATE_Y_MAX: Angular Velocity Max for Yaw

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

## Q_A_INPUT_TC: Attitude control input time constant

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

## Q_A_RAT_RLL_P: Roll axis rate controller P gain

Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output

- Range: 0.01 0.5

- Increment: 0.005

## Q_A_RAT_RLL_I: Roll axis rate controller I gain

Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate

- Range: 0.01 2.0

- Increment: 0.01

## Q_A_RAT_RLL_IMAX: Roll axis rate controller I gain maximum

Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## Q_A_RAT_RLL_D: Roll axis rate controller D gain

Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate

- Range: 0.0 0.05

- Increment: 0.001

## Q_A_RAT_RLL_FF: Roll axis rate controller feed forward

Roll axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## Q_A_RAT_RLL_FLTT: Roll axis rate controller target frequency in Hz

Roll axis rate controller target frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## Q_A_RAT_RLL_FLTE: Roll axis rate controller error frequency in Hz

Roll axis rate controller error frequency in Hz

- Range: 0 100

- Increment: 1

- Units: Hz

## Q_A_RAT_RLL_FLTD: Roll axis rate controller derivative frequency in Hz

Roll axis rate controller derivative frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## Q_A_RAT_RLL_SMAX: Roll slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## Q_A_RAT_PIT_P: Pitch axis rate controller P gain

Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output

- Range: 0.01 0.50

- Increment: 0.005

## Q_A_RAT_PIT_I: Pitch axis rate controller I gain

Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate

- Range: 0.01 2.0

- Increment: 0.01

## Q_A_RAT_PIT_IMAX: Pitch axis rate controller I gain maximum

Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## Q_A_RAT_PIT_D: Pitch axis rate controller D gain

Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate

- Range: 0.0 0.05

- Increment: 0.001

## Q_A_RAT_PIT_FF: Pitch axis rate controller feed forward

Pitch axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## Q_A_RAT_PIT_FLTT: Pitch axis rate controller target frequency in Hz

Pitch axis rate controller target frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## Q_A_RAT_PIT_FLTE: Pitch axis rate controller error frequency in Hz

Pitch axis rate controller error frequency in Hz

- Range: 0 100

- Increment: 1

- Units: Hz

## Q_A_RAT_PIT_FLTD: Pitch axis rate controller derivative frequency in Hz

Pitch axis rate controller derivative frequency in Hz

- Range: 5 100

- Increment: 1

- Units: Hz

## Q_A_RAT_PIT_SMAX: Pitch slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## Q_A_RAT_YAW_P: Yaw axis rate controller P gain

Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output

- Range: 0.10 2.50

- Increment: 0.005

## Q_A_RAT_YAW_I: Yaw axis rate controller I gain

Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate

- Range: 0.010 1.0

- Increment: 0.01

## Q_A_RAT_YAW_IMAX: Yaw axis rate controller I gain maximum

Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## Q_A_RAT_YAW_D: Yaw axis rate controller D gain

Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate

- Range: 0.000 0.02

- Increment: 0.001

## Q_A_RAT_YAW_FF: Yaw axis rate controller feed forward

Yaw axis rate controller feed forward

- Range: 0 0.5

- Increment: 0.001

## Q_A_RAT_YAW_FLTT: Yaw axis rate controller target frequency in Hz

Yaw axis rate controller target frequency in Hz

- Range: 1 50

- Increment: 1

- Units: Hz

## Q_A_RAT_YAW_FLTE: Yaw axis rate controller error frequency in Hz

Yaw axis rate controller error frequency in Hz

- Range: 0 20

- Increment: 1

- Units: Hz

## Q_A_RAT_YAW_FLTD: Yaw axis rate controller derivative frequency in Hz

Yaw axis rate controller derivative frequency in Hz

- Range: 5 50

- Increment: 1

- Units: Hz

## Q_A_RAT_YAW_SMAX: Yaw slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## Q_A_THR_MIX_MIN: Throttle Mix Minimum

*Note: This parameter is for advanced users*

Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)

- Range: 0.1 0.25

## Q_A_THR_MIX_MAX: Throttle Mix Maximum

*Note: This parameter is for advanced users*

Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

- Range: 0.5 0.9

## Q_A_THR_MIX_MAN: Throttle Mix Manual

*Note: This parameter is for advanced users*

Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)

- Range: 0.1 0.9

# QLOIT Parameters

## Q_LOIT_ANG_MAX: Loiter pilot angle max

*Note: This parameter is for advanced users*

Loiter maximum pilot requested lean angle. Set to zero for 2/3 of Q_P_ANGLE_MAX/Q_ANGLE_MAX. The maximum vehicle lean angle is still limited by Q_P_ANGLE_MAX/Q_ANGLE_MAX

- Units: deg

- Range: 0 45

- Increment: 1

## Q_LOIT_SPEED: Loiter Horizontal Maximum Speed

Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode

- Units: cm/s

- Range: 20 3500

- Increment: 50

## Q_LOIT_ACC_MAX: Loiter maximum correction acceleration

*Note: This parameter is for advanced users*

Loiter maximum correction acceleration in cm/s/s.  Higher values cause the copter to correct position errors more aggressively.

- Units: cm/s/s

- Range: 100 981

- Increment: 1

## Q_LOIT_BRK_ACCEL: Loiter braking acceleration

*Note: This parameter is for advanced users*

Loiter braking acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered.

- Units: cm/s/s

- Range: 25 250

- Increment: 1

## Q_LOIT_BRK_JERK: Loiter braking jerk

*Note: This parameter is for advanced users*

Loiter braking jerk in cm/s/s/s. Higher values will remove braking faster if the pilot moves the sticks during a braking maneuver.

- Units: cm/s/s/s

- Range: 500 5000

- Increment: 1

## Q_LOIT_BRK_DELAY: Loiter brake start delay (in seconds)

*Note: This parameter is for advanced users*

Loiter brake start delay (in seconds)

- Units: s

- Range: 0 2

- Increment: 0.1

# QM Parameters

## Q_M_YAW_HEADROOM: Matrix Yaw Min

*Note: This parameter is for advanced users*

Yaw control is given at least this pwm in microseconds range

- Range: 0 500

- Units: PWM

## Q_M_THST_EXPO: Thrust Curve Expo

*Note: This parameter is for advanced users*

Motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)

- Range: -1.0 1.0

## Q_M_SPIN_MAX: Motor Spin maximum

*Note: This parameter is for advanced users*

Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range

|Value|Meaning|
|:---:|:---:|
|0.9|Low|
|0.95|Default|
|1.0|High|

## Q_M_BAT_VOLT_MAX: Battery voltage compensation maximum voltage

*Note: This parameter is for advanced users*

Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.2 * cell count, 0 = Disabled

- Range: 6 53

- Units: V

## Q_M_BAT_VOLT_MIN: Battery voltage compensation minimum voltage

*Note: This parameter is for advanced users*

Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.3 * cell count, 0 = Disabled

- Range: 6 42

- Units: V

## Q_M_BAT_CURR_MAX: Motor Current Max

*Note: This parameter is for advanced users*

Maximum current over which maximum throttle is limited (0 = Disabled)

- Range: 0 200

- Units: A

## Q_M_PWM_TYPE: Output PWM type

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

## Q_M_PWM_MIN: PWM output minimum

*Note: This parameter is for advanced users*

This sets the min PWM output value in microseconds that will ever be output to the motors

- Units: PWM

- Range: 0 2000

## Q_M_PWM_MAX: PWM output maximum

*Note: This parameter is for advanced users*

This sets the max PWM value in microseconds that will ever be output to the motors

- Units: PWM

- Range: 0 2000

## Q_M_SPIN_MIN: Motor Spin minimum

*Note: This parameter is for advanced users*

Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.  Should be higher than MOT_SPIN_ARM.

|Value|Meaning|
|:---:|:---:|
|0.0|Low|
|0.15|Default|
|0.25|High|

## Q_M_SPIN_ARM: Motor Spin armed

*Note: This parameter is for advanced users*

Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range.  Should be lower than MOT_SPIN_MIN.

|Value|Meaning|
|:---:|:---:|
|0.0|Low|
|0.1|Default|
|0.2|High|

## Q_M_BAT_CURR_TC: Motor Current Max Time Constant

*Note: This parameter is for advanced users*

Time constant used to limit the maximum current

- Range: 0 10

- Units: s

## Q_M_THST_HOVER: Thrust Hover Value

*Note: This parameter is for advanced users*

Motor thrust needed to hover expressed as a number from 0 to 1

- Range: 0.2 0.8

## Q_M_HOVER_LEARN: Hover Value Learning

*Note: This parameter is for advanced users*

Enable/Disable automatic learning of hover throttle

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Learn|
|2|Learn and Save|

## Q_M_SAFE_DISARM: Motor PWM output disabled when disarmed

*Note: This parameter is for advanced users*

Disables motor PWM output when disarmed

|Value|Meaning|
|:---:|:---:|
|0|PWM enabled while disarmed|
|1|PWM disabled while disarmed|

## Q_M_YAW_SV_ANGLE: Yaw Servo Max Lean Angle

Yaw servo's maximum lean angle (Tricopter only)

- Range: 5 80

- Units: deg

- Increment: 1

## Q_M_SPOOL_TIME: Spool up time

*Note: This parameter is for advanced users*

Time in seconds to spool up the motors from zero to min throttle. 

- Range: 0 2

- Units: s

- Increment: 0.1

## Q_M_BOOST_SCALE: Motor boost scale

*Note: This parameter is for advanced users*

Booster motor output scaling factor vs main throttle.  The output to the BoostThrottle servo will be the main throttle times this scaling factor. A higher scaling factor will put more of the load on the booster motor. A value of 1 will set the BoostThrottle equal to the main throttle.

- Range: 0 5

- Increment: 0.1

## Q_M_BAT_IDX: Battery compensation index

*Note: This parameter is for advanced users*

Which battery monitor should be used for doing compensation

|Value|Meaning|
|:---:|:---:|
|0|First battery|
|1|Second battery|

## Q_M_SLEW_UP_TIME: Output slew time for increasing throttle

*Note: This parameter is for advanced users*

Time in seconds to slew output from zero to full. This is used to limit the rate at which output can change. Range is constrained between 0 and 0.5.

- Range: 0 .5

- Units: s

- Increment: 0.001

## Q_M_SLEW_DN_TIME: Output slew time for decreasing throttle

*Note: This parameter is for advanced users*

Time in seconds to slew output from full to zero. This is used to limit the rate at which output can change.  Range is constrained between 0 and 0.5.

- Range: 0 .5

- Units: s

- Increment: 0.001

## Q_M_SAFE_TIME: Time taken to disable and enable the motor PWM output when disarmed and armed.

*Note: This parameter is for advanced users*

Time taken to disable and enable the motor PWM output when disarmed and armed.

- Range: 0 5

- Units: s

- Increment: 0.001

# QP Parameters

## Q_P_ACC_XY_FILT: XY Acceleration filter cutoff frequency

*Note: This parameter is for advanced users*

Lower values will slow the response of the navigation controller and reduce twitchiness

- Units: Hz

- Range: 0.5 5

- Increment: 0.1

## Q_P_POSZ_P: Position (vertical) controller P gain

Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller

- Range: 1.000 3.000

## Q_P_VELZ_P: Velocity (vertical) controller P gain

Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller

- Range: 1.000 8.000

## Q_P_VELZ_I: Velocity (vertical) controller I gain

*Note: This parameter is for advanced users*

Velocity (vertical) controller I gain.  Corrects long-term difference in desired velocity to a target acceleration

- Range: 0.02 1.00

- Increment: 0.01

## Q_P_VELZ_IMAX: Velocity (vertical) controller I gain maximum

Velocity (vertical) controller I gain maximum.  Constrains the target acceleration that the I gain will output

- Range: 1.000 8.000

## Q_P_VELZ_D: Velocity (vertical) controller D gain

*Note: This parameter is for advanced users*

Velocity (vertical) controller D gain.  Corrects short-term changes in velocity

- Range: 0.00 1.00

- Increment: 0.001

## Q_P_VELZ_FF: Velocity (vertical) controller Feed Forward gain

*Note: This parameter is for advanced users*

Velocity (vertical) controller Feed Forward gain.  Produces an output that is proportional to the magnitude of the target

- Range: 0 1

- Increment: 0.01

## Q_P_VELZ_FLTE: Velocity (vertical) error filter

*Note: This parameter is for advanced users*

Velocity (vertical) error filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## Q_P_VELZ_FLTD: Velocity (vertical) input filter for D term

*Note: This parameter is for advanced users*

Velocity (vertical) input filter for D term.  This filter (in Hz) is applied to the input for D terms

- Range: 0 100

- Units: Hz

## Q_P_ACCZ_P: Acceleration (vertical) controller P gain

Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output

- Range: 0.200 1.500

- Increment: 0.05

## Q_P_ACCZ_I: Acceleration (vertical) controller I gain

Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration

- Range: 0.000 3.000

## Q_P_ACCZ_IMAX: Acceleration (vertical) controller I gain maximum

Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate

- Range: 0 1000

- Units: d%

## Q_P_ACCZ_D: Acceleration (vertical) controller D gain

Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration

- Range: 0.000 0.400

## Q_P_ACCZ_FF: Acceleration (vertical) controller feed forward

Acceleration (vertical) controller feed forward

- Range: 0 0.5

- Increment: 0.001

## Q_P_ACCZ_FLTT: Acceleration (vertical) controller target frequency in Hz

Acceleration (vertical) controller target frequency in Hz

- Range: 1 50

- Increment: 1

- Units: Hz

## Q_P_ACCZ_FLTE: Acceleration (vertical) controller error frequency in Hz

Acceleration (vertical) controller error frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## Q_P_ACCZ_FLTD: Acceleration (vertical) controller derivative frequency in Hz

Acceleration (vertical) controller derivative frequency in Hz

- Range: 1 100

- Increment: 1

- Units: Hz

## Q_P_ACCZ_SMAX: Accel (vertical) slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

## Q_P_POSXY_P: Position (horizontal) controller P gain

Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller

- Range: 0.500 2.000

## Q_P_VELXY_P: Velocity (horizontal) P gain

*Note: This parameter is for advanced users*

Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration

- Range: 0.1 6.0

- Increment: 0.1

## Q_P_VELXY_I: Velocity (horizontal) I gain

*Note: This parameter is for advanced users*

Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration

- Range: 0.02 1.00

- Increment: 0.01

## Q_P_VELXY_D: Velocity (horizontal) D gain

*Note: This parameter is for advanced users*

Velocity (horizontal) D gain.  Corrects short-term changes in velocity

- Range: 0.00 1.00

- Increment: 0.001

## Q_P_VELXY_IMAX: Velocity (horizontal) integrator maximum

*Note: This parameter is for advanced users*

Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output

- Range: 0 4500

- Increment: 10

- Units: cm/s/s

## Q_P_VELXY_FLTE: Velocity (horizontal) input filter

*Note: This parameter is for advanced users*

Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms

- Range: 0 100

- Units: Hz

## Q_P_VELXY_FLTD: Velocity (horizontal) input filter

*Note: This parameter is for advanced users*

Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term

- Range: 0 100

- Units: Hz

## Q_P_VELXY_FF: Velocity (horizontal) feed forward gain

*Note: This parameter is for advanced users*

Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration

- Range: 0 6

- Increment: 0.01

## Q_P_ANGLE_MAX: Position Control Angle Max

*Note: This parameter is for advanced users*

Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value

- Units: deg

- Range: 0 45

- Increment: 1

## Q_P_JERK_XY: Jerk limit for the horizontal kinematic input shaping

*Note: This parameter is for advanced users*

Jerk limit of the horizontal kinematic path generation used to determine how quickly the aircraft varies the acceleration target

- Units: m/s/s/s

- Range: 1 20

- Increment: 1

## Q_P_JERK_Z: Jerk limit for the vertical kinematic input shaping

*Note: This parameter is for advanced users*

Jerk limit of the vertical kinematic path generation used to determine how quickly the aircraft varies the acceleration target

- Units: m/s/s/s

- Range: 5 50

- Increment: 1

# QTAILSIT Parameters

## Q_TAILSIT_ENABLE: Enable Tailsitter

This enables Tailsitter functionality. A value of 2 forces Qassist active and always stabilize in forward flight with airmode for stabalisation at 0 throttle, for use on vehicles with no control surfaces, vehicle will not arm in forward flight modes, see also Q_OPTIONS "Mtrs_Only_Qassist"

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|
|2|Enable Always|

- RebootRequired: True

## Q_TAILSIT_ANGLE: Tailsitter fixed wing transition angle

This is the pitch angle at which tailsitter aircraft will change from VTOL control to fixed wing control.

- Units: deg

- Range: 5 80

## Q_TAILSIT_ANG_VT: Tailsitter VTOL transition angle

This is the pitch angle at which tailsitter aircraft will change from fixed wing control to VTOL control, if zero Q_TAILSIT_ANGLE will be used

- Units: deg

- Range: 5 80

## Q_TAILSIT_INPUT: Tailsitter input type bitmask

This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When PlaneMode is not enabled (bit0 = 0) the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When PlaneMode input is enabled, the roll and yaw sticks are swapped so that the roll stick controls earth-frame yaw and rudder controls earth-frame roll. When body-frame roll is enabled (bit1 = 1), the yaw stick controls earth-frame yaw rate and the roll stick controls roll in the tailsitter's body frame when flying level.

- Bitmask: 0:PlaneMode,1:BodyFrameRoll

## Q_TAILSIT_VFGAIN: Tailsitter vector thrust gain in forward flight

This controls the amount of vectored thrust control used in forward flight for a vectored tailsitter

- Range: 0 1

- Increment: 0.01

## Q_TAILSIT_VHGAIN: Tailsitter vector thrust gain in hover

This controls the amount of vectored thrust control used in hover for a vectored tailsitter

- Range: 0 1

- Increment: 0.01

## Q_TAILSIT_VHPOW: Tailsitter vector thrust gain power

This controls the amount of extra pitch given to the vectored control when at high pitch errors

- Range: 0 4

- Increment: 0.1

## Q_TAILSIT_GSCMAX: Maximum tailsitter gain scaling

Maximum gain scaling for tailsitter Q_TAILSIT_GSCMSK options

- Range: 1 5

## Q_TAILSIT_RLL_MX: Maximum Roll angle

Maximum Allowed roll angle for tailsitters. If this is zero then Q_ANGLE_MAX is used.

- Units: deg

- Range: 0 80

## Q_TAILSIT_MOTMX: Tailsitter motor mask

Bitmask of motors to remain active in forward flight for a 'Copter' tailsitter. Non-zero indicates airframe is a Copter tailsitter and uses copter style motor layouts determined by Q_FRAME_CLASS and Q_FRAME_TYPE. This should be zero for non-Copter tailsitters.

- Bitmask: 0:Motor 1,1:Motor 2,2:Motor 3,3:Motor 4, 4:Motor 5,5:Motor 6,6:Motor 7,7:Motor 8

## Q_TAILSIT_GSCMSK: Tailsitter gain scaling mask

Bitmask of gain scaling methods to be applied: Throttle: scale gains with throttle, ATT_THR: reduce gain at high throttle/tilt, 2:Disk theory velocity calculation, requires Q_TAILSIT_DSKLD to be set, ATT_THR must not be set, 3:Altitude correction, scale with air density

- Bitmask: 0:Throttle,1:ATT_THR,2:Disk Theory,3:Altitude correction

## Q_TAILSIT_GSCMIN: Minimum tailsitter gain scaling

Minimum gain scaling for tailsitter Q_TAILSIT_GSCMSK options

- Range: 0.1 1

## Q_TAILSIT_DSKLD: Tailsitter disk loading

This is the vehicle weight in kg divided by the total disk area of all propellers in m^2. Only used with Q_TAILSIT_GSCMSK = 4

- Units: kg/m/m

- Range: 0 50

## Q_TAILSIT_RAT_FW: Tailsitter VTOL to forward flight transition rate

The pitch rate at which tailsitter aircraft will pitch down in the transition from VTOL to forward flight

- Units: deg/s

- Range: 10 500

## Q_TAILSIT_RAT_VT: Tailsitter forward flight to VTOL transition rate

The pitch rate at which tailsitter aircraft will pitch up in the transition from forward flight to VTOL

- Units: deg/s

- Range: 10 500

## Q_TAILSIT_THR_VT: Tailsitter forward flight to VTOL transition throttle

Throttle used during FW->VTOL transition, -1 uses hover throttle

- Units: %

- Range: -1 100

## Q_TAILSIT_VT_R_P: Tailsitter VTOL control surface roll gain

Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains

- Range: 0 2

## Q_TAILSIT_VT_P_P: Tailsitter VTOL control surface pitch gain

Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains

- Range: 0 2

## Q_TAILSIT_VT_Y_P: Tailsitter VTOL control surface yaw gain

Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains

- Range: 0 2

## Q_TAILSIT_MIN_VO: Tailsitter Disk loading minimum outflow speed

Use in conjunction with disk therory gain scaling and Q_TAILSIT_DSKLD to specify minumum airspeed over control surfaces, this will be used to boost throttle, when decending for example, 0 disables

- Range: 0 15

# QTILT Parameters

## Q_TILT_ENABLE: Enable Tiltrotor functionality

This enables Tiltrotor functionality

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

- RebootRequired: True

## Q_TILT_MASK: Tiltrotor mask

This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.

## Q_TILT_RATE_UP: Tiltrotor upwards tilt rate

This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover

- Units: deg/s

- Increment: 1

- Range: 10 300

## Q_TILT_MAX: Tiltrotor maximum VTOL angle

This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE

- Units: deg

- Increment: 1

- Range: 20 80

## Q_TILT_TYPE: Tiltrotor type

This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrottor must use the tailsitter frame class (10)

|Value|Meaning|
|:---:|:---:|
|0|Continuous|
|1|Binary|
|2|VectoredYaw|
|3|Bicopter|

## Q_TILT_RATE_DN: Tiltrotor downwards tilt rate

This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.

- Units: deg/s

- Increment: 1

- Range: 10 300

## Q_TILT_YAW_ANGLE: Tilt minimum angle for vectored yaw

This is the angle of the tilt servos when in VTOL mode and at minimum output. This needs to be set for Q_TILT_TYPE=3 to enable vectored control for yaw of tricopter tilt quadplanes. This is also used to limit the forwards travel of bicopter tilts when in VTOL modes

- Range: 0 30

## Q_TILT_FIX_ANGLE: Fixed wing tiltrotor angle

This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft

- Units: deg

- Range: 0 30

## Q_TILT_FIX_GAIN: Fixed wing tiltrotor gain

This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes

- Range: 0 1

## Q_TILT_WING_FLAP: Tiltrotor tilt angle that will be used as flap

For use on tilt wings, the wing will tilt up to this angle for flap, transition will be complete when the wing reaches this angle from the forward fight position, 0 disables

- Units: deg

- Increment: 1

- Range: 0 15

# QWP Parameters

## Q_WP_SPEED: Waypoint Horizontal Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission

- Units: cm/s

- Range: 20 2000

- Increment: 50

## Q_WP_RADIUS: Waypoint Radius

Defines the distance from a waypoint, that when crossed indicates the wp has been hit.

- Units: cm

- Range: 5 1000

- Increment: 1

## Q_WP_SPEED_UP: Waypoint Climb Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission

- Units: cm/s

- Range: 10 1000

- Increment: 50

## Q_WP_SPEED_DN: Waypoint Descent Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission

- Units: cm/s

- Range: 10 500

- Increment: 10

## Q_WP_ACCEL: Waypoint Acceleration 

Defines the horizontal acceleration in cm/s/s used during missions

- Units: cm/s/s

- Range: 50 500

- Increment: 10

## Q_WP_ACCEL_Z: Waypoint Vertical Acceleration

Defines the vertical acceleration in cm/s/s used during missions

- Units: cm/s/s

- Range: 50 500

- Increment: 10

## Q_WP_RFND_USE: Waypoint missions use rangefinder for terrain following

*Note: This parameter is for advanced users*

This controls if waypoint missions use rangefinder for terrain following

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## Q_WP_JERK: Waypoint Jerk

Defines the horizontal jerk in m/s/s used during missions

- Units: m/s/s/s

- Range: 1 20

## Q_WP_TER_MARGIN: Waypoint Terrain following altitude margin

*Note: This parameter is for advanced users*

Waypoint Terrain following altitude margin.  Vehicle will stop if distance from target altitude is larger than this margin (in meters)

- Units: m

- Range: 0.1 100

# QWVANE Parameters

## Q_WVANE_ENABLE: Enable

Enable weather vaning.  When active, the aircraft will automatically yaw into wind when in a VTOL position controlled mode. Pilot yaw commands overide the weathervaning action.

|Value|Meaning|
|:---:|:---:|
|0|Disabled|
|1|Nose into wind|
|2|Nose or tail into wind|
|3|Side into wind|
|4|tail into wind|

## Q_WVANE_GAIN: Weathervaning gain

This converts the target roll/pitch angle of the aircraft into the correcting (into wind) yaw rate. e.g. Gain = 2, roll = 30 deg, pitch = 0 deg, yaw rate = 60 deg/s.

- Range: 0.5 4

- Increment: 0.1

## Q_WVANE_ANG_MIN: Weathervaning min angle

The minimum target roll/pitch angle before active weathervaning will start.  This provides a dead zone that is particularly useful for poorly trimmed quadplanes.

- Units: deg

- Range: 0 10

- Increment: 0.1

## Q_WVANE_HGT_MIN: Weathervaning min height

Above this height weathervaning is permitted.  If RNGFND_LANDING is enabled or terrain is enabled then this parameter sets height AGL. Otherwise this parameter sets height above home.  Set zero to ignore minimum height requirement to activate weathervaning

- Units: m

- Range: 0 50

- Increment: 1

## Q_WVANE_SPD_MAX: Weathervaning max ground speed

Below this ground speed weathervaning is permitted. Set to 0 to ignore this condition when checking if vehicle should weathervane.

- Units: m/s

- Range: 0 50

- Increment: 0.1

## Q_WVANE_VELZ_MAX: Weathervaning max vertical speed

The maximum climb or descent speed that the vehicle will still attempt to weathervane. Set to 0 to ignore this condition to get the aircraft to weathervane at any climb/descent rate.  This is particularly useful for aircraft with low disc loading that struggle with yaw control in decent.

- Units: m/s

- Range: 0 5

- Increment: 0.1

## Q_WVANE_TAKEOFF: Takeoff override

Override the weather vaning behaviour when in takeoffs

|Value|Meaning|
|:---:|:---:|
|-1|No override|
|0|Disabled|
|1|Nose into wind|
|2|Nose or tail into wind|
|3|Side into wind|
|4|tail into wind|

## Q_WVANE_LAND: Landing override

Override the weather vaning behaviour when in landing

|Value|Meaning|
|:---:|:---:|
|-1|No override|
|0|Disabled|
|1|Nose into wind|
|2|Nose or tail into wind|
|3|Side into wind|
|4|tail into wind|

## Q_WVANE_OPTIONS: Weathervaning options

Options impacting weathervaning behaviour

- Bitmask: 0:Use pitch when nose or tail-in for faster weathervaning

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
|4|ModeRTL|
|9|Camera Trigger|
|11|Fence|
|16|ModeAuto|
|22|Parachute Release|
|24|Auto Mission Reset|
|27|Retract Mount1|
|28|Relay On/Off|
|29|Landing Gear|
|30|Lost Plane Sound|
|31|Motor Emergency Stop|
|34|Relay2 On/Off|
|35|Relay3 On/Off|
|36|Relay4 On/Off|
|38|ADSB Avoidance En|
|41|ArmDisarm (4.1 and lower)|
|43|InvertedFlight|
|46|RC Override Enable|
|51|ModeManual|
|52|ModeACRO|
|55|ModeGuided|
|56|ModeLoiter|
|58|Clear Waypoints|
|62|Compass Learn|
|64|Reverse Throttle|
|65|GPS Disable|
|66|Relay5 On/Off|
|67|Relay6 On/Off|
|72|ModeCircle|
|77|ModeTakeoff|
|78|RunCam Control|
|79|RunCam OSD Control|
|81|Disarm|
|82|QAssist 3pos|
|84|Air Mode|
|85|Generator|
|86|Non Auto Terrain Follow Disable|
|87|Crow Select|
|88|Soaring Enable|
|89|Landing Flare|
|90|EKF Pos Source|
|91|Airspeed Ratio Calibration|
|92|FBWA|
|94|VTX Power|
|95|FBWA taildragger takeoff mode|
|96|trigger re-reading of mode switch|
|98|ModeTraining|
|100|KillIMU1|
|101|KillIMU2|
|102|Camera Mode Toggle|
|105|GPS Disable Yaw|
|106|Disable Airspeed Use|
|107|EnableFixedWingAutotune|
|108|ModeQRTL|
|150|CRUISE|
|153|ArmDisarm (4.2 and higher)|
|154|ArmDisarm with Quadplane AirMode (4.2 and higher)|
|155|set roll pitch and yaw trim to current servo and RC|
|157|Force FS Action to FBWA|
|158|Optflow Calibration|
|160|Weathervane Enable|
|162|FFT Tune|
|163|Mount Lock|
|164|Pause Stream Logging|
|165|Arm/Emergency Motor Stop|
|166|Camera Record Video|
|167|Camera Zoom|
|168|Camera Manual Focus|
|169|Camera Auto Focus|
|208|Flap|
|209|Forward Throttle|
|210|Airbrakes|
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

# RLL Parameters

## RLL2SRV_TCONST: Roll Time Constant

*Note: This parameter is for advanced users*

Time constant in seconds from demanded to achieved roll angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.

- Range: 0.4 1.0

- Units: s

- Increment: 0.1

## RLL2SRV_RMAX: Maximum Roll Rate

*Note: This parameter is for advanced users*

This sets the maximum roll rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables this limit.

- Range: 0 180

- Units: deg/s

- Increment: 1

## RLL_RATE_P: Roll axis rate controller P gain

Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output

- Range: 0.08 0.35

- Increment: 0.005

## RLL_RATE_I: Roll axis rate controller I gain

Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate

- Range: 0.01 0.6

- Increment: 0.01

## RLL_RATE_IMAX: Roll axis rate controller I gain maximum

Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## RLL_RATE_D: Roll axis rate controller D gain

Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate

- Range: 0.001 0.03

- Increment: 0.001

## RLL_RATE_FF: Roll axis rate controller feed forward

Roll axis rate controller feed forward

- Range: 0 3.0

- Increment: 0.001

## RLL_RATE_FLTT: Roll axis rate controller target frequency in Hz

Roll axis rate controller target frequency in Hz

- Range: 2 50

- Increment: 1

- Units: Hz

## RLL_RATE_FLTE: Roll axis rate controller error frequency in Hz

Roll axis rate controller error frequency in Hz

- Range: 2 50

- Increment: 1

- Units: Hz

## RLL_RATE_FLTD: Roll axis rate controller derivative frequency in Hz

Roll axis rate controller derivative frequency in Hz

- Range: 0 50

- Increment: 1

- Units: Hz

## RLL_RATE_SMAX: Roll slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5

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

## SERVO_AUTO_TRIM: Automatic servo trim

*Note: This parameter is for advanced users*

This enables automatic servo trim in flight. Servos will be trimed in stabilized flight modes when the aircraft is close to level. Changes to servo trim will be saved every 10 seconds and will persist between flights. The automatic trim won't go more than 20% away from a centered trim.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

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
|33|Motor1|
|34|Motor2|
|35|Motor3|
|36|Motor4|
|37|Motor5|
|38|Motor6|
|39|Motor7/TailTiltServo|
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
|73|ThrottleLeft|
|74|ThrottleRight|
|75|TiltMotorFrontLeft|
|76|TiltMotorFrontRight|
|77|ElevonLeft|
|78|ElevonRight|
|79|VTailLeft|
|80|VTailRight|
|82|Motor9|
|83|Motor10|
|84|Motor11|
|85|Motor12|
|86|DifferentialSpoilerLeft2|
|87|DifferentialSpoilerRight2|
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
|110|Airbrakes|
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

# SOAR Parameters

## SOAR_ENABLE: Is the soaring mode enabled or not

*Note: This parameter is for advanced users*

Toggles the soaring mode on and off

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## SOAR_VSPEED: Vertical v-speed

*Note: This parameter is for advanced users*

Rate of climb to trigger themalling speed

- Units: m/s

- Range: 0 10

## SOAR_Q1: Process noise

*Note: This parameter is for advanced users*

Standard deviation of noise in process for strength

- Range: 0.0001 0.01

## SOAR_Q2: Process noise

*Note: This parameter is for advanced users*

Standard deviation of noise in process for position and radius

- Range: 0.01 1

## SOAR_R: Measurement noise

*Note: This parameter is for advanced users*

Standard deviation of noise in measurement

- Range: 0.01 1

## SOAR_DIST_AHEAD: Distance to thermal center

*Note: This parameter is for advanced users*

Initial guess of the distance to the thermal center

- Units: m

- Range: 0 100

## SOAR_MIN_THML_S: Minimum thermalling time

*Note: This parameter is for advanced users*

Minimum number of seconds to spend thermalling

- Units: s

- Range: 0 600

## SOAR_MIN_CRSE_S: Minimum cruising time

*Note: This parameter is for advanced users*

Minimum number of seconds to spend cruising

- Units: s

- Range: 0 600

## SOAR_POLAR_CD0: Zero lift drag coef.

*Note: This parameter is for advanced users*

Zero lift drag coefficient

- Range: 0.005 0.5

## SOAR_POLAR_B: Induced drag coeffient

*Note: This parameter is for advanced users*

Induced drag coeffient

- Range: 0.005 0.05

## SOAR_POLAR_K: Cl factor

*Note: This parameter is for advanced users*

Cl factor 2*m*g/(rho*S)

- Units: m.m/s/s

- Range: 20 400

## SOAR_ALT_MAX: Maximum soaring altitude, relative to the home location

*Note: This parameter is for advanced users*

Don't thermal any higher than this.

- Units: m

- Range: 0 5000.0

## SOAR_ALT_MIN: Minimum soaring altitude, relative to the home location

*Note: This parameter is for advanced users*

Don't get any lower than this.

- Units: m

- Range: 0 1000.0

## SOAR_ALT_CUTOFF: Maximum power altitude, relative to the home location

*Note: This parameter is for advanced users*

Cut off throttle at this alt.

- Units: m

- Range: 0 5000.0

## SOAR_MAX_DRIFT: (Optional) Maximum drift distance to allow when thermalling.

*Note: This parameter is for advanced users*

The previous mode will be restored if the horizontal distance to the thermalling start location exceeds this value. -1 to disable.

- Range: 0 1000

## SOAR_MAX_RADIUS: (Optional) Maximum distance from home

*Note: This parameter is for advanced users*

RTL will be entered when a thermal is exited and the plane is more than this distance from home. -1 to disable.

- Range: 0 1000

## SOAR_THML_BANK: Thermalling bank angle

*Note: This parameter is for advanced users*

This parameter sets the bank angle to use when thermalling. Typically 30 - 45 degrees works well.

- Range: 20 50

- Units: deg

## SOAR_THML_ARSPD: Specific setting for airspeed when thermalling.

*Note: This parameter is for advanced users*

If non-zero this airspeed will be used when thermalling.

- Range: 5 50

## SOAR_CRSE_ARSPD: Specific setting for airspeed when cruising.

*Note: This parameter is for advanced users*

If non-zero this airspeed will be used when cruising. If set to -1, airspeed will be selected based on speed-to-fly theory.

- Range: 5 50

## SOAR_THML_FLAP: Flap percent to be used during thermalling flight.

*Note: This parameter is for advanced users*

This sets the flap when in LOITER with soaring active. Overrides the usual auto flap behaviour.

- Range: 0 100

# SRn Parameters

## SRn_RAW_SENS: Raw sensor stream rate

*Note: This parameter is for advanced users*

Raw sensor stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXT_STAT: Extended status stream rate to ground station

*Note: This parameter is for advanced users*

Extended status stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_RC_CHAN: RC Channel stream rate to ground station

*Note: This parameter is for advanced users*

RC Channel stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_RAW_CTRL: Raw Control stream rate to ground station

*Note: This parameter is for advanced users*

Raw Control stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_POSITION: Position stream rate to ground station

*Note: This parameter is for advanced users*

Position stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXTRA1: Extra data type 1 stream rate to ground station

*Note: This parameter is for advanced users*

Extra data type 1 stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXTRA2: Extra data type 2 stream rate to ground station

*Note: This parameter is for advanced users*

Extra data type 2 stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_EXTRA3: Extra data type 3 stream rate to ground station

*Note: This parameter is for advanced users*

Extra data type 3 stream rate to ground station

- Units: Hz

- Range: 0 50

- Increment: 1

- RebootRequired: True

## SRn_PARAMS: Parameter stream rate to ground station

*Note: This parameter is for advanced users*

Parameter stream rate to ground station

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

# STEER2SRV Parameters

## STEER2SRV_TCONST: Steering Time Constant

*Note: This parameter is for advanced users*

This controls the time constant in seconds from demanded to achieved steering angle. A value of 0.75 is a good default and will work with nearly all rovers. Ground steering in aircraft needs a bit smaller time constant, and a value of 0.5 is recommended for best ground handling in fixed wing aircraft. A value of 0.75 means that the controller will try to correct any deviation between the desired and actual steering angle in 0.75 seconds. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the vehicle can achieve.

- Range: 0.4 1.0

- Units: s

- Increment: 0.1

## STEER2SRV_P: Steering turning gain

The proportional gain for steering. This should be approximately equal to the diameter of the turning circle of the vehicle at low speed and maximum steering angle

- Range: 0.1 10.0

- Increment: 0.1

## STEER2SRV_I: Integrator Gain

This is the gain from the integral of steering angle. Increasing this gain causes the controller to trim out steady offsets due to an out of trim vehicle.

- Range: 0 1.0

- Increment: 0.05

## STEER2SRV_D: Damping Gain

This adjusts the damping of the steering control loop. This gain helps to reduce steering jitter with vibration. It should be increased in 0.01 increments as too high a value can lead to a high frequency steering oscillation that could overstress the vehicle.

- Range: 0 0.1

- Increment: 0.01

## STEER2SRV_IMAX: Integrator limit

*Note: This parameter is for advanced users*

This limits the number of degrees of steering in centi-degrees over which the integrator will operate. At the default setting of 1500 centi-degrees, the integrator will be limited to +- 15 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 1/3rd of the total control throw which is adequate unless the vehicle is severely out of trim.

- Range: 0 4500

- Increment: 10

- Units: cdeg

## STEER2SRV_MINSPD: Minimum speed

This is the minimum assumed ground speed in meters/second for steering. Having a minimum speed prevents oscillations when the vehicle first starts moving. The vehicle can still drive slower than this limit, but the steering calculations will be done based on this minimum speed.

- Range: 0 5

- Increment: 0.1

- Units: m/s

## STEER2SRV_FF: Steering feed forward

The feed forward gain for steering this is the ratio of the achieved turn rate to applied steering. A value of 1 means that the vehicle would yaw at a rate of 45 degrees per second with full steering deflection at 1m/s ground speed.

- Range: 0.0 10.0

- Increment: 0.1

## STEER2SRV_DRTSPD: Derating speed

*Note: This parameter is for advanced users*

Speed after that the maximum degree of steering will start to derate. Set this speed to a maximum speed that a plane can do controlled turn at maximum angle of steering wheel without rolling to wing. If 0 then no derating is used.

- Range: 0.0 30.0

- Increment: 0.1

- Units: m/s

## STEER2SRV_DRTFCT: Derating factor

*Note: This parameter is for advanced users*

Degrees of steering wheel to derate at each additional m/s of speed above "Derating speed". Should be set so that at higher speeds the plane does not roll to the wing in turns.

- Range: 0.0 50.0

- Increment: 0.1

- Units: deg/m/s

## STEER2SRV_DRTMIN: Minimum angle of wheel

*Note: This parameter is for advanced users*

The angle that limits smallest angle of steering wheel at maximum speed. Even if it should derate below, it will stop derating at this angle.

- Range: 0 4500

- Increment: 10

- Units: cdeg

# TECS Parameters

## TECS_CLMB_MAX: Maximum Climb Rate (metres/sec)

Maximum demanded climb rate. Do not set higher than the climb speed at THR_MAX at TRIM_ARSPD_CM when the battery is at low voltage. Reduce value if airspeed cannot be maintained on ascent. Increase value if throttle does not increase significantly to ascend.

- Increment: 0.1

- Range: 0.1 20.0

## TECS_SINK_MIN: Minimum Sink Rate (metres/sec)

Minimum sink rate when at THR_MIN and TRIM_ARSPD_CM.

- Increment: 0.1

- Range: 0.1 10.0

## TECS_TIME_CONST: Controller time constant (sec)

*Note: This parameter is for advanced users*

Time constant of the TECS control algorithm. Small values make faster altitude corrections but can cause overshoot and aggressive behavior.

- Range: 3.0 10.0

- Increment: 0.2

## TECS_THR_DAMP: Controller throttle damping

*Note: This parameter is for advanced users*

Damping gain for throttle demand loop. Increase to add throttle activity to dampen oscillations in speed and height.

- Range: 0.1 1.0

- Increment: 0.1

## TECS_INTEG_GAIN: Controller integrator

*Note: This parameter is for advanced users*

Integrator gain to trim out long-term speed and height errors.

- Range: 0.0 0.5

- Increment: 0.02

## TECS_VERT_ACC: Vertical Acceleration Limit (metres/sec^2)

*Note: This parameter is for advanced users*

Maximum vertical acceleration used to correct speed or height errors.

- Range: 1.0 10.0

- Increment: 0.5

## TECS_HGT_OMEGA: Height complementary filter frequency (radians/sec)

*Note: This parameter is for advanced users*

This is the cross-over frequency of the complementary filter used to fuse vertical acceleration and baro alt to obtain an estimate of height rate and height.

- Range: 1.0 5.0

- Increment: 0.05

## TECS_SPD_OMEGA: Speed complementary filter frequency (radians/sec)

*Note: This parameter is for advanced users*

This is the cross-over frequency of the complementary filter used to fuse longitudinal acceleration and airspeed to obtain a lower noise and lag estimate of airspeed.

- Range: 0.5 2.0

- Increment: 0.05

## TECS_RLL2THR: Bank angle compensation gain

*Note: This parameter is for advanced users*

Gain from bank angle to throttle to compensate for loss of airspeed from drag in turns. Set to approximately 10x the sink rate in m/s caused by a 45-degree turn. High efficiency models may need less while less efficient aircraft may need more. Should be tuned in an automatic mission with waypoints and turns greater than 90 degrees. Tune with PTCH2SRV_RLL and KFF_RDDRMIX to achieve constant airspeed, constant altitude turns.

- Range: 5.0 30.0

- Increment: 1.0

## TECS_SPDWEIGHT: Weighting applied to speed control

*Note: This parameter is for advanced users*

Mixing of pitch and throttle correction for height and airspeed errors. Pitch controls altitude and throttle controls airspeed if set to 0. Pitch controls airspeed and throttle controls altitude if set to 2 (good for gliders). Blended if set to 1.

- Range: 0.0 2.0

- Increment: 0.1

## TECS_PTCH_DAMP: Controller pitch damping

*Note: This parameter is for advanced users*

Damping gain for pitch control from TECS control.  Increasing may correct for oscillations in speed and height, but too much may cause additional oscillation and degraded control.

- Range: 0.1 1.0

- Increment: 0.1

## TECS_SINK_MAX: Maximum Descent Rate (metres/sec)

Maximum demanded descent rate. Do not set higher than the vertical speed the aircraft can maintain at THR_MIN, TECS_PITCH_MIN, and ARSPD_FBW_MAX.

- Increment: 0.1

- Range: 0.0 20.0

## TECS_LAND_ARSPD: Airspeed during landing approach (m/s)

When performing an autonomus landing, this value is used as the goal airspeed during approach.  Note that this parameter is not useful if your platform does not have an airspeed sensor (use TECS_LAND_THR instead).  If negative then this value is not used during landing.

- Range: -1 127

- Increment: 1

## TECS_LAND_THR: Cruise throttle during landing approach (percentage)

Use this parameter instead of LAND_ARSPD if your platform does not have an airspeed sensor.  It is the cruise throttle during landing approach.  If this value is negative then it is disabled and TECS_LAND_ARSPD is used instead.

- Range: -1 100

- Increment: 0.1

## TECS_LAND_SPDWGT: Weighting applied to speed control during landing.

*Note: This parameter is for advanced users*

Same as SPDWEIGHT parameter, with the exception that this parameter is applied during landing flight stages.  A value closer to 2 will result in the plane ignoring height error during landing and our experience has been that the plane will therefore keep the nose up -- sometimes good for a glider landing (with the side effect that you will likely glide a ways past the landing point).  A value closer to 0 results in the plane ignoring speed error -- use caution when lowering the value below 1 -- ignoring speed could result in a stall. Values between 0 and 2 are valid values for a fixed landing weight. When using -1 the weight will be scaled during the landing. At the start of the landing approach it starts with TECS_SPDWEIGHT and scales down to 0 by the time you reach the land point. Example: Halfway down the landing approach you'll effectively have a weight of TECS_SPDWEIGHT/2.

- Range: -1.0 2.0

- Increment: 0.1

## TECS_PITCH_MAX: Maximum pitch in auto flight

*Note: This parameter is for advanced users*

Overrides LIM_PITCH_MAX in automatic throttle modes to reduce climb rates. Uses LIM_PITCH_MAX if set to 0. For proper TECS tuning, set to the angle that the aircraft can climb at TRIM_ARSPD_CM and THR_MAX.

- Range: 0 45

- Increment: 1

## TECS_PITCH_MIN: Minimum pitch in auto flight

*Note: This parameter is for advanced users*

Overrides LIM_PITCH_MIN in automatic throttle modes to reduce descent rates. Uses LIM_PITCH_MIN if set to 0. For proper TECS tuning, set to the angle that the aircraft can descend at without overspeeding.

- Range: -45 0

- Increment: 1

## TECS_LAND_SINK: Sink rate for final landing stage

*Note: This parameter is for advanced users*

The sink rate in meters/second for the final stage of landing.

- Range: 0.0 2.0

- Increment: 0.1

## TECS_LAND_TCONST: Land controller time constant (sec)

*Note: This parameter is for advanced users*

This is the time constant of the TECS control algorithm when in final landing stage of flight. It should be smaller than TECS_TIME_CONST to allow for faster flare

- Range: 1.0 5.0

- Increment: 0.2

## TECS_LAND_DAMP: Controller sink rate to pitch gain during flare

*Note: This parameter is for advanced users*

This is the sink rate gain for the pitch demand loop when in final landing stage of flight. It should be larger than TECS_PTCH_DAMP to allow for better sink rate control during flare.

- Range: 0.1 1.0

- Increment: 0.1

## TECS_LAND_PMAX: Maximum pitch during final stage of landing

*Note: This parameter is for advanced users*

This limits the pitch used during the final stage of automatic landing. During the final landing stage most planes need to keep their pitch small to avoid stalling. A maximum of 10 degrees is usually good. A value of zero means to use the normal pitch limits.

- Range: -5 40

- Increment: 1

## TECS_APPR_SMAX: Sink rate max for landing approach stage

*Note: This parameter is for advanced users*

The sink rate max for the landing approach stage of landing. This will need to be large for steep landing approaches especially when using reverse thrust. If 0, then use TECS_SINK_MAX.

- Range: 0.0 20.0

- Units: m/s

- Increment: 0.1

## TECS_LAND_SRC: Land sink rate change

*Note: This parameter is for advanced users*

When zero, the flare sink rate (TECS_LAND_SINK) is a fixed sink demand. With this enabled the flare sinkrate will increase/decrease the flare sink demand as you get further beyond the LAND waypoint. Has no effect before the waypoint. This value is added to TECS_LAND_SINK proportional to distance traveled after wp. With an increasing sink rate you can still land in a given distance if you're traveling too fast and cruise passed the land point. A positive value will force the plane to land sooner proportional to distance passed land point. A negative number will tell the plane to slowly climb allowing for a pitched-up stall landing. Recommend 0.2 as initial value.

- Range: -2.0 2.0

- Units: m/s/m

- Increment: 0.1

## TECS_LAND_TDAMP: Controller throttle damping when landing

*Note: This parameter is for advanced users*

Damping gain for the throttle demand loop during an auto-landing. Same as TECS_THR_DAMP but only in effect during an auto-land. Increase to add throttle activity to dampen oscillations in speed and height. When set to 0 landing throttle damping is controlled by TECS_THR_DAMP.

- Range: 0.1 1.0

- Increment: 0.1

## TECS_LAND_IGAIN: Controller integrator during landing

*Note: This parameter is for advanced users*

This is the integrator gain on the control loop during landing. When set to 0 then TECS_INTEG_GAIN is used. Increase to increase the rate at which speed and height offsets are trimmed out. Typically values lower than TECS_INTEG_GAIN work best

- Range: 0.0 0.5

- Increment: 0.02

## TECS_TKOFF_IGAIN: Controller integrator during takeoff

*Note: This parameter is for advanced users*

This is the integrator gain on the control loop during takeoff. When set to 0 then TECS_INTEG_GAIN is used. Increase to increase the rate at which speed and height offsets are trimmed out. Typically values higher than TECS_INTEG_GAIN work best

- Range: 0.0 0.5

- Increment: 0.02

## TECS_LAND_PDAMP: Pitch damping gain when landing

*Note: This parameter is for advanced users*

This is the damping gain for the pitch demand loop during landing. Increase to add damping  to correct for oscillations in speed and height. If set to 0 then TECS_PTCH_DAMP will be used instead.

- Range: 0.1 1.0

- Increment: 0.1

## TECS_SYNAIRSPEED: Enable the use of synthetic airspeed

*Note: This parameter is for advanced users*

This enable the use of synthetic airspeed for aircraft that don't have a real airspeed sensor. This is useful for development testing where the user is aware of the considerable limitations of the synthetic airspeed system, such as very poor estimates when a wind estimate is not accurate. Do not enable this option unless you fully understand the limitations of a synthetic airspeed estimate.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## TECS_OPTIONS: Extra TECS options

*Note: This parameter is for advanced users*

This allows the enabling of special features in the speed/height controller

- Bitmask: 0:GliderOnly

## TECS_PTCH_FF_V0: Baseline airspeed for pitch feed-forward.

*Note: This parameter is for advanced users*

This parameter sets the airspeed at which no feed-forward is applied between demanded airspeed and pitch. It should correspond to the airspeed in metres per second at which the plane glides at neutral pitch including STAB_PITCH_DOWN.

- Range: 5.0 50.0

## TECS_PTCH_FF_K: Gain for pitch feed-forward.

*Note: This parameter is for advanced users*

This parameter sets the gain between demanded airspeed and pitch. It has units of radians per metre per second and should generally be negative. A good starting value is -0.04 for gliders and -0.08 for draggy airframes. The default (0.0) disables this feed-forward.

- Range: -5.0 0.0

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

# TKOFF Parameters

## TKOFF_ALT: Takeoff mode altitude

This is the target altitude for TAKEOFF mode

- Range: 0 200

- Increment: 1

- Units: m

## TKOFF_LVL_ALT: Takeoff mode altitude level altitude

This is the altitude below which wings are held level for TAKEOFF mode

- Range: 0 50

- Increment: 1

- Units: m

## TKOFF_LVL_PITCH: Takeoff mode altitude initial pitch

This is the target pitch for the initial climb to TKOFF_LVL_ALT

- Range: 0 30

- Increment: 1

- Units: deg

## TKOFF_DIST: Takeoff mode distance

This is the distance from the takeoff location where the plane will loiter. The loiter point will be in the direction of takeoff (the direction the plane is facing when the motor starts)

- Range: 0 500

- Increment: 1

- Units: m

# TUNE Parameters

## TUNE_PARAM: Transmitter tuning parameter or set of parameters

This sets which parameter or set of parameters will be tuned. Values greater than 100 indicate a set of parameters rather than a single parameter. Parameters less than 50 are for QuadPlane vertical lift motors only.

|Value|Meaning|
|:---:|:---:|
|0|None|
|1|RateRollPI|
|2|RateRollP|
|3|RateRollI|
|4|RateRollD|
|5|RatePitchPI|
|6|RatePitchP|
|7|RatePitchI|
|8|RatePitchD|
|9|RateYawPI|
|10|RateYawP|
|11|RateYawI|
|12|RateYawD|
|13|AngleRollP|
|14|AnglePitchP|
|15|AngleYawP|
|16|PosXYP|
|17|PosZP|
|18|VelXYP|
|19|VelXYI|
|20|VelZP|
|21|AccelZP|
|22|AccelZI|
|23|AccelZD|
|24|RatePitchFF|
|25|RateRollFF|
|26|RateYawFF|
|50|FixedWingRollP|
|51|FixedWingRollI|
|52|FixedWingRollD|
|53|FixedWingRollFF|
|54|FixedWingPitchP|
|55|FixedWingPitchI|
|56|FixedWingPitchD|
|57|FixedWingPitchFF|
|101|Set_RateRollPitch|
|102|Set_RateRoll|
|103|Set_RatePitch|
|104|Set_RateYaw|
|105|Set_AngleRollPitch|
|106|Set_VelXY|
|107|Set_AccelZ|
|108|Set_RatePitchDP|
|109|Set_RateRollDP|
|110|Set_RateYawDP|

## TUNE_CHAN: Transmitter tuning channel

This sets the channel for transmitter tuning. This should be connected to a knob or slider on your transmitter. It needs to be setup to use the PWM range given by TUNE_CHAN_MIN to TUNE_CHAN_MAX

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

## TUNE_CHAN_MIN: Transmitter tuning channel minimum pwm

This sets the PWM lower limit for the tuning channel

- Range: 900 2100

## TUNE_CHAN_MAX: Transmitter tuning channel maximum pwm

This sets the PWM upper limit for the tuning channel

- Range: 900 2100

## TUNE_SELECTOR: Transmitter tuning selector channel

This sets the channel for the transmitter tuning selector switch. This should be a 2 position switch, preferably spring loaded. A PWM above 1700 means high, below 1300 means low. If no selector is set then you won't be able to switch between parameters during flight or re-center the tuning knob

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Chan1|
|2|Chan3|
|3|Chan3|
|4|Chan4|
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

## TUNE_RANGE: Transmitter tuning range

This sets the range over which tuning will change a parameter. A value of 2 means the tuning parameter will go from 0.5 times the start value to 2x the start value over the range of the tuning channel

## TUNE_MODE_REVERT: Revert on mode change

This controls whether tuning values will revert on a flight mode change.

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## TUNE_ERR_THRESH: Controller error threshold

This sets the controller error threshold above which an alarm will sound and a message will be sent to the GCS to warn of controller instability while tuning. The error is the rms value of the P+D corrections in the loop. High values in hover indicate possible instability due to too high PID gains or excessively high D to P gain ratios.-1 will disable this message.

- Range: 0 1

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

# YAW Parameters

## YAW2SRV_SLIP: Sideslip control gain

*Note: This parameter is for advanced users*

Gain from lateral acceleration to demanded yaw rate for aircraft with enough fuselage area to detect lateral acceleration and sideslips. Do not enable for flying wings and gliders. Actively coordinates flight more than just yaw damping. Set after YAW2SRV_DAMP and YAW2SRV_INT are tuned.

- Range: 0 4

- Increment: 0.25

## YAW2SRV_INT: Sideslip control integrator

*Note: This parameter is for advanced users*

Integral gain from lateral acceleration error. Effectively trims rudder to eliminate long-term sideslip.

- Range: 0 2

- Increment: 0.25

## YAW2SRV_DAMP: Yaw damping

*Note: This parameter is for advanced users*

Gain from yaw rate to rudder. Most effective at yaw damping and should be tuned after KFF_RDDRMIX. Also disables YAW2SRV_INT if set to 0.

- Range: 0 2

- Increment: 0.25

## YAW2SRV_RLL: Yaw coordination gain

*Note: This parameter is for advanced users*

Gain to the yaw rate required to keep it consistent with the turn rate in a coordinated turn. Corrects for yaw tendencies after the turn is established. Increase yaw into the turn by raising. Increase yaw out of the turn by decreasing. Values outside of 0.9-1.1 range indicate airspeed calibration problems.

- Range: 0.8 1.2

- Increment: 0.05

## YAW2SRV_IMAX: Integrator limit

*Note: This parameter is for advanced users*

Limit of yaw integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 1500 allows trim of up to 1/3 of servo travel range.

- Range: 0 4500

- Increment: 1

## YAW_RATE_ENABLE: Yaw rate enable

*Note: This parameter is for advanced users*

Enable yaw rate controller for aerobatic flight

|Value|Meaning|
|:---:|:---:|
|0|Disable|
|1|Enable|

## YAW_RATE_P: Yaw axis rate controller P gain

Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output

- Range: 0.08 0.35

- Increment: 0.005

## YAW_RATE_I: Yaw axis rate controller I gain

Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate

- Range: 0.01 0.6

- Increment: 0.01

## YAW_RATE_IMAX: Yaw axis rate controller I gain maximum

Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output

- Range: 0 1

- Increment: 0.01

## YAW_RATE_D: Yaw axis rate controller D gain

Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate

- Range: 0.001 0.03

- Increment: 0.001

## YAW_RATE_FF: Yaw axis rate controller feed forward

Yaw axis rate controller feed forward

- Range: 0 3.0

- Increment: 0.001

## YAW_RATE_FLTT: Yaw axis rate controller target frequency in Hz

Yaw axis rate controller target frequency in Hz

- Range: 2 50

- Increment: 1

- Units: Hz

## YAW_RATE_FLTE: Yaw axis rate controller error frequency in Hz

Yaw axis rate controller error frequency in Hz

- Range: 2 50

- Increment: 1

- Units: Hz

## YAW_RATE_FLTD: Yaw axis rate controller derivative frequency in Hz

Yaw axis rate controller derivative frequency in Hz

- Range: 0 50

- Increment: 1

- Units: Hz

## YAW_RATE_SMAX: Yaw slew rate limit

*Note: This parameter is for advanced users*

Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.

- Range: 0 200

- Increment: 0.5