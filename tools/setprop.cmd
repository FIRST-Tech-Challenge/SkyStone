adb shell setprop debug.ftc.logging 1
adb shell setprop debug.ftc.enable_arm 0
adb shell setprop debug.ftc.skystonePos 1
adb shell setprop debug.ftc.pause 5
adb shell setprop debug.ftc.bulk 1
adb shell setprop debug.ftc.imu 1
adb shell setprop debug.ftc.resetfollow 1
adb shell setprop debug.ftc.imuInterval 10
adb shell setprop debug.ftc.odom 0
adb shell setprop debug.ftc.vuforia 0
adb shell setprop debug.ftc.brake 0
adb shell setprop debug.ftc.recreateDrv 0
adb shell setprop debug.ftc.drvCorrect 0
rem (-24, 24) for strafe diagonal
adb shell setprop debug.ftc.distance 72
adb shell setprop debug.ftc.distance0 48
adb shell setprop debug.ftc.strafeDiag 1
adb shell setprop debug.ftc.trackwidth 14.2
adb shell setprop debug.ftc.maxVel 70.0
adb shell setprop debug.ftc.maxAccel 35.0
adb shell setprop debug.ftc.strafeMaxVel 70.0
adb shell setprop debug.ftc.strafeMaxAccel 35.0
rem smaller does make straight test go shorter distance;
adb shell setprop debug.ftc.kV 0.0111
rem 0.0111 for 4 wheel
adb shell setprop debug.ftc.kP 1.72
adb shell setprop debug.ftc.kI 0.172
adb shell setprop debug.ftc.kD 0.0
rem drivetrain paramters;
adb shell setprop debug.ftc.txP 5.0
adb shell setprop debug.ftc.txI 0.5
adb shell setprop debug.ftc.txD 0.00001
adb shell setprop debug.ftc.tyP 5.0
adb shell setprop debug.ftc.tyI 10.0
adb shell setprop debug.ftc.tyD 0.00001
adb shell setprop debug.ftc.hP 10.0
adb shell setprop debug.ftc.hI 0.5
adb shell setprop debug.ftc.hD 0.00001
rem strafing paramters ---------------------
adb shell setprop debug.ftc.stxP 20
adb shell setprop debug.ftc.stxI 1
adb shell setprop debug.ftc.stxD 0.75
adb shell setprop debug.ftc.styP 15
adb shell setprop debug.ftc.styI 0.5
adb shell setprop debug.ftc.styD 1
adb shell setprop debug.ftc.shP 6
adb shell setprop debug.ftc.shI 2
adb shell setprop debug.ftc.shD 0.4
rem time in seconds needed per inch;
adb shell setprop debug.ftc.strafeTimeDistanceRat 0.093
adb shell setprop debug.ftc.strafeMotorPower 0.19
adb shell setprop debug.ftc.rear_ratio 1.105
adb shell setprop debug.ftc.odoTicksPerRev 1565
adb shell setprop debug.ftc.odomTrackwidth 14.8
adb shell setprop debug.ftc.odomForwardOffset -5.5
adb shell getprop |grep debug.ftc
rem adb logcat -s VrApi
IF "%1"=="1" (
exit
)
