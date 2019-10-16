package org.firstinspires.ftc.teamcode.All;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import java.util.ArrayList;

public class HardwareMap {
    public DcMotorEx backLeft, backRight, frontLeft, frontRight, firstJoint, secondJoint, intakeJoint;
    public CRServo intake;
    public BNO055IMU gyro;
    public IntegratingGyroscope imu;
    public static AnalogInput leftForward, rightForward, sideways;
    public static String TAG = "MainThread";

    public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;

    public HardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        backLeft = (DcMotorEx) hwMap.get(DcMotor.class, "backLeft");
        backRight = (DcMotorEx) hwMap.get(DcMotor.class, "backRight");
        frontLeft = (DcMotorEx) hwMap.get(DcMotor.class, "frontLeft");
        frontRight = (DcMotorEx) hwMap.get(DcMotor.class, "frontRight");

        leftForward = hwMap.get(AnalogInput.class, "leftForward");
        rightForward = hwMap.get(AnalogInput.class, "rightForward");
        sideways = hwMap.get(AnalogInput.class, "sideways");
        gyro = hwMap.get(BNO055IMU.class, "imu");
        imu = (IntegratingGyroscope) gyro;


        //firstJointVirtualServo = new MotorServo(firstJoint, MotorServo.MotorConfiguration.firstJoint);
        //secondJointVirtualServo = new MotorServo(secondJoint, MotorServo.MotorConfiguration.secondJoint);

        this.hardwareMap = hwMap;
    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        gyro.initialize(parameters);
    }

    //--------------------------------==================\/\/Track Encoders\/\/==================---------------------------------

    public static class track extends Thread {  //Encoders: Min = 0.0, Max = 3.3
        static double numOfLeftLoops = 0;
        static double numOfRightLoops = 0;
        static double numOfSideLoops = 0;
        static boolean tracking = false;
        static double startTime = -1;
        static double elapsedTime = -1;
        static int delay = 25;
        static int readDelay = 15;

        private static void beginTracking() {        //Left & Side--Encoders: Forward = 3 - 2 - 1 - 0 - 3 - 2..., Backward = 0 - 1 - 2 - 3 - 0 - 1...
            Thread left = new Thread() {      //Right--Encoders: Forward = 0 - 1 - 2 - 3 - 0 - 1..., Backward = 3 - 2 - 1 - 0 - 3 - 2...
                public void run() {
                    if (tracking && startTime == -1)
                        startTime = System.nanoTime();
                    while (tracking) {
                        double left = leftForward.getVoltage();

                        if (left < 1) {
                            try {
                                Thread.sleep(readDelay);
                            } catch (Exception e) {
                            }

                            double leftDelta = Math.abs(left - leftForward.getVoltage());

                            if (leftDelta >= 2.7)
                                numOfLeftLoops += 1;
                        } else if (left > 2.3) {
                            try {
                                Thread.sleep(readDelay);
                            } catch (Exception e) {
                            }

                            double leftDelta = Math.abs(left - leftForward.getVoltage());

                            if (leftDelta >= 2.7)
                                numOfLeftLoops -= 1;
                        }

                        try {
                            Thread.sleep(delay);
                        } catch (Exception e) {
                        }
                    }
                    elapsedTime = System.nanoTime() - startTime;
                }
            };

            Thread right = new Thread() {      //Right--Encoders: Forward = 0 - 1 - 2 - 3 - 0 - 1..., Backward = 3 - 2 - 1 - 0 - 3 - 2...
                public void run() {
                    while (tracking) {
                        double right = rightForward.getVoltage();

                        if (right > 1) {
                            try {
                                Thread.sleep(readDelay);
                            } catch (Exception e) {
                            }

                            double rightDelta = Math.abs(right - rightForward.getVoltage());

                            if (rightDelta >= 2.3)
                                numOfRightLoops -= 1;
                        } else if (right < 0.5) {
                            try {
                                Thread.sleep(readDelay);
                            } catch (Exception e) {
                            }

                            double rightDelta = Math.abs(right - rightForward.getVoltage());

                            if (rightDelta >= 2.7)
                                numOfRightLoops += 1;
                        }

                        try {
                            Thread.sleep(delay);
                        } catch (Exception e) {
                        }
                    }
                }
            };

            Thread side = new Thread() {      //Right--Encoders: Forward = 0 - 1 - 2 - 3 - 0 - 1..., Backward = 3 - 2 - 1 - 0 - 3 - 2...
                public void run() {
                    while (tracking) {
                        double side = sideways.getVoltage();

                        if (side < 1) {
                            try {
                                Thread.sleep(readDelay);
                            } catch (Exception e) {
                            }

                            double sideDelta = Math.abs(side - sideways.getVoltage());

                            if (sideDelta >= 2.7)
                                numOfSideLoops += 1;
                        } else if (side > 2.3) {
                            try {
                                Thread.sleep(readDelay);
                            } catch (Exception e) {
                            }

                            double sideDelta = Math.abs(side - sideways.getVoltage());

                            if (sideDelta >= 2.7)
                                numOfSideLoops -= 1;
                        }

                        try {
                            Thread.sleep(delay);
                        } catch (Exception e) {
                        }
                    }
                }
            };
            left.start();
            right.start();
            side.start();
        }

        public static ArrayList<Double> getEncoderTicks() {  //Returns ArrayList [leftTicks, rightTicks, sideTicks] in cm
            ArrayList<Double> totalTicks = new ArrayList<>();
            double addLeftVal = 3.3;
            double addRightVal = 3.3;
            double addSideVal = 3.3;
            double lVal = numOfLeftLoops;
            double rVal = numOfRightLoops;
            double sVal = numOfSideLoops;

            if (numOfLeftLoops < 0) {
                addLeftVal = 0;
                lVal += 1;
            }
            if (numOfRightLoops < 0) {
                addRightVal = 0;
                rVal += 1;
            }
            if (numOfSideLoops < 0) {
                addSideVal = 0;
                sVal += 1;
            }

            totalTicks.add((addLeftVal - leftForward.getVoltage() + 3.3 * lVal) /*/
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION * DriveConstant.LEFT_GEAR_RATIO *
                    (2 * Math.PI * DriveConstant.ODOMETRY_RAD) * (DriveConstant.ODOMETRY_RAD / DriveConstant.MECANUM_RAD)*/);
            totalTicks.add((addRightVal - rightForward.getVoltage() + 3.3 * rVal) /*/
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION * DriveConstant.RIGHT_GEAR_RATIO *
                    (2 * Math.PI * DriveConstant.ODOMETRY_RAD) * (DriveConstant.ODOMETRY_RAD / DriveConstant.MECANUM_RAD)*/);
            totalTicks.add((addSideVal - sideways.getVoltage() + 3.3 * sVal) /*/
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION * DriveConstant.SIDE_GEAR_RATIO *
                    (2 * Math.PI * DriveConstant.ODOMETRY_RAD) * (DriveConstant.ODOMETRY_RAD / DriveConstant.MECANUM_RAD)*/);
            return totalTicks;
        }

        public static ArrayList<String> getEncoderDebug() {  //Debugs the entire formula for calculating encoder ticks
            ArrayList<String> debug = new ArrayList<>();
            double addLeftVal = 3.3;
            double addRightVal = 3.3;
            double addSideVal = 3.3;

            double lVal = numOfLeftLoops;
            double rVal = numOfRightLoops;
            double sVal = numOfSideLoops;

            if (numOfLeftLoops + 1 < 0) {
                addLeftVal = 0;
                lVal += 1;
            }
            if (numOfRightLoops + 1 < 0) {
                addRightVal = 0;
                rVal += 1;
            }
            if (numOfSideLoops + 1 < 0) {
                addSideVal = 0;
                sVal += 1;
            }

            debug.add("(ignoreLoopsAdditionalValue - getVoltage + 3.3 * numOfLoops) / VOLTS_PER_REVOLUTION * GEAR_RATIO * " +
                    "(2 * PI * ODOMETRY_RADIUS) * (ODOMETRY_RADIUS / MECANUM_RADIUS)");
            debug.add("(" + addLeftVal + " - " + leftForward.getVoltage() + " + " + 3.3 + " * " + lVal + ")" /*+ " / " +
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION + " * " + DriveConstant.LEFT_GEAR_RATIO + " * " +
                    "(" + 2 + " * " + Math.PI + " * " + DriveConstant.ODOMETRY_RAD + ")" + " * " +
                    "(" + DriveConstant.ODOMETRY_RAD + " / " + DriveConstant.MECANUM_RAD + ")"*/);
            debug.add("(" + addRightVal + " - " + rightForward.getVoltage() + " + " + 3.3 + " * " + rVal + ")" /*+ " / " +
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION + " * " + DriveConstant.RIGHT_GEAR_RATIO + " * " +
                    "(" + 2 + " * " + Math.PI + " * " + DriveConstant.ODOMETRY_RAD + ")" + " * " +
                    "(" + DriveConstant.ODOMETRY_RAD + " / " + DriveConstant.MECANUM_RAD + ")"*/);
            debug.add("(" + addSideVal + " - " + sideways.getVoltage() + " + " + 3.3 + " * " + sVal + ")" /*+ " / " +
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION + " * " + DriveConstant.SIDE_GEAR_RATIO + " * " +
                    "(" + 2 + " * " + Math.PI + " * " + DriveConstant.ODOMETRY_RAD + ")" + " * " +
                    "(" + DriveConstant.ODOMETRY_RAD + " / " + DriveConstant.MECANUM_RAD + ")"*/);
            return debug;
        }

        public static ArrayList<String> getElapsedTime() {   //Returns [elapsedTime, CurrentlyTracking]
            ArrayList<String> time = new ArrayList<>();
            if (tracking) {
                time.add(String.valueOf(System.nanoTime() - startTime));
                time.add("Currently Tracking: YES");
            } else {
                time.add(String.valueOf(elapsedTime));
                time.add("Currently Tracking: NO");
            }
            return time;
        }

        public static void resetEncoders() {     //Resets everything to 0 including elapsedTime
            leftForward.resetDeviceConfigurationForOpMode();
            rightForward.resetDeviceConfigurationForOpMode();
            sideways.resetDeviceConfigurationForOpMode();
            numOfLeftLoops = 0;
            numOfRightLoops = 0;
            numOfSideLoops = 0;
            startTime = -1;
        }

        public static void encoders(boolean track, int readingDelay, int loopDelay) {
            tracking = track;

            if (readingDelay >= 0)
                readDelay = loopDelay;
            else
                readDelay = 15;

            //if (readingDelay >= 0 && readingDelay < 5)
            //    readDelay = 5;


            if (loopDelay >= 0)
                delay = loopDelay;
            else
                delay = 25;

            //if (loopDelay >= 0 && loopDelay < 5)
            //    delay = 5;

            if (track)
                beginTracking();
        }

        public static void encoders(boolean track) {
            tracking = track;
            readDelay = 15;
            delay = 25;
            if (track)
                beginTracking();
        }
    }

    //----------------------------==================/\/\End Track Encoders/\/\==================-----------------------------
}
