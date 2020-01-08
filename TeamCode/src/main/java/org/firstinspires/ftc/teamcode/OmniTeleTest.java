package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;

/**
 * Created by Ethan on 12/2/2016.
 */

@TeleOp(name="Omni: TeleOpTest", group ="TeleOp")
public class OmniTeleTest extends OmniAutoClass {

    public OmniTeleTest() {
        msStuckDetectInit = 10000;
    }

    public HardwareOmnibot robot = new HardwareOmnibot();

//    @Override
    public void initRobot() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
		// Turn off the SDK deadzone so we can do it ourselves
        gamepad1.setJoystickDeadzone(0.0f);
        gamepad2.setJoystickDeadzone(0.0f);
        robot.init(hardwareMap);
        robot.setInputShaping(true);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean yHeld = false;
    private boolean xHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean rightHeld = false;
    private boolean leftHeld = false;
    private boolean rightBumperHeld = false;
    private boolean leftBumperHeld = false;
    private boolean a2Held = false;
    private boolean b2Held = false;
    private boolean y2Held = false;
    private boolean x2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean aPressed;
    private boolean bPressed;
    private boolean yPressed;
    private boolean xPressed;
    private boolean upPressed;
    private boolean downPressed;
    private boolean rightPressed;
    private boolean leftPressed;
    private boolean rightBumperPressed;
    private boolean leftBumperPressed;
    private boolean a2Pressed;
    private boolean b2Pressed;
    private boolean y2Pressed;
    private boolean x2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;
    private double liftPower;
    private double extendPower;
    private double collectPower;
    private boolean reverse = false;
    private boolean spinning = false;
    private boolean doneExtending = true;
    private boolean clawOpen = true;
    private boolean clawFront = true;
    private double leftTof;
    private double rightTof;
    private double backLeftTof;
    private double backRightTof;
    private double backTof;
    private double sideDistanceTarget = 30;
    private double backDistanceTarget = 30;
    private double clawPosition = 0;
    private double clawdricopterPosition = 0;

//    @Override
//    public void start()
//    {
//    }

//    @Override
//    public void loop() {
    @Override
    public void runOpMode() {
        initRobot();
        setupRobotParameters(4.0, 19.2);
        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive()) {
            //left joystick is for moving
            //right joystick is for rotation
            gyroAngle = robot.readIMU();

            yPower = -HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_y);
            xPower = HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_x);
            spin = HardwareOmnibot.cleanMotionValues(gamepad1.right_stick_x);
            aPressed = gamepad1.a;
            bPressed = gamepad1.b;
            yPressed = gamepad1.y;
            xPressed = gamepad1.x;
            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;
            rightPressed = gamepad1.dpad_right;
            leftPressed = gamepad1.dpad_left;
            rightBumperPressed = gamepad1.right_bumper;
            leftBumperPressed = gamepad1.left_bumper;
            a2Pressed = gamepad2.a;
            b2Pressed = gamepad2.b;
            y2Pressed = gamepad2.y;
            x2Pressed = gamepad2.x;
            up2Pressed = gamepad2.dpad_up;
            down2Pressed = gamepad2.dpad_down;

            // Allow the robot to read encoders again
            robot.resetReads();


            if (!xHeld && xPressed) {
                xHeld = true;
            } else if (!xPressed) {
                xHeld = false;
            }

            if (!rightBumperHeld && rightBumperPressed) {
                rightBumperHeld = true;
                robot.startExtendingIntake();
            } else if (!rightBumperPressed) {
                rightBumperHeld = false;
            }

            if (!leftBumperHeld && leftBumperPressed) {
                leftBumperHeld = true;
                robot.extender.setPower(-1.0);
            } else if (!leftBumperPressed) {
                leftBumperHeld = false;
                robot.extender.setPower(0.0);
            }

            if (!aHeld && aPressed) {
                robot.clawdricopter.setPosition(robot.CLAWDRICOPTER_FRONT);
                aHeld = true;

            } else if (!aPressed) {
                aHeld = false;
            }

            if (!bHeld && bPressed) {
                bHeld = true;
            } else if (!bPressed) {
                bHeld = false;
            }

            if (!yHeld && yPressed) {
                yHeld = true;
            } else if (!yPressed) {
                yHeld = false;
            }

            if (!upHeld && upPressed) {
                upHeld = true;
            } else if (!upPressed) {
                upHeld = false;
            }

            if (!downHeld && downPressed) {
                downHeld = true;
            } else if (!downPressed) {
                downHeld = false;
            }

            if (!rightHeld && rightPressed) {
                rightHeld = true;
            } else if (!rightPressed) {
                rightHeld = false;
            }

            if (!leftHeld && leftPressed) {
                leftHeld = true;
            } else if (!leftPressed) {
                leftHeld = false;
            }

            robot.performExtendingIntake();

            if(robot.stackStone != HardwareOmnibot.StackActivities.IDLE) {
                robot.drive(speedMultiplier * xPower, speedMultiplier * yPower, spinMultiplier * spin, driverAngle, robot.defaultInputShaping);
            }

//        if(!a2Held && a2Pressed)
//        {
//            a2Held = true;
//            if(robot.frontLeftMotorPower > 0.5) {
//                robot.setFrontLeftMotorPower(0.0);
//            } else {
//                robot.setFrontLeftMotorPower(1.0);
//            }
//        } else if(!a2Pressed) {
//            a2Held = false;
//        }

//        if(!b2Held && b2Pressed)
//        {
//            b2Held = true;
//            if(robot.frontRightMotorPower > 0.5) {
//                robot.setFrontRightMotorPower(0.0);
//            } else {
//                robot.setFrontRightMotorPower(1.0);
//            }
//        } else if(!b2Pressed) {
//            b2Held = false;
//        }

//        if(!y2Held && y2Pressed)
//        {
//            y2Held = true;
//            if(robot.rearLeftMotorPower > 0.5) {
//                robot.setRearLeftMotorPower(0.0);
//            } else {
//                robot.setRearLeftMotorPower(1.0);
//            }
//        } else if(!y2Pressed) {
//            y2Held = false;
//        }

//        if(!x2Held && x2Pressed)
//        {
//            x2Held = true;
//            robot.runLift(robot.liftTargetHeight);
//        } else if(!x2Pressed) {
//            x2Held = false;
//        }

//        if(!up2Held && up2Pressed)
//        {
//            up2Held = true;
//            robot.addStone();
//        } else if (!up2Pressed) {
//			up2Held = false;
//		}

//        if(!down2Held && down2Pressed)
//        {
//            down2Held = true;
//            robot.removeStone();
//        } else if (!down2Pressed) {
//			down2Held = false;
//		}

//        if(Math.abs(xPower) > 0.1) {
//            robot.manualExtendIntake(xPower);
//        } else {
//            robot.manualExtendIntake(0.0);
//        }

//        if(Math.abs(spin) > 0.1) {
//            robot.manualLift(spin);
//        } else {
//            robot.manualLift(0.0);
//        }
            // If the activity is not performing, it will be idle and return.
//        robot.performLifting();
//        robot.performReleasing();
//        robot.performStowing();


            //telemetry.addData("Left TOF: ", leftTof);
            //telemetry.addData("Right TOF: ", rightTof);
            //telemetry.addData("Back TOF: ", backTof);
            telemetry.addData("Side Target Distance: ", sideDistanceTarget);
            telemetry.addData("Back Target Distance: ", backDistanceTarget);
            telemetry.addData("Back Left TOF: ", backLeftTof);
            telemetry.addData("Back Right TOF: ", backRightTof);
            telemetry.addData("Lift Target Height: ", robot.liftTargetHeight.toString());
            telemetry.addData("Lift State: ", robot.liftState);
            telemetry.addData("Release State: ", robot.releaseState);
            telemetry.addData("Stow State: ", robot.stowState);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Spin: ", spin);
            telemetry.addData("Offset Angle: ", driverAngle);
            telemetry.addData("Gyro Angle: ", gyroAngle);
            telemetry.addData("Gyro X Angle: ", robot.xAngle);
            telemetry.addData("Gyro Y Angle: ", robot.yAngle);
            telemetry.addData("Gyro Z Angle: ", robot.zAngle);
            telemetry.addData("Front Left Encoder: ", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Encoder: ", robot.frontRight.getCurrentPosition());
            telemetry.addData("Rear Left Encoder: ", robot.rearLeft.getCurrentPosition());
            telemetry.addData("Rear Right Encoder: ", robot.rearRight.getCurrentPosition());
            telemetry.addData("Lifter Encoder: ", robot.lifter.getCurrentPosition());
            telemetry.addData("Extender Encoder: ", robot.extender.getCurrentPosition());
            telemetry.addData("Claw Front: ", clawFront);
            telemetry.addData("Claw Open: ", clawOpen);
            telemetry.addData("Claw Position: ", clawPosition);
            telemetry.addData("Clawdricopter Position: ", clawdricopterPosition);
            updateTelemetry(telemetry);
        }
    }

//    @Override
//    public void stop() {
//        robot.stopGroundEffects();
//    }
}
