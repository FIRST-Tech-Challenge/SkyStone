package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.*;

/**
 * Created by Ethan on 12/2/2016.
 */

@TeleOp(name="Omni: TeleOp", group ="TeleOp")
public class OmniTeleOp extends OpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();

    public enum CapstoneState {
        ALIGN,
        GRAB,
        LIFT,
        RELEASE
    }


    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.setInputShaping(true);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    private CapstoneState capstoneState = CapstoneState.ALIGN;
    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private final double FOUNDATION_SPEED = 0.20;
    private final double FOUNDATION_SPIN = 0.20;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private int heightIncrement = 20;
    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean yHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean leftHeld = false;
    private boolean rightHeld = false;
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean a2Held = false;
    private boolean b2Held = false;
    private boolean y2Held = false;
    private boolean x2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean left2Held = false;
    private boolean right2Held = false;
    private boolean leftBumper2Held = false;
    private boolean rightBumper2Held = false;
    private boolean aPressed;
    private boolean bPressed;
    private boolean yPressed;
    private boolean leftPressed;
    private boolean rightPressed;
    private boolean leftBumperPressed;
    private boolean rightBumperPressed;
    private boolean a2Pressed;
    private boolean b2Pressed;
    private boolean y2Pressed;
    private boolean x2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private boolean left2Pressed;
    private boolean right2Pressed;
    private boolean leftBumper2Pressed;
    private boolean rightBumper2Pressed;
    private boolean fingersUp = true;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;
    private double liftPower;
    private double extendPower;
    private double collectPower;


    @Override
    public void start()
    {
    }

    @Override
    public void loop() {
        //left joystick is for moving
        //right joystick is for rotation
        gyroAngle = robot.readIMU();

        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;
        yPressed = gamepad1.y;
		rightPressed = gamepad1.dpad_right;
		leftPressed = gamepad1.dpad_left;
		leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;
        a2Pressed = gamepad2.a;
        b2Pressed = gamepad2.b;
        y2Pressed = gamepad2.y;
        x2Pressed = gamepad2.x;
        up2Pressed = gamepad2.dpad_up;
        down2Pressed = gamepad2.dpad_down;
        right2Pressed = gamepad2.dpad_right;
        left2Pressed = gamepad2.dpad_left;
        leftBumper2Pressed = gamepad2.left_bumper;
        rightBumper2Pressed = gamepad2.right_bumper;

		// Allow the robot to read encoders again
		robot.resetEncoderReads();

        if (gamepad1.x) {
            // The driver presses X, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as X is pressed, and will
            // not drive the robot using the left stick.  Once X is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            //robot.resetGyro();
            driverAngle = toDegrees(atan2(yPower, xPower)) - 90.0 - robot.readIMU();
            xPower = 0.0;
            yPower = 0.0;
            spin = 0.0;
        }

		// ********************************************************************
		// DRIVER JOYSTICK
		// ********************************************************************
        if(!bHeld && bPressed)
        {
            bHeld = true;
            robot.startReleasing();
        } else if(!bPressed) {
            bHeld = false;
        }

        if(!yHeld && yPressed)
        {
            if(fingersUp) {
                robot.fingersDown();
                fingersUp = false;
            } else {
                robot.fingersUp();
                fingersUp = true;
            }
            yHeld = true;
        } else if(!yPressed) {
            yHeld = false;
        }

        if(!aHeld && aPressed)
        {
            aHeld = true;
			if(robot.intakePower != 0.0) {
				robot.stopIntake();
			} else {
				robot.startIntake(false);
			}
        } else if(!aPressed) {
            aHeld = false;
        }

        if(!rightHeld && rightPressed)
        {
            rightHeld = true;
			robot.intakeOut();
        } else if(!rightPressed) {
            rightHeld = false;
        }

        if(!leftHeld && leftPressed)
        {
            leftHeld = true;
			robot.intakeIn();
        } else if(!leftPressed) {
            leftHeld = false;
        }

        if(!rightBumperHeld && rightBumperPressed)
        {
            robot.moveIntake(robot.intakeTargetPosition);
            rightBumperHeld = true;
        } else if(!rightBumperPressed) {
            rightBumperHeld = false;
        }

        if(!leftBumperHeld && leftBumperPressed)
        {
			if(speedMultiplier == MAX_SPEED) {
				speedMultiplier = FOUNDATION_SPEED;
				spinMultiplier = FOUNDATION_SPIN;
			} else {
				speedMultiplier = MAX_SPEED;
				spinMultiplier = MAX_SPIN;
			}
            leftBumperHeld = true;
        } else if(!leftBumperPressed) {
            leftBumperHeld = false;
        }

		// ********************************************************************
		// OPERATOR JOYSTICK
		// ********************************************************************
		// This was unassigned (fingers up/down)
        if(!x2Held && x2Pressed)
        {
            robot.startEjecting();
            x2Held = true;
        } else if(!x2Pressed) {
            x2Held = false;
        }

        if(!a2Held && a2Pressed)
        {
            a2Held = true;
            robot.startLifting();
        } else if(!a2Pressed) {
            a2Held = false;
        }

        if(!b2Held && b2Pressed)
        {
            b2Held = true;
            robot.startStowing();
        } else if(!b2Pressed) {
            b2Held = false;
        }

        if(!y2Held && y2Pressed)
        {
            y2Held = true;
            switch(capstoneState) {
                case ALIGN:
                    robot.startAligningCapstone();
                    capstoneState = CapstoneState.GRAB;
                    break;
                case GRAB:
                    robot.startGrabbingCapstone();
                    capstoneState = CapstoneState.LIFT;
                    break;
                case LIFT:
                    robot.startLiftingCapstone();
                    capstoneState = CapstoneState.RELEASE;
                    break;
                case RELEASE:
                    robot.startReleasingCapstone();
                    capstoneState = CapstoneState.ALIGN;
                    break;
            }
        } else if(!y2Pressed) {
            y2Held = false;
        }

        if(!up2Held && up2Pressed)
        {
            up2Held = true;
            robot.addStone();
        } else if (!up2Pressed) {
			up2Held = false;
		}

        if(!down2Held && down2Pressed)
        {
            down2Held = true;
            robot.removeStone();
        } else if (!down2Pressed) {
			down2Held = false;
		}

        if(!rightBumper2Held && rightBumper2Pressed)
        {
            int newHeight = robot.getLifterPosition();
            newHeight += heightIncrement;
            robot.lifter.setTargetPosition(newHeight);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(1.0);
            rightBumper2Held = true;
        } else if(!rightBumper2Pressed) {
            rightBumper2Held = false;
        }

        if(!leftBumper2Held && leftBumper2Pressed)
        {
            int newHeight = robot.getLifterPosition();
            newHeight -= heightIncrement;
            robot.lifter.setTargetPosition(newHeight);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(1.0);
            leftBumper2Held = true;
        } else if(!leftBumper2Pressed) {
            leftBumper2Held = false;
        }


        // If the activity is not performing, it will be idle and return.
        robot.performLifting();
        robot.performReleasing();
        robot.performStowing();
        robot.performEjecting();
        robot.performAligningCapstone();
        robot.performGrabbingCapstone();
        robot.performLiftingCapstone();
        robot.performReleasingCapstone();

        robot.drive(speedMultiplier * xPower, speedMultiplier * yPower, spinMultiplier * spin, driverAngle);

		telemetry.addData("Lift Target Height: ", robot.liftTargetHeight);
        telemetry.addData("Intake Target: ", robot.intakeTargetPosition);
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Lift State: ", robot.liftState);
        telemetry.addData("Release State: ", robot.releaseState);
        telemetry.addData("Stow State: ", robot.stowState);
        telemetry.addData("Eject State: ", robot.ejectState);
        telemetry.addData("Capstone State: ", robot.capstoneLiftState);
        telemetry.addData("Front Left Encoder: ", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Encoder: ", robot.frontRight.getCurrentPosition());
        telemetry.addData("Rear Left Encoder: ", robot.rearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Encoder: ", robot.rearRight.getCurrentPosition());
        telemetry.addData("Lifter Encoder: ", robot.getLifterPosition());
        telemetry.addData("Intake Encoder: ", robot.getIntakePosition());
        telemetry.addData("Intake Zero: ", robot.intakeZero);
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
        robot.stopGroundEffects();
    }
}
