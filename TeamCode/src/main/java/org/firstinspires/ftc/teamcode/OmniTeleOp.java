package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.*;

/**
 * Created by Ethan on 12/2/2016.
 */

@TeleOp(name="Omni: TeleOp", group ="TeleOp")
public class OmniTeleOp extends OpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.setInputShaping(true);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    private final static int NOTHING = 0;
    private final static int BALL = 1;
    private final static int CUBE = 2;
    private boolean leftHasElement = false;
    private boolean rightHasElement = false;
    private int leftElement = NOTHING;
    private int rightElement = NOTHING;
    private float leftH = 0F;
    private float rightH = 0F;
    private float leftRed = 0;
    private float leftGreen = 0;
    private float leftBlue = 0;
    private float rightRed = 0;
    private float rightGreen = 0;
    private float rightBlue = 0;
    private float leftHueBase = 144.0F;
    private float rightHueBase = 140.0F;
    private float leftHueRoomCorrection = 0.0f;
    private float rightHueRoomCorrection = 0.0f;

    private double driverAngle = 0.0;

    private boolean raisingLifter = false;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean autoCollectRight = false;
    private boolean autoCollectLeft = false;
    private final double COLLECTOR_SPEED = 0.6;
    private final double ELEMENT_PRESENT = 10.0;
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    final private int NO_ELEMENT_COLOR = 0x00000000;
    final private int SILVER_ELEMENT_COLOR = 0xC0C0C0;
    final private int GOLD_ELEMENT_COLOR = 0xFFD700;
    private int leftColor = 0x00000000;
    private int rightColor = 0x00000000;
    private boolean updateLeds = false;

    @Override
    public void start()
    {
        float leftHsvValues[] = {0F, 0F, 0F};
//        leftRed = robot.sensorColorLeft.red();
//        leftGreen = robot.sensorColorLeft.green();
//        leftBlue = robot.sensorColorLeft.blue();
        Color.RGBToHSV((int) (leftRed * SCALE_FACTOR),
                (int) (leftGreen * SCALE_FACTOR),
                (int) (leftBlue * SCALE_FACTOR),
                leftHsvValues);
        leftHueRoomCorrection = leftHueBase - leftHsvValues[0];

        float rightHsvValues[] = {0F, 0F, 0F};
//        rightRed = robot.sensorColorRight.red();
//        rightGreen = robot.sensorColorRight.green();
//        rightBlue = robot.sensorColorRight.blue();
        Color.RGBToHSV((int) (rightRed * SCALE_FACTOR),
                (int) (rightGreen * SCALE_FACTOR),
                (int) (rightBlue * SCALE_FACTOR),
                rightHsvValues);
        rightHueRoomCorrection = rightHueBase - rightHsvValues[0];
        // Executes when start is pressed before loop starts getting called
        // Might want to reset the Gyro here.
        //robot.resetGyro();
    }

    @Override
    public void loop() {
        //left joystick is for moving
        //right joystick is for rotation
        boolean extenderOverride = false;
        double yPower;
        double xPower;
        double spin;
        double liftPower;
        double gyroAngle = robot.readIMU();
        double extendPower;
        double rotatePower;
        double leftCollectorPower;
        double rightCollectorPower;
        int lifterEncoder = 0;
        int rotatorEncoder = 0;
        int extenderEncoder = 0;

        extendPower = -gamepad2.left_stick_y;
        rotatePower = gamepad2.right_stick_y;
        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        extenderOverride = gamepad2.y;

        // Sets the robot to collect until it detects an element.
        if(gamepad2.a) {
            leftColor = NO_ELEMENT_COLOR;
            leftElement = NOTHING;
            leftH = 0F;
            leftRed = 0F;
            leftGreen = 0F;
            leftBlue = 0F;
            rightColor = NO_ELEMENT_COLOR;
            rightElement = NOTHING;
            rightH = 0F;
            rightRed = 0F;
            rightGreen = 0F;
            rightBlue = 0F;
            autoCollectRight = true;
            autoCollectLeft = true;
            rightHasElement = false;
            leftHasElement = false;
            updateLeds = true;
        }

        if(gamepad2.right_bumper)
        {
            autoCollectRight = false;
            rightCollectorPower = -COLLECTOR_SPEED;
        } else if(gamepad2.right_trigger > 0.1) {
            autoCollectRight = false;
            rightCollectorPower = COLLECTOR_SPEED;
        } else
        {
            if(!autoCollectRight) {
                rightCollectorPower = 0.0;
            } else {
                rightCollectorPower = COLLECTOR_SPEED;
            }
        }
        if(gamepad2.left_bumper)
        {
            autoCollectLeft = false;
            leftCollectorPower = -COLLECTOR_SPEED;
        } else if(gamepad2.left_trigger > 0.1) {
            autoCollectLeft = false;
            leftCollectorPower = COLLECTOR_SPEED;
        } else
        {
            if(!autoCollectLeft) {
                leftCollectorPower = 0.0;
            } else {
                leftCollectorPower = COLLECTOR_SPEED;
            }
        }

//        lifterEncoder = robot.lifter.getCurrentPosition();
        if(raisingLifter) {
//            if(!robot.lifter.isBusy()) {
//                raisingLifter = false;
//                robot.stopLifter();
//            }
        }
        if(gamepad1.y) {
            raisingLifter = true;
//            robot.startLifterUp();
        }
        if(gamepad1.right_trigger > 0.4)
        {
            if(raisingLifter) {
                raisingLifter = false;
//                robot.stopLifter();
            }
            liftPower = 1.0;
        } else if(gamepad1.left_trigger > 0.4)
        {
            if(raisingLifter) {
                raisingLifter = false;
//                robot.stopLifter();
            }
            liftPower = -1.0;
        } else
        {
            liftPower = 0.0;
        }

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

        if(autoCollectLeft && !leftHasElement) {
            float hsvValues[] = {0F, 0F, 0F};
//            double leftDistance = robot.sensorDistanceLeft.getDistance(DistanceUnit.CM);
//            if(leftDistance < ELEMENT_PRESENT) {
//                leftRed = robot.sensorColorLeft.red();
//                leftGreen = robot.sensorColorLeft.green();
//                leftBlue = robot.sensorColorLeft.blue();
                Color.RGBToHSV((int) (leftRed * SCALE_FACTOR),
                        (int) (leftGreen * SCALE_FACTOR),
                        (int) (leftBlue * SCALE_FACTOR),
                        hsvValues);
                autoCollectLeft = false;
                leftCollectorPower = 0.0;
                leftHasElement = true;
                leftH = hsvValues[0] + leftHueRoomCorrection;
                if(hsvValues[0] > 20 && hsvValues[0] < 75) {
                    leftColor = GOLD_ELEMENT_COLOR;
                    leftElement = CUBE;
                } else if(hsvValues[0] > 75) {
                    leftColor = SILVER_ELEMENT_COLOR;
                    leftElement = BALL;
                } else {
                    leftColor = NO_ELEMENT_COLOR;
                    leftElement = NOTHING;
                }
                updateLeds = true;
//            }
        }

//        if(autoCollectRight && !rightHasElement) {
            float hsvValues[] = {0F, 0F, 0F};
//            double rightDistance = robot.sensorDistanceRight.getDistance(DistanceUnit.CM);
//            if(rightDistance < ELEMENT_PRESENT) {
//                rightRed = robot.sensorColorRight.red();
//                rightGreen = robot.sensorColorRight.green();
//                rightBlue = robot.sensorColorRight.blue();
//                Color.RGBToHSV((int) (rightRed * SCALE_FACTOR),
//                        (int) (rightGreen * SCALE_FACTOR),
//                        (int) (rightBlue * SCALE_FACTOR),
//                        hsvValues);
//                autoCollectRight = false;
//                rightCollectorPower = 0.0;
//                rightHasElement = true;
//                rightH = hsvValues[0] + rightHueRoomCorrection;
//                if(hsvValues[0] > 20 && hsvValues[0] < 75) {
//                    rightColor = GOLD_ELEMENT_COLOR;
//                    rightElement = CUBE;
//                } else if(hsvValues[0] > 75) {
//                    rightColor = SILVER_ELEMENT_COLOR;
//                    rightElement = BALL;
//                } else {
//                    rightColor = NO_ELEMENT_COLOR;
//                    rightElement = NOTHING;
//                }
//                updateLeds = true;
//            }
//        }

        //slow up
//        robot.drive(speedMultiplier * xPower, speedMultiplier * yPower, spinMultiplier * spin, driverAngle);
        robot.drive(speedMultiplier * xPower, speedMultiplier * yPower, spinMultiplier * spin, driverAngle);

        //Set null zones
        if(abs(liftPower) <= 0.1)
        {
            liftPower = 0.0;
        }
        if(abs(extendPower) <= 0.1)
        {
            extendPower = 0.0;
        }
        if(abs(rotatePower) <= 0.1)
        {
            rotatePower = 0.0;
        }
        if(!raisingLifter) {
//            robot.setLiftMotorPower(liftPower);
        }
//        robot.setRotatorMotorPower(rotatePower);
//        robot.setExtenderMotorPower(extendPower, extenderOverride);
//        robot.setLeftCollectorPower(rightCollectorPower);
//        robot.setRightCollectorPower(leftCollectorPower);

//        rotatorEncoder = robot.rotator1.getCurrentPosition();
//        extenderEncoder = robot.extender.getCurrentPosition();

        if(updateLeds) {
            robot.updateElementColors(rightColor, leftColor);
            updateLeds = false;
        }
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Rotate Power: ", rotatePower);
        telemetry.addData("Extend Power: ", extendPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData( "LanderEncoder: ", lifterEncoder);
        telemetry.addData( "RotatorEncoder: ", rotatorEncoder);
        telemetry.addData( "ExtenderEncoder: ", extenderEncoder);
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        telemetry.addData("Element Detected Left: ", leftElement);
        telemetry.addData("Left H: ", leftH);
        telemetry.addData("Element Detected Right: ", rightElement);
        telemetry.addData("Right H: ", rightH);
        telemetry.addData("LF Power: ", robot.frontLeftMotorPower);
        telemetry.addData("RF Power: ", robot.frontRightMotorPower);
        telemetry.addData("LR Power: ", robot.rearLeftMotorPower);
        telemetry.addData("RR Power: ", robot.rearRightMotorPower);
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
        robot.stopGroundEffects();
    }
}
