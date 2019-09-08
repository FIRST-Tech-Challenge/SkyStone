package org.baconeers.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GamePadDualMotorSteerDrive2 extends BaconComponent {

    final private DcMotor leftLeftMotor;
    final private DcMotor leftRightMotor;
    final private DcMotor rightLeftMotor;
    final private DcMotor rightRightMotor;
    final private Gamepad gamepad;
    private Telemetry.Item leftPowerItem = null;
    private Telemetry.Item rightPowerItem = null;
    private float lastLeftPower = 0.0f;
    private float lastRightPower = 0.0f;


    public GamePadDualMotorSteerDrive2(BaconOpMode opMode, Gamepad gamepad,
                                       DcMotor leftLeftMotor, DcMotor leftRightMotor,
                                       DcMotor rightLeftMotor, DcMotor rightRightMotor) {
        super(opMode);

        this.gamepad = gamepad;
        this.leftLeftMotor = leftLeftMotor;
        this.leftRightMotor = leftRightMotor;
        this.rightLeftMotor = rightLeftMotor;
        this.rightRightMotor = rightRightMotor;

        leftPowerItem = getOpMode().telemetry.addData("Left power", "%.2f", 0.0f);
        leftPowerItem.setRetained(true);
        rightPowerItem = getOpMode().telemetry.addData("Right power", "%.2f", 0.0f);
        rightPowerItem.setRetained(true);
    }

    /*
     * Update the motor power based on the gamepad state
     */
    public void update() {

        float triggerPower;
//        float scalePower = scaleTriggerPower(gamepad.left_stick_y);

        boolean rightTriggerOn = gamepad.right_trigger > 0;
        boolean rightBumperOn = gamepad.right_bumper;

        boolean leftTriggerOn = gamepad.left_trigger > 0;
        boolean leftBumperOn = gamepad.left_bumper;

        float slowPower = 0.3f;
        float mediumPower = 0.6f;
        float fastPower = 1.0f;

        float steerFactor = 0.5f;

        if (rightBumperOn && rightTriggerOn && gamepad.b) {
            triggerPower = -slowPower;
            steerFactor = 0.75f;
        }
        else if (rightTriggerOn && rightBumperOn) {
            triggerPower = -mediumPower;
            steerFactor = 0.75f;
        }
        else if (rightTriggerOn && gamepad.b) {
            triggerPower = -slowPower;
            steerFactor = 0.75f;
        }
        else if (rightTriggerOn) {
            triggerPower = -fastPower;
            steerFactor = 0.5f;
        }
        else if (rightBumperOn && gamepad.b) {
            triggerPower = -slowPower;
            steerFactor = 0.75f;
        }
        else if (rightBumperOn){
            triggerPower = -mediumPower;
            steerFactor = 0.75f;
        }
        else {
            if (leftBumperOn && leftTriggerOn && gamepad.b) {
                triggerPower = slowPower;
                steerFactor = 0.75f;
            }
            else if (leftTriggerOn && leftBumperOn) {
                triggerPower = mediumPower;
                steerFactor = 0.75f;
            }
            else if (leftTriggerOn && gamepad.b) {
                triggerPower = slowPower;
                steerFactor = 0.75f;
            }
            else if (leftTriggerOn) {
                triggerPower = fastPower;
                steerFactor = 0.5f;
            }
            else if (leftBumperOn && gamepad.b) {
                triggerPower = slowPower;
                steerFactor = 0.75f;
            }
            else if (leftBumperOn){
                triggerPower = mediumPower;
                steerFactor = 0.75f;
            }
            else {
                triggerPower = 0.0f;
            }
        }

        float steer = scaleSteerPower(gamepad.left_stick_x);
        float leftPower;
        float rightPower;

        if (triggerPower == 0.0f) {
            if (gamepad.b) {
                leftPower = -steer/2.0f;
                rightPower = steer/2.0f;
            }
            else {
                leftPower = -steer;
                rightPower = steer;
            }
        }
        else if (triggerPower < 0){
            rightPower = triggerPower * ((steer > 0) ? 1.0f - (steer * steerFactor) : 1.0f);
            leftPower = triggerPower * ((steer < 0) ? 1.0f + (steer * steerFactor) : 1.0f);
        }
        else {
            leftPower = triggerPower * ((steer > 0) ? 1.0f - (steer * steerFactor) : 1.0f);
            rightPower = triggerPower * ((steer < 0) ? 1.0f + (steer * steerFactor) : 1.0f);
        }

        if (lastLeftPower != leftPower)
        {
            leftLeftMotor.setPower(leftPower);
            leftRightMotor.setPower(leftPower);
            leftPowerItem.setValue("%.2f", leftPower);
        }
        lastLeftPower = leftPower;

        if (lastRightPower != rightPower) {
            rightLeftMotor.setPower(rightPower);
            rightRightMotor.setPower(rightPower);
            rightPowerItem.setValue("%.2f", rightPower);
        }
        lastRightPower = rightPower;

    }

    private static float[] steer_curve =
            {0.00f, 0.1f, 0.2f, 0.3f, 0.4f, 0.4f, 0.4f, 0.5f, 0.5f, 0.7f};

    /**
     * The DC motors are scaled to make it easier to control them at slower speeds
     * The clip method guarantees the value never exceeds the range 0-1.
     */

    private float scaleSteerPower(float p_power) {

        // Ensure the values are legal.
        float clipped_power = Range.clip(p_power, -1, 1);

        // Remember if this is positive or negative
        float sign = Math.signum(clipped_power);

        // Work only with positive numbers for simplicity
        float abs_power = Math.abs(clipped_power);

        // Map the power value [0..1.0] to a power curve index
        int index = (int) (abs_power * (steer_curve.length - 1));

        float scaled_power = sign * steer_curve[index];

        return scaled_power;

    }

}
