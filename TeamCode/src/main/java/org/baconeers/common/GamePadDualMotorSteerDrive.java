package org.baconeers.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GamePadDualMotorSteerDrive extends BaconComponent {

    final private DcMotor leftLeftMotor;
    final private DcMotor leftRightMotor;
    final private DcMotor rightLeftMotor;
    final private DcMotor rightRightMotor;
    final private Gamepad gamepad;
    private Telemetry.Item leftPowerItem = null;
    private Telemetry.Item rightPowerItem = null;
    private Telemetry.Item steerPowerItem = null;
    private Telemetry.Item rawPowerItem = null;


    public GamePadDualMotorSteerDrive(BaconOpMode opMode, Gamepad gamepad,
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
        steerPowerItem = getOpMode().telemetry.addData("steer power", "%.2f", 0.0f);
        steerPowerItem.setRetained(true);
        rawPowerItem = getOpMode().telemetry.addData("raw power", "%.2f", 0.0f);
        rawPowerItem.setRetained(true);
    }

    /*
     * Update the motor power based on the gamepad state
     */
    public void update() {

        float scalePower = scaleTriggerPower(gamepad.left_trigger - gamepad.right_trigger);
//        float scalePower = scaleTriggerPower(gamepad.left_stick_y);

        float steer = scaleSteerPower(gamepad.left_stick_x);
        float leftPower;
        float rightPower;
        if (scalePower == 0.0f) {
            leftPower = steer;
            rightPower = -steer;
        }
        else {
            leftPower = scalePower * ((steer < 0) ? 1.0f + steer : 1.0f);
            rightPower = scalePower * ((steer > 0) ? 1.0f - steer : 1.0f);
        }


        leftLeftMotor.setPower(leftPower);
        leftRightMotor.setPower(leftPower);
        rightLeftMotor.setPower(rightPower);
        rightRightMotor.setPower(rightPower);

        leftPowerItem.setValue("%.2f", leftPower);
        rightPowerItem.setValue("%.2f", rightPower);
        steerPowerItem.setValue("%.2f", steer);
        rawPowerItem.setValue("%.2f", scalePower);
    }

    private static float[] power_curve =
            {0.00f, 0.1f, 0.3f, 0.7f, 1.0f, 1.0f };

    /**
     * The DC motors are scaled to make it easier to control them at slower speeds
     * The clip method guarantees the value never exceeds the range 0-1.
     */
    private float scaleTriggerPower(float power) {

        // Ensure the values are legal.
        float clipped_power = Range.clip(power, -1, 1);

        // Remember if this is positive or negative
        float sign = Math.signum(clipped_power);

        // Work only with positive numbers for simplicity
        float abs_power = Math.abs(clipped_power);

        // Map the power value [0..1.0] to a power curve index
        int index = (int) (abs_power * (power_curve.length - 1));

        float scaled_power = sign * power_curve[index];

        return scaled_power;
    }


    private static float[] steer_curve =
            {0.00f, 0.1f, 0.2f, 0.3f, 0.4f, 0.4f, 0.4f, 0.5f, 0.5f, 0.7f};

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
