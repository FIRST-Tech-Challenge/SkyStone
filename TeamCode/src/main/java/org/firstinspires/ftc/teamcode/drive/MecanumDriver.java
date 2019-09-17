package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.DeviceMap;

import java.util.Locale;

public final class MecanumDriver implements IDriver {
    private DeviceMap map;
    private Telemetry telemetry;
    private DcMotor[] motors;

    private static final double COUNTS_PER_MOTOR_REV = 560;
    private static final double WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER_INCHES * Math.PI);

    public MecanumDriver() {
        this.map = DeviceMap.getInstance();
        this.motors = map.getMotors();
    }

    /**
     * Move an unspecified amount of distance
     * @param direction
     * @param power
     */
    @Override
    public void move(Direction direction, double power) {
        telemetry.addData("MOVING: ", direction.name());
        map.getLeftTop().setPower(direction.getLeftTop() * power);
        map.getRightBottom().setPower(direction.getRightBottom() * power);
        map.getRightTop().setPower(direction.getRightTop() * power);
        map.getLeftBottom().setPower(direction.getLeftBottom() * power);

        /*
        int[] movements = direction.getMovement();
        int length = movements.length;
        for(int i = 0; i < length; i++) {
            map.getMotors()[i].setPower(movements[i] * power);
        }
         */

        telemetry.update();
    }

    /**
     * encoder drive
     * @param direction
     * @param power
     * @param inches
     */
    @Override
    public void move(Direction direction, double power, double inches) {
        DcMotor leftTop = map.getLeftTop();
        DcMotor rightTop = map.getRightTop();
        DcMotor leftBottom = map.getLeftBottom();
        DcMotor rightBottom = map.getRightBottom();

        //other calculations needed
        leftTop.setTargetPosition(leftTop.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches * direction.getLeftTop()));
        rightTop.setTargetPosition(rightTop.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches * direction.getRightTop()));
        leftBottom.setTargetPosition(leftBottom.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches * direction.getLeftBottom()));
        rightBottom.setTargetPosition(rightBottom.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches * direction.getRightBottom()));

        for(DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        move(direction, power);
        while(motorsBusy()) {
            telemetry.addLine(String.format(Locale.ENGLISH,"%d : %d", leftTop.getCurrentPosition(), leftTop.getTargetPosition()));
            telemetry.addLine(String.format(Locale.ENGLISH,"%d : %d", rightTop.getCurrentPosition(), rightTop.getTargetPosition()));
            telemetry.addLine(String.format(Locale.ENGLISH,"%d : %d", rightBottom.getCurrentPosition(), rightBottom.getTargetPosition()));
            telemetry.addLine(String.format(Locale.ENGLISH,"%d : %d", leftBottom.getCurrentPosition(), leftBottom.getTargetPosition()));
            telemetry.addLine("Direction: " + direction.name());
            telemetry.update();
        }
        stop();

        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void move(AngleConverter angleConverter) {
        map.getLeftTop().setPower(angleConverter.getLeftTop());
        map.getRightTop().setPower(angleConverter.getRightTop());
        map.getRightBottom().setPower(angleConverter.getRightBottom());
        map.getLeftBottom().setPower(angleConverter.getLeftBottom());
    }

    /**
     * Updated mecanum drive function this year (math is ? ?? ? )
     * @param left_stick_x gamepadleftX
     * @param left_stick_y gamepadleftY
     * @param right_stick_x gamepadrightX
     */
    public void move(double left_stick_x, double left_stick_y, double right_stick_x){
        right_stick_x *= -1;
        double LB = Range.clip(left_stick_x + left_stick_y + right_stick_x, -1, 1);
        double LF = Range.clip(left_stick_x - left_stick_y + right_stick_x, -1, 1);
        double RB = Range.clip(left_stick_x - left_stick_y - right_stick_x, -1, 1);
        double RF = Range.clip(left_stick_x + left_stick_y - right_stick_x, -1, 1);
        map.getLeftBottom().setPower(LB);
        map.getLeftTop().setPower(LF);
        map.getRightBottom().setPower(RB);
        map.getRightTop().setPower(RF);
    }

    @Override
    public void stop() {
        for(DcMotor motor : map.getMotors())
            motor.setPower(0);
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private boolean motorsBusy() {
        for(DcMotor motor : motors) {
            if(motor.isBusy()) return true;
        }
        return false;
    }
}
