package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.MonitorIMU;

public final class MecanumDriver implements IDriver {
    private static final float TURN_OFFSET = 1.5F;

    private boolean test;
    private DeviceMap map;
    private Telemetry telemetry;
    private DcMotor[] motors;

    private boolean conveyerCurrent;

    private static final double COUNTS_PER_MOTOR_REV = 560;
    private static final double WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER_INCHES * Math.PI);

    public MecanumDriver() {
        this.map = DeviceMap.getInstance();
        this.motors = map.getDriveMotors();
        this.test = true;
    }

    /**
     * Move an unspecified amount of distance
     * @param direction
     * @param power
     */
    @Override
    public void move(Direction direction, double power) {
        addData("MOVING: ", direction.name());
        map.getLeftTop().setPower(direction.getLeftTop() * power);
        map.getRightBottom().setPower(direction.getRightBottom() * power);
        map.getRightTop().setPower(direction.getRightTop() * power);
        map.getLeftBottom().setPower(direction.getLeftBottom() * power);

        /*
        int[] movements = direction.getMovement();
        int length = movements.length;
        for(int i = 0; i < length; i++) {
            map.getDriveMotors()[i].setPower(movements[i] * power);
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

        double calc = COUNTS_PER_INCH * inches;
        //other calculations needed
        leftTop.setTargetPosition(leftTop.getCurrentPosition() + (int) (calc * direction.getLeftTop()));
        rightTop.setTargetPosition(rightTop.getCurrentPosition() + (int) (calc * direction.getRightTop()));
        leftBottom.setTargetPosition(leftBottom.getCurrentPosition() + (int) (calc * direction.getLeftBottom()));
        rightBottom.setTargetPosition(rightBottom.getCurrentPosition() + (int) (calc * direction.getRightBottom()));

        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        move(direction, power);
        while(motorsBusy()) {

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
        double LF = Range.clip(left_stick_y + left_stick_x + right_stick_x, -1, 1);
        double RF = Range.clip(left_stick_y - left_stick_x - right_stick_x, -1, 1);
        double LB = Range.clip(left_stick_y - left_stick_x + right_stick_x, -1, 1);
        double RB = Range.clip(left_stick_y + left_stick_x - right_stick_x, -1, 1);
        map.getLeftBottom().setPower(LB);
        map.getLeftTop().setPower(LF);
        map.getRightBottom().setPower(RB);
        map.getRightTop().setPower(RF);
    }

    @Override
    public void turn(double power, double angle, IActive stopRequested) {
        if(stopRequested == null) stopRequested = () -> {return false;};
        angle = MathUtil.convert180to360(angle);
        if(Math.abs(angle) > 180) {
            //if it's more than +180 or less than -180, add towards 0: 180
            turn(power, angle + ((angle < 0) ? +180D : -180D), stopRequested);
            return;
        }
        Direction direction = angle > 0 ? Direction.COUNTERCLOCKWISE : Direction.CLOCKWISE;

        move(direction, power);
        float min = (float) angle - TURN_OFFSET;
        float max = (float) angle + TURN_OFFSET;

        float currentAngle = MonitorIMU.getAngleThird();
            while ((currentAngle < min || currentAngle > max)) {

                addData("Direction: ", direction);
                addData("Min: ", min);
                addData("Max: ", max);
                addData("zAngle: ", currentAngle);
                telemetry.update();
                currentAngle = MonitorIMU.getAngleThird();
            }
        stop();
    }

    public void turnOrigin(double power, IActive stopRequested) {
        float angle = MonitorIMU.getAngleThird();
        turn(power, angle, stopRequested);
    }

    public void intake(double leftPower, double rightPower) {
        map.getLeftIntake().setPower(leftPower);
        map.getRightIntake().setPower(rightPower);
    }

    public void conveyer(double power) {
        DcMotor motor = map.getConveyer();
        if(motor.getPower() == power) return;
        map.getConveyer().setPower(power);
    }

    @Override
    public void stop() {
        for(DcMotor motor : map.getDriveMotors())
            motor.setPower(0);
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean motorsBusy() {
        for(DcMotor motor : motors) {
            if(motor.isBusy()) return true;
        }
        return false;
    }
    public DcMotor[] getMotors() {
        return motors;
    }

    public boolean isTest() {
        return test;
    }

    public void setTest(boolean test) {
        this.test = test;
    }
    
    public void addData(String header, Object value) {
        if(test) telemetry.addData(header, value);
    }
    public void updateTelemetry() {
        if(test) telemetry.update();
    }
}
