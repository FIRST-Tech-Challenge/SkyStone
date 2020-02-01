package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.stormbots.MiniPID;

public class GyroBot extends CameraBot {

    BNO055IMU imu;
    double startAngle, power = 0.15;


    public GyroBot(LinearOpMode opMode) {
        super(opMode);
    }


    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";


        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void resetAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotLog.d(String.format("Reset Angle : %.3f , %.3f, %.3f", angles.firstAngle, angles.secondAngle, angles.thirdAngle));
        startAngle = angles.firstAngle;
    }


    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    public double getDeltaAngle() {

        double angle = getAngle();
        double deltaAngle = angle - startAngle;
        RobotLog.d(String.format("Delta Angle : %.3f from %.3f", deltaAngle, angle));

        return deltaAngle;
    }


    public void goBacktoStartAngle() {

        int direction;
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double delta = getDeltaAngle();

        while (Math.abs(delta) > 2) {
            if (delta < 0) {
                // turn clockwize
                direction = -1;
            } else {
                // turn CC wize
                direction = 1;
            }
            leftFront.setPower(power * direction);
            rightFront.setPower(-power * direction);
            leftRear.setPower(power * direction);
            rightRear.setPower(-power * direction);

            delta = getDeltaAngle();

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

    public void goBacktoStartAnglePID() {

        MiniPID pid = new MiniPID(0.03, 0, 0);
        pid.setOutputLimits(0.5);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angle;
        angle = getAngle();
        double power = pid.getOutput(angle, startAngle);
        while (Math.abs(power) > 0.06) {
            RobotLog.d(String.format("PID(source: %.3f, target: %.3f) = power: %.3f", angle, startAngle, power));
            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftRear.setPower(-power);
            rightRear.setPower(power);
            opMode.sleep(50);
            angle = getAngle();
            power = pid.getOutput(angle, startAngle);
        };
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void driveStraightByGyro(int direction, double distance, double maxPower, boolean useCurrentAngle) {
        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD && direction != DIRECTION_LEFT && direction != DIRECTION_RIGHT){
            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
            print(msg);
            return;
        }
        double originalAngle;
        if (useCurrentAngle) {
            originalAngle = getAngle();
        } else {
            originalAngle = startAngle;
        }

        // distance (in mm) = revolution * pi * diameter (100 mm)
        int distanceTicks = (int) (distance / 3.1415 / 100 * DRIVING_MOTOR_TICK_COUNT);
        int startingPosition = leftFront.getCurrentPosition();
        MiniPID pid = new MiniPID(0.03, 0, 0);
        pid.setOutputLimits(maxPower);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angle;
        angle = getAngle();
        double adjustPower = pid.getOutput(angle, originalAngle);
        int currentPosition = leftFront.getCurrentPosition();
        while (Math.abs(currentPosition - startingPosition) < distanceTicks) {
            RobotLog.d(String.format("driveStraightByGyro : Current: %d - Start:%d > 10 => power: %.3f  +/- PID(source: %.3f, target: %.3f) = adjustPower: %.3f", currentPosition, startingPosition, maxPower, angle, originalAngle, adjustPower));
            switch (direction){
                case DIRECTION_FORWARD:
                    leftFront.setPower(maxPower - adjustPower);
                    rightFront.setPower(maxPower + adjustPower);
                    leftRear.setPower(maxPower - adjustPower);
                    rightRear.setPower(maxPower + adjustPower);
                    break;
                case DIRECTION_BACKWARD:
                    leftFront.setPower(- maxPower - adjustPower);
                    rightFront.setPower(- maxPower + adjustPower);
                    leftRear.setPower(- maxPower - adjustPower);
                    rightRear.setPower(- maxPower + adjustPower);
                    break;
                case DIRECTION_LEFT:
                    leftFront.setPower(- maxPower - adjustPower);
                    rightFront.setPower(+ maxPower + adjustPower);
                    leftRear.setPower(+ maxPower - adjustPower);
                    rightRear.setPower(- maxPower + adjustPower);
                    break;
                case DIRECTION_RIGHT:
                    leftFront.setPower(+ maxPower - adjustPower);
                    rightFront.setPower(- maxPower + adjustPower);
                    leftRear.setPower(- maxPower - adjustPower);
                    rightRear.setPower(+ maxPower + adjustPower);
                    break;
            }
            opMode.sleep(50);
            angle = getAngle();
            adjustPower = pid.getOutput(angle, originalAngle);
            currentPosition = leftFront.getCurrentPosition();
        };
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        opMode.sleep(500);
    }
}