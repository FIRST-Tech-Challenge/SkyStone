package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PIDBot2 extends GyroBot {

    DcMotor leftMotor, rightMotor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;


    public PIDBot2(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");


        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

    }
    public void driveStraightbyDistancePID(int direction, double distance, double maxPower){
        // distance (in mm) = revolution * pi * diameter (100 mm)
        int target = (int)(distance / 3.1415 / 100 * DRIVING_MOTOR_TICK_COUNT);
        int startingPosition = leftFront.getCurrentPosition();
        switch (direction){
            case DIRECTION_FORWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_BACKWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_LEFT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_RIGHT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_RQUARTER:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_LQUARTER:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            default:
                String msg = String.format("Unaccepted direction value (%d) for driveStraightByDistance()", direction);
                print(msg);
        }

        double power = maxPower;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        while (this.opMode.opModeIsActive() && leftFront.isBusy()) {
            leftFront.setPower(power + correction);
            rightFront.setPower(power - correction);
            leftRear.setPower(power - correction );
            rightRear.setPower(power + correction);
            // Display it for the driver.
            print(String.format("Target : %7d @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                    target,
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(),
                    rightRear.getCurrentPosition()));
        }
        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        print(String.format("Arrive target : %7d @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                target,
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition()));
    }





}
