// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Foundation", group = "Autonomous")
public class Auto_IMU extends LinearOpMode  {
    private DcMotor drive_FL, drive_FR, drive_RL, drive_RR;
    private DcMotor intake_L, intake_R;
    private Servo latch_L, latch_R;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;

    // called when init button is  pressed.
    @Override
    public void runOpMode() {
        drive_FL = hardwareMap.dcMotor.get("front left");
        drive_FR = hardwareMap.dcMotor.get("front right");
        drive_RL = hardwareMap.dcMotor.get("rear left");
        drive_RR = hardwareMap.dcMotor.get("rear right");

        latch_L = hardwareMap.servo.get("left servo");
        latch_R = hardwareMap.servo.get("right servo");

        drive_FL.setDirection(DcMotor.Direction.REVERSE);
        drive_RR.setDirection(DcMotor.Direction.REVERSE);

        drive_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to touch sensor.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.

        while (opModeIsActive()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            //set direction of motors
            drive_FL.setDirection(DcMotor.Direction.FORWARD);
            drive_RR.setDirection(DcMotor.Direction.FORWARD);
            drive_FR.setDirection(DcMotor.Direction.REVERSE);
            drive_RL.setDirection(DcMotor.Direction.REVERSE);

            //drive forward
            drive_FL.setPower(power - correction);
            drive_FR.setPower(power + correction);
            drive_RL.setPower((power) - correction);
            drive_RR.setPower((power) + correction);

            sleep(1000);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            sleep(1000);

            //turn right
            rotate(85, 0.3);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            sleep(1000);

            //drive forward
            drive_FL.setPower(power - correction);
            drive_FR.setPower(power + correction);
            drive_RL.setPower((power) - correction);
            drive_RR.setPower((power) + correction);

            sleep(1000);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            sleep(1000);

            //turn left
            rotate(-85, 0.3);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            sleep(1000);

            //drive forward
            drive_FL.setPower(power - correction);
            drive_FR.setPower(power + correction);
            drive_RL.setPower((power) - correction);
            drive_RR.setPower((power) + correction);

            sleep(2500);

            //put servos down
            latch_L.setPosition(0.5);
            latch_R.setPosition(0.3);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            sleep(1000);

            //set direction to reverse
            drive_FL.setDirection(DcMotor.Direction.REVERSE);
            drive_RR.setDirection(DcMotor.Direction.REVERSE);
            drive_FR.setDirection(DcMotor.Direction.FORWARD);
            drive_RL.setDirection(DcMotor.Direction.FORWARD);

            //drive backwards with foundation
            drive_FL.setPower(0.4 - correction);
            drive_FR.setPower(0.4 + correction);
            drive_RL.setPower(0.4 - correction);
            drive_RR.setPower(0.4  + correction);

            sleep(4500);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            sleep(1000);

            //raise servos
            latch_L.setPosition(1);
            latch_R.setPosition(0);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            sleep(1000);


            //drive backwards even further to break contact with foundation
            drive_FL.setPower(0.4 - correction);
            drive_FR.setPower(0.4 + correction);
            drive_RL.setPower((0.4)-correction);
            drive_RR.setPower((0.4) + correction);

            sleep(500);

            drive_FL.setPower(0);
            drive_FR.setPower(0);
            drive_RL.setPower(0);
            drive_RR.setPower(0);

            //wait until game ends
            sleep(30000);



            //rotate(82, 0.4);


            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.
        }

        // turn the motors off.
        drive_FR.setPower(0);
        drive_FL.setPower(0);
        drive_RR.setPower(0);
        drive_RL.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        drive_FL.setPower(leftPower);
        drive_FR.setPower(rightPower);
        drive_RL.setPower(leftPower*0.5);
        drive_RR.setPower(rightPower*0.5);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                idle();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                idle();
            }
        } else {   // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                idle();
            }
        }

        // turn the motors off.
        drive_FR.setPower(0);
        drive_FL.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}