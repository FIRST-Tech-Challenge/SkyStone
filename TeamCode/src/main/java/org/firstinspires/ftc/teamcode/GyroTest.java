package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "GyroTest", group = "8872")
public class GyroTest extends LinearOpMode {
    Chassis robot = new Chassis();
    Orientation angles;
    double adjust = 4;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // Neverest 20
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        composeTelemetry();
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.update();
        waitForStart();
        gyroTurn(90);
        telemetry.addData("DONE", angles.firstAngle);
        sleep(100);
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.update();
        sleep(10000);
    }

    public void gyroTurn(double targetAngle) {
        double error = 2;
        double currentSpeed = 1;
        double headingAngle = normalizeAngle();
        double originalTargetAngle = targetAngle;
        adjust += Math.abs(targetAngle - headingAngle) * .012;
        if (shortestDirection(targetAngle)) {
            targetAngle -= adjust;
            if (targetAngle < -180) targetAngle += 360;
            while (headingAngle > targetAngle + error / 2 || headingAngle < targetAngle - error / 2 && opModeIsActive()) {
                if (Math.abs(targetAngle - headingAngle) < 30) {
                    currentSpeed = .2;
                } else {
                    currentSpeed = 1;
                }
                robot.leftFront.setPower(currentSpeed);
                robot.rightFront.setPower(-currentSpeed);
                robot.leftRear.setPower(currentSpeed);
                robot.rightRear.setPower(-currentSpeed);
                headingAngle = normalizeAngle();

            }
        } else {
            targetAngle += adjust;
            if (targetAngle > 180) targetAngle -= 360;
            while (headingAngle > targetAngle + error / 2 || headingAngle < targetAngle - error / 2 && opModeIsActive()) {
                if (Math.abs(targetAngle - headingAngle) < 30) {
                    currentSpeed = .2;
                } else {
                    currentSpeed = 1;
                }
                robot.leftFront.setPower(-currentSpeed);
                robot.rightFront.setPower(currentSpeed);
                robot.leftRear.setPower(-currentSpeed);
                robot.rightRear.setPower(currentSpeed);

                headingAngle = normalizeAngle();

            }
        }
        brake();
        sleep(200);
        headingAngle = normalizeAngle();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftRearTarget;
        int newRightRearTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftRearTarget = robot.leftRear.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            newRightRearTarget = robot.rightRear.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.leftRear.setTargetPosition(newLeftRearTarget);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);

            robot.rightRear.setTargetPosition(newRightRearTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftRear.setPower(speed);
            robot.rightRear.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;

            brake();
            sleep(1000);
            brake();
            sleep(1000);

            // Turn off RUN_TO_POSITION
            robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void gyroDrive(double speed, double inches, double timeout) {

        double encoderCount = inches * COUNTS_PER_INCH;
        double startPosition = robot.leftRear.getCurrentPosition();

        runtime.reset();

        if (encoderCount > 0) {

            while (robot.leftRear.getCurrentPosition() < (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }

                robot.leftFront.setPower(-speed); //negative because random problem with TMK chassis
                robot.rightFront.setPower(-speed); //negative because random problem with TMK chassis
                robot.leftRear.setPower(speed);
                robot.rightRear.setPower(speed);

                telemetry.addData("encoder value", robot.leftRear.getCurrentPosition());
                telemetry.addData("inches travelled", Math.round((robot.leftRear.getCurrentPosition() - startPosition) / COUNTS_PER_INCH));
                telemetry.update();
                sleep(50);
            }
        } else if (encoderCount < 0) {

            while (robot.leftRear.getCurrentPosition() > (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }

                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.leftRear.setPower(-speed);
                robot.rightRear.setPower(-speed);

                telemetry.addData("encoder value", robot.leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);


        sleep(50);
    }


    public boolean shortestDirection(double angle) {
        if (normalizeAngle() < angle) return true;
        else return false;

    }

    public double normalizeAngle() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double headingAngle = angles.firstAngle;
        telemetry.update();
        return headingAngle;
    }

    public void brake() {
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
    }

    public void composeTelemetry() {
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });


    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
