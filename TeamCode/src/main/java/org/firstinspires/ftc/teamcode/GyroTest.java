package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "GyroTest", group = "8872")
//@Disabled
public class GyroTest extends LinearOpMode {
    double currentHeading = 0;
    Chassis robot = new Chassis();
    Orientation angles;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        composeTelemetry();
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.update();

        waitForStart();
        gyroTurn(-90);

        sleep(3000);
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.update();
        sleep(10000);


    }

    public void gyroTurn(double targetAngle) {
        double currentSpeed = 0.2;
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double headingAngle = angles.firstAngle;
        if (targetAngle > 0) {
            while (headingAngle < targetAngle && opModeIsActive()) {
                robot.leftFront.setPower(currentSpeed);
                robot.rightFront.setPower(-currentSpeed);
                robot.leftRear.setPower(currentSpeed);
                robot.rightRear.setPower(-currentSpeed);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                headingAngle = angles.firstAngle;
                telemetry.update();
            }
        } else if (targetAngle < 0) {
            while (headingAngle > targetAngle && opModeIsActive()) {
                robot.leftFront.setPower(-currentSpeed);
                robot.rightFront.setPower(currentSpeed);
                robot.leftRear.setPower(-currentSpeed);
                robot.rightRear.setPower(currentSpeed);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                headingAngle = angles.firstAngle;
                telemetry.update();
            }
        }
        currentHeading += targetAngle;
        if(currentHeading>=360){
            currentHeading-=360;
        }
        if(currentHeading<0){
            currentHeading+=360;
        }
        brake();


    }
    public void encoderDrive(double speed, double distance){

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
