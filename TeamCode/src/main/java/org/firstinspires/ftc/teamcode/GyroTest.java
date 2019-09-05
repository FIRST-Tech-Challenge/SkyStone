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
    Chassis robot = new Chassis();
    Orientation angles;
    double adjust=4;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        composeTelemetry();
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.update();

        waitForStart();
        gyroTurn(20);
        sleep(1000);
        normalizeAngle();
        sleep(3000);
        gyroTurn(0);
        sleep(1000);
        normalizeAngle();
        sleep(3000);
        gyroTurn(90);
        sleep(1000);
        normalizeAngle();
        sleep(3000);
        gyroTurn(45);
        sleep(1000);
        normalizeAngle();
        sleep(3000);
        gyroTurn(180);
        sleep(1000);
        normalizeAngle();
        sleep(3000);
        gyroTurn(0);


        telemetry.addData("DONE", angles);


        sleep(3000);
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.update();
        sleep(10000);


    }

    public void gyroTurn(double targetAngle) {
        double error = 2;
        double currentSpeed = 1;
        double headingAngle = normalizeAngle();
        double originalTargetAngle=targetAngle;
        adjust+=Math.abs(targetAngle-headingAngle)*.012;
        if (shortestDirection(targetAngle)) {
            targetAngle-=adjust;
            if(targetAngle<-180) targetAngle+=360;
            while (headingAngle > targetAngle + error / 2 || headingAngle < targetAngle - error / 2 && opModeIsActive()) {
                if(Math.abs(targetAngle-headingAngle)<40) currentSpeed=.2;
                else currentSpeed=1;
                robot.leftFront.setPower(currentSpeed);
                robot.rightFront.setPower(-currentSpeed);
                robot.leftRear.setPower(currentSpeed);
                robot.rightRear.setPower(-currentSpeed);
                headingAngle = normalizeAngle();

            }
        } else {
            targetAngle+=adjust;
            if(targetAngle>180) targetAngle-=360;
            while (headingAngle > targetAngle + error / 2 || headingAngle < targetAngle - error / 2 && opModeIsActive()) {
                if(Math.abs(targetAngle-headingAngle)<40) currentSpeed=.2;
                else currentSpeed=1;
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
