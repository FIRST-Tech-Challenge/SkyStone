package org.firstinspires.ftc.opmodes.encoderTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.robot.EncoderTestPlatformRobot;

@Disabled
@Autonomous(name="Encoder Test Platform Auto", group="EncoderTestPlatform")
public class EncoderTestPlatformAuto extends LinearOpMode
{
    private EncoderTestPlatformRobot robot;
    private static final double VELOCITY = 0.25;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new EncoderTestPlatformRobot(this.hardwareMap);
        robot.informationTelemetry(telemetry, "Wait For Start");
        waitForStart();

        while (opModeIsActive())
        {
            robot.drivetrain.autoPosition(0, 12, VELOCITY, 0);
            sleep(10000);
            robot.drivetrain.autoPosition(90, 12, VELOCITY, 0);
            sleep(10000);
            robot.drivetrain.autoPosition(180, 12, VELOCITY, 0);
            sleep(10000);
            robot.drivetrain.autoPosition(270, 12, VELOCITY, 0);
            sleep(10000);
        }
    }

    private void autoPosition(double course, double distanceIN, double velocity, double rotation)
    {
        // carbon copy of function found in robot.drivetrain.autoPosition(); just so we can include telemetry
        robot.drivetrain.setTargetPosition(distanceIN * robot.drivetrain.getTicksPerIn());
        robot.drivetrain.setCourse(course * (Math.PI/180.0));
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setRotation(rotation);

        while (robot.drivetrain.isPositioning())
        {
            robot.drivetrain.updatePosition();
            robot.informationTelemetry(this.telemetry, "Auto Position C: " + course
                    + " D: " + (distanceIN * robot.drivetrain.getTicksPerIn())
                    + " V: " + velocity);
        }
        robot.drivetrain.finishPositioning();
        sleep(10000);
    }

    private void encoderDrive(int distance, double speed)
    {
        if (opModeIsActive())
        {
            robot.driveFrontRight.setTargetPosition(robot.driveFrontRight.getCurrentPosition() + distance);
            robot.driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveFrontRight.setPower(speed);

            while (opModeIsActive() && robot.driveFrontRight.isBusy())
            {
                robot.informationTelemetry(telemetry, "Encoder driving " + distance);
            }

            robot.driveFrontRight.setPower(0);
            robot.driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1000);
        }
    }
}
