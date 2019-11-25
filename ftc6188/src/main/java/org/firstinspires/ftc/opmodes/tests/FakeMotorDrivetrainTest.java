package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotlib.robot.FakeMecanumRobot;

@Disabled
@TeleOp(name="Fake Motor Drivetrain Test", group="Test")
public class FakeMotorDrivetrainTest extends LinearOpMode
{
    private FakeMecanumRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new FakeMecanumRobot(this.hardwareMap, this.telemetry, false);
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Status", "36in Forward");
            robot.drivetrain.autoPosition(0, 36, 1, 0);
            sleep(10000);

            telemetry.addData("Status", "36in Reverse");
            robot.drivetrain.autoPosition(180, 36, 1, 0);
            sleep(10000);

            telemetry.addData("Status", "36in Right");
            robot.drivetrain.autoPosition(270, 36, 1, 0);
            sleep(10000);

            telemetry.addData("Status", "36in Left");
            robot.drivetrain.autoPosition(90, 36,1,  0);
            sleep(10000);
        }
    }
}
