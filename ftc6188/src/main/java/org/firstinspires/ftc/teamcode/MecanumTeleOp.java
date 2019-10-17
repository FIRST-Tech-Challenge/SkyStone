package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@TeleOp(name="Mecanum TeleOp", group="TeleOp")
public class MecanumTeleOp extends OpMode
{
    private MecanumRobot robot;
    private ElapsedTime elapsedTime;

    @Override
    public void init()
    {
        robot = new MecanumRobot(this.hardwareMap);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void init_loop()
    {
        telemetry.addData("Status", "Init Loop");
        telemetry.update();
    }

    @Override
    public void start()
    {
        elapsedTime.reset();
    }

    @Override
    public void loop()
    {
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

        robot.drivetrain.halfPowerInput(gamepad1.right_stick_button);

        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);

        //robot.armParallelLift.setPower(gamepad2.left_stick_y);

        /**
        if (gamepad1.dpad_down)
        {
            robot.servoBuildClawLeft.setPosition(120);
            robot.servoBuildClawRight.setPosition(120);
        }

        if (gamepad1.dpad_up)
        {
            robot.servoBuildClawLeft.setPosition(0);
            robot.servoBuildClawRight.setPosition(0);
        }
         **/

        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", -gamepad1.left_stick_x);
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
