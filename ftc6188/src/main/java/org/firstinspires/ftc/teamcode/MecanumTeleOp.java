package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@TeleOp(name="Mecanum TeleOp V-Test", group="TeleOp")
public class MecanumTeleOp extends OpMode
{
    private MecanumRobot robot;
    private ElapsedTime elapsedTime;

    private ToggleBoolean driverTwoBrakes;

    @Override
    public void init()
    {
        robot = new MecanumRobot(this.hardwareMap);
        driverTwoBrakes = new ToggleBoolean();
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
        //DRIVER ONE
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        robot.drivetrain.halfPowerInput(gamepad1.right_stick_button);

        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity * (driverTwoBrakes.output() ? 0 : 1));
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);

        if (gamepad1.dpad_down) {robot.platformServos.setPosition(120);}
        else if (gamepad1.dpad_up) {robot.platformServos.setPosition(0);}

        //DRIVER TWO
        //arm movement
        driverTwoBrakes.input(gamepad2.left_bumper); //freezes robot in place for stacking, prevents stick bumping from driver one

        //TELEMETRY
        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", -gamepad1.left_stick_x);
        telemetry.addData("Servo Position", robot.platformServos.getPosition());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
