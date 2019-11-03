package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@TeleOp(name="Mecanum TeleOp V-Final", group="Tele")
public class MecanumTeleOp extends OpMode
{
    private MecanumRobot robot;
    private ElapsedTime elapsedTime;

    private ToggleBoolean driverTwoBrakes;
    private Button servosUp;
    private Button servosDown;
    private Button servosMid;

    @Override
    public void init()
    {
        robot = new MecanumRobot(this.hardwareMap, this.telemetry, true);
        driverTwoBrakes = new ToggleBoolean();
        servosUp = new Button();
        servosDown = new Button();
        servosMid = new Button();
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
        // both convert sticks into vectors and take two different readings from the resulting vector

        robot.drivetrain.halfPowerInput(gamepad1.right_stick_button);
        servosUp.input(gamepad1.dpad_up);
        servosDown.input(gamepad1.dpad_down);
        servosMid.input(gamepad1.dpad_left || gamepad1.dpad_right);

        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity * (driverTwoBrakes.output() ? 0 : 1));
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);

        if (servosUp.onPress()){robot.platformServos.setPosition(1); }
        else if (servosDown.onPress()){robot.platformServos.setPosition(0);}
        else if (servosMid.onPress()){robot.platformServos.setPosition(0.6);}

        //DRIVER TWO
        //arm movement to be added later
        driverTwoBrakes.input(gamepad2.left_bumper); //freezes robot in place for stacking, prevents stick bumping from driver one

        //TELEMETRY
        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", -gamepad1.left_stick_x);
        telemetry.addData("Servo Position", robot.platformServos.getPosition());
        telemetry.addData("Servo Actual", robot.platformServos.getActual());
		telemetry.addData("Half Power", robot.drivetrain.isHalfPower());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
