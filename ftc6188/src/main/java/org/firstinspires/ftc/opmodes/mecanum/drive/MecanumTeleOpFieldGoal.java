package org.firstinspires.ftc.opmodes.mecanum.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumFieldGoalRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@TeleOp(name="Mecanum TeleOp V-FieldGoal", group="Tele")
public class MecanumTeleOpFieldGoal extends OpMode
{
    // TeleOp specific variables
    private MecanumFieldGoalRobot robot;
    private ElapsedTime elapsedTime;

    // Buttons and toggles
    private ToggleBoolean driverTwoBrakes;  //freezes robot in place for stacking, prevents stick bumping from driver one
    private Button playSound;
    private Button platformServoUp;
    private Button platformServoDown;
    private Button armServoUp;
    private Button armServoDown;
    private Button toggleLimited;

    @Override
    public void init()
    {
        robot = new MecanumFieldGoalRobot(this.hardwareMap, this.telemetry, true);
        elapsedTime = new ElapsedTime();

        driverTwoBrakes = new ToggleBoolean(false);
        playSound = new Button();
        platformServoUp = new Button();
        platformServoDown = new Button();
        armServoUp = new Button();
        armServoDown = new Button();
        toggleLimited = new Button();
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
        // both convert sticks into vectors and take two different readings from the resulting vector
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        // gamepad 1 inputs
        robot.drivetrain.lowPowerInput(gamepad1.right_stick_button);
        playSound.input(gamepad1.x);

        // movement controls
        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity * (driverTwoBrakes.output() ? 0 : 1));
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);

        // sound
        if (playSound.onRelease()) { robot.basicSound.toggleSound(); }


        //DRIVER TWO
        // gamepad 2 inputs
        platformServoUp.input(gamepad2.dpad_up);
        platformServoDown.input(gamepad2.dpad_down);
        armServoUp.input(gamepad2.y);
        armServoDown.input(gamepad2.a);
        toggleLimited.input(gamepad2.b);
        driverTwoBrakes.input(gamepad2.left_bumper || gamepad1.left_bumper);

        if (toggleLimited.onRelease())
        {
            robot.armSystem.getVerticalLimitedMotor().setLimited(!robot.armSystem.getVerticalLimitedMotor().isLimited());
            robot.armSystem.getHorizontalLimitedMotor().setLimited(!robot.armSystem.getHorizontalLimitedMotor().isLimited());
        }

        robot.armSystem.setVerticalPower(gamepad2.left_stick_y);
        robot.armSystem.setHorizontalPower(gamepad2.right_stick_y);

        if (platformServoUp.onRelease()) { robot.platformServos.setPosition(1); }
        else if (platformServoDown.onRelease()) { robot.platformServos.setPosition(0); }

        if (armServoUp.onRelease()) { robot.armGripSlide.setPosition(1); }
        else if (armServoDown.onRelease()) { robot.armGripSlide.setPosition(0); }

        //TELEMETRY
        robot.informationUpdate();
    }

    @Override
    public void stop()
    {
        robot.basicSound.stopSound(); // very important otherwise it will keep playing forever

        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
