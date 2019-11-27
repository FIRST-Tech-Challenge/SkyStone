package org.firstinspires.ftc.opmodes.mecanum.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@TeleOp(name="Mecanum TeleOp V-CompetitionReady", group="TeleComp")
public class SiBorgsMecanumTeleOp extends OpMode
{
    // TeleOp specific variables
    private SiBorgsMecanumRobot robot;
    private ElapsedTime elapsedTime;

    // Servo buttons
    private Button platformServoUp;
    private Button platformServoDown;
    private Button armServoUp;
    private Button armServoDown;

    // Buttons and toggles
    private ToggleBoolean driverTwoBrakes;  //freezes robot in place for stacking, prevents stick bumping from driver one
    private Button playSound;
    private Button toggleLimited;

    @Override
    public void init()
    {
        // Misc class declarations
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        elapsedTime = new ElapsedTime();

        // Servo buttons
        platformServoUp = new Button();
        platformServoDown = new Button();
        armServoUp = new Button();
        armServoDown = new Button();

        // Buttons and toggles
        driverTwoBrakes = new ToggleBoolean(false);
        playSound = new Button();
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
        if (playSound.onRelease()) { robot.sirenSound.toggleSound(); }


        //DRIVER TWO
        // gamepad 2 inputs
        toggleLimited.input(gamepad2.b);
        driverTwoBrakes.input(gamepad2.left_bumper);
        // servo controls
        platformServoUp.input(gamepad2.dpad_up);
        platformServoDown.input(gamepad2.dpad_down);
        armServoUp.input(gamepad2.y);
        armServoDown.input(gamepad2.a);

        if (toggleLimited.onRelease())
        {
            robot.crane.getVerticalLimitedMotor().setLimited(!robot.crane.getVerticalLimitedMotor().isLimited());
            robot.crane.getHorizontalLimitedMotor().setLimited(!robot.crane.getHorizontalLimitedMotor().isLimited());
        }

        robot.crane.setVerticalPower(gamepad2.left_stick_y);
        robot.crane.setHorizontalPower(gamepad2.right_stick_y);

        if (platformServoUp.onRelease()) { robot.platformServo.setPosition(1); }
        else if (platformServoDown.onRelease()) { robot.platformServo.setPosition(0); }

        if (armServoUp.onRelease()) { robot.armGripSlide.setPosition(1); }
        else if (armServoDown.onRelease()) { robot.armGripSlide.setPosition(0); }

        //TELEMETRY
        robot.driverTelemetry();
    }

    @Override
    public void stop()
    {
        robot.sirenSound.stopSound(); // very important otherwise it will keep playing forever

        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
