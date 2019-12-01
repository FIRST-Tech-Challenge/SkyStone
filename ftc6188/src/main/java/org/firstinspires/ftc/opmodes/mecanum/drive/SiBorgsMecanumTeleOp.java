package org.firstinspires.ftc.opmodes.mecanum.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoButton;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@TeleOp(name="Mecanum TeleOp V-CompetitionReady", group="TeleComp")
public class SiBorgsMecanumTeleOp extends OpMode
{
    // TeleOp specific variables
    private SiBorgsMecanumRobot robot;
    private ElapsedTime elapsedTime;

    // Servo buttons
    private ServoButton platformServoControl;
    private ServoButton armServoControl;

    // Buttons and toggles
    private ToggleBoolean driverTwoBrakes;  //freezes robot in place for stacking, prevents stick bumping from driver one
    private ToggleBoolean driveTelemetry; // changes the display output from driver style telemetry to debugging telemetry
    private Button playSound;
    private Button toggleLimited;

    @Override
    public void init()
    {
        // Misc class declarations
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        elapsedTime = new ElapsedTime();

        // Servo buttons
        platformServoControl = new ServoButton();
        armServoControl = new ServoButton();

        // Buttons and toggles
        driverTwoBrakes = new ToggleBoolean(false);
        driveTelemetry = new ToggleBoolean(true);
        playSound = new Button();
        toggleLimited = new Button();
    }

    @Override
    public void init_loop()
    {
        driveTelemetry.input(gamepad1.x || gamepad2.x);

        if (driveTelemetry.output()) { robot.driverTelemetry(); }
        else { robot.informationTelemetry("Init_Loop display"); }
    }

    @Override
    public void start()
    {
        elapsedTime.reset();
    }

    @Override
    public void loop()
    {
        /** DRIVER ONE **/

        // Gamepad 1 inputs
        // both convert sticks into vectors and take two different readings from the resulting vector
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        robot.drivetrain.lowPowerInput(gamepad1.right_stick_button);
        playSound.input(gamepad1.x);


        // Drivetrain updates
        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity * (driverTwoBrakes.output() ? 0 : 1));
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);


        // sound
        if (playSound.onPress()) { robot.sirenSound.toggleSound(); }


        /** DRIVER TWO **/

        // Gamepad 2 inputs
        toggleLimited.input(gamepad2.b);
        driverTwoBrakes.input(gamepad2.left_bumper);

        platformServoControl.input(gamepad2.dpad_up, gamepad2.dpad_down);
        armServoControl.input(gamepad2.y, gamepad2.a);


        // Basic toggles
        if (toggleLimited.onPress())
        {
            robot.crane.getVerticalLimitedMotor().setLimited(!robot.crane.getVerticalLimitedMotor().isLimited());
            robot.crane.getHorizontalLimitedMotor().setLimited(!robot.crane.getHorizontalLimitedMotor().isLimited());
        }

        // Servos are set to the enum output of the servo button controller
        robot.platformServo.setPosition(platformServoControl.output());
        robot.armGripSlide.setPosition(armServoControl.output());

        // Motor powers
        robot.crane.setVerticalPower(gamepad2.left_stick_y);
        robot.crane.setHorizontalPower(gamepad2.right_stick_y);


        /** TELEMETRY **/
        if (driveTelemetry.output()) { robot.driverTelemetry(); }
        else { robot.informationTelemetry(); }
    }

    @Override
    public void stop()
    {
        robot.sirenSound.stopSound(); // very important otherwise it will keep playing forever
    }
}
