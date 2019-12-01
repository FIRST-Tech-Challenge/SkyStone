package org.firstinspires.ftc.opmodes.mecanum.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;
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
    private Button armGripServoUp;
    private Button armGripServoDown;

    // Buttons and toggles
    private ToggleBoolean driverTwoBrakes;  //freezes robot in place for stacking, prevents stick bumping from driver one
    private ToggleBoolean driveTelemetry; // changes the display output from driver style telemetry to debugging telemetry
    private Button playSound;

    @Override
    public void init()
    {
        // Misc class declarations
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        elapsedTime = new ElapsedTime();

        // Servo buttons
        platformServoUp = new Button();
        platformServoDown = new Button();
        armGripServoUp = new Button();
        armGripServoDown = new Button();

        // Buttons and toggles
        driverTwoBrakes = new ToggleBoolean(false);
        driveTelemetry = new ToggleBoolean(true);
        playSound = new Button();

        // Stop the sound cause its buggy
        robot.sirenSound.stopSound();
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
        // Movement vector creation
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        // Control inputs
        robot.drivetrain.lowPowerInput(gamepad1.right_stick_button);
        playSound.input(gamepad1.x);

        // Drivetrain updates
        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity * (driverTwoBrakes.output() ? 0 : 1));
        robot.drivetrain.setRotation((-gamepad1.left_stick_x * (3.0/4.0)) * (driverTwoBrakes.output() ? 0.5 : 1));

        // sound
        if (playSound.onPress()) { robot.sirenSound.toggleSound(); }


        /** DRIVER TWO **/
        // Movement overrides
        driverTwoBrakes.input(gamepad2.left_bumper || gamepad2.right_bumper);

        // Servo input
        platformServoUp.input(gamepad2.dpad_up);
        platformServoDown.input(gamepad2.dpad_down);
        armGripServoUp.input(gamepad2.a);
        armGripServoDown.input(gamepad2.y);

        // Servo motion
        if (platformServoUp.onPress()) { robot.platformServo.setPosition(ServoState.UP); }
        else if (platformServoDown.onPress()) { robot.platformServo.setPosition(ServoState.DOWN); }

        if (armGripServoUp.onPress()) { robot.armGripSlide.setPosition(ServoState.UP); }
        else if (armGripServoDown.onPress()) { robot.armGripSlide.setPosition(ServoState.DOWN); }

        // Motor powers
        robot.crane.setVerticalPower(-gamepad2.left_stick_y);
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
