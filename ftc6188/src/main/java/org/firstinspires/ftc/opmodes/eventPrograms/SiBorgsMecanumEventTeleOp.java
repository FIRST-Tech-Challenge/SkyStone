package org.firstinspires.ftc.opmodes.eventPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@Disabled
@TeleOp(name="Event Mecanum", group="Event")
public class SiBorgsMecanumEventTeleOp extends OpMode
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
    private Button limitSetter;
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
        limitSetter = new Button();
        playSound = new Button();

        // Change low power mode for kids
        robot.drivetrain.setLowPower(0.5);
    }

    @Override
    public void init_loop()
    {
        limitSetter.input(gamepad1.b || gamepad2.b);

        robot.armCrane.setVerticalPower(-gamepad2.left_stick_y);
        robot.armCrane.setHorizontalPower(gamepad2.right_stick_y);

        robot.armCrane.setLimited(false);

        if (limitSetter.onPress()) {
            robot.armCrane.getHorizontalLimitedMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armCrane.getVerticalLimitedMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.armCrane.getHorizontalLimitedMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armCrane.getVerticalLimitedMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Motor Calibration Mode", "Calibrate Slide");
        robot.driverTelemetry();
    }

    @Override
    public void start()
    {
        elapsedTime.reset();

        robot.armCrane.setLimited(true);
        robot.drivetrain.lowPowerInput(true);
    }

    @Override
    public void loop()
    {
        /** DRIVER ONE **/
        // Movement vector creation
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        // Control inputs
        robot.drivetrain.lowPowerInput(gamepad1.right_stick_button);
        playSound.input(gamepad1.x);

        // Drivetrain updates
        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity * (driverTwoBrakes.output() ? 0 : 1));
        robot.drivetrain.setRotation((-gamepad1.left_stick_x * (3.0 / 4.0)) * (driverTwoBrakes.output() ? 0.5 : 1));

        // sound
        if (playSound.onPress()) {
            robot.sirenSound.toggleSound();
        }


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
        robot.armCrane.setVerticalPower(-gamepad2.left_stick_y);
        robot.armCrane.setHorizontalPower(gamepad2.right_stick_y);


        /** TELEMETRY **/
        robot.driverTelemetry();
    }
}