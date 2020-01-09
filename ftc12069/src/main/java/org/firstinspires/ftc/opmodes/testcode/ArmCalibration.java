package org.firstinspires.ftc.opmodes.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Autonomous(name="Arm Calib", group="Autonomous")
public class ArmCalibration extends OpMode {
    private MecanumHardwareMap hardware;
    private ElapsedTime elapsedTime;

    // TeleOp States
    private boolean rightMotion = true;
    private double servoPosition = 0.0;

    // Buttons
    private Button leftBumper;
    private Button rightBumper;
    private Button rightTrigger;
    private Button rightStickButton;
    private Button yButton;
    private Button xButton;

    @Override
    public void init()
    {
        hardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();

        leftBumper = new Button();
        rightBumper = new Button();
        rightTrigger = new Button();
        rightStickButton = new Button();
        yButton = new Button();
        xButton = new Button();

        hardware.deliveryLeft.reset();
        hardware.deliveryRight.reset();
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
        // GAMEPAD 1
        // Movement
        double course;
        double velocity;
        double rotation;
        if (rightMotion) {
            course = -Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
            velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            rotation = -gamepad1.left_stick_x;
        } else {
            course = -Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI/2;
            velocity = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            rotation = -gamepad1.right_stick_x;
        }

        hardware.drivetrain.setCourse(course);
        hardware.drivetrain.setVelocity(velocity, true);
        hardware.drivetrain.setRotation(rotation);

        // Intake
        if (rightTrigger.isToggled()) {
            hardware.intakeMotorManager.setMotorsVelocity(1.0);
        }
        if (rightTrigger.isReleased()) {
            hardware.intakeMotorManager.setMotorsVelocity(0.0);
        }

        // Platform Servos
        if (xButton.isReleased()) {
            if (hardware.platformServoLeft.getPosition() == 0.0) {
                hardware.platformServoLeft.setPosition(1.0);
                hardware.platformServoRight.setPosition(0.0);
            } else {
                hardware.platformServoLeft.setPosition(0.0);
                hardware.platformServoRight.setPosition(1.0);
            }
        }

        //if (gamepad1.a) rightMotion = false;
        //if (gamepad1.b) rightMotion = true;

        rightTrigger.input(gamepad1.right_trigger > 0);
        rightStickButton.input(gamepad1.right_stick_button);
        xButton.input(gamepad1.x);

        // GAMEPAD 2
        if (leftBumper.isReleased()) {
            servoPosition -= 0.01;
            hardware.deliveryLeft.setPosition(servoPosition);
            hardware.deliveryRight.setPosition(servoPosition);
        } else if (rightBumper.isReleased()) {
            servoPosition += 0.01;
            hardware.deliveryLeft.setPosition(servoPosition);
            hardware.deliveryRight.setPosition(servoPosition);
        }

        if (yButton.isReleased()) {
            if (hardware.blockGrabber.getPosition() == ServoState.OPEN.getPosition()) hardware.blockGrabber.setPosition(ServoState.CLOSED);
            else hardware.blockGrabber.setPosition(ServoState.OPEN);
        }

        leftBumper.input(gamepad2.left_bumper);
        rightBumper.input(gamepad2.right_bumper);
        yButton.input(gamepad2.y);

        // TELEMETRY
        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", rotation);
        telemetry.addData("Driving Mode", rightMotion ? "RIGHT" : "LEFT");
        telemetry.addData("Left Delivery Servo State", hardware.deliveryLeft.getPosition());
        telemetry.addData("Right Delivery Servo State", hardware.deliveryRight.getPosition());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
