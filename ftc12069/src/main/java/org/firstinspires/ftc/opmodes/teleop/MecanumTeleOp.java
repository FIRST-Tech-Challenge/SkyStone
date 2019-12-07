package org.firstinspires.ftc.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@TeleOp(name="Mecanum TELEOP (12069)", group="Linear Opmode")
public class MecanumTeleOp extends OpMode
{
    private MecanumHardwareMap hardware;
    private ElapsedTime elapsedTime;

    // TeleOp States
    private boolean rightMotion = true;
    private Button rightTrigger;
    private Button rightStickButton;
    private Button yButton;
    private Button xButton;

    // Servo
    double[] servoPositions = new double[]{0.07, 0.15, 0.71, 0.78, 0.89, 1.0};
    String[] positionNames = new String[]{"Cradle", "Carry", "Hover2", "Deposit2/Hover1", "Deposit1", "Floor"};
    int servoPositionIndex = 0;
    private Button leftBumper;
    private Button rightBumper;

    @Override
    public void init()
    {
        hardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();

        yButton = new Button();
        xButton = new Button();
        leftBumper = new Button();
        rightBumper = new Button();
        rightTrigger = new Button();
        rightStickButton = new Button();

        //hardware.deliveryServoManager.reset();
        hardware.platformServoManagers.reset();
        hardware.intakeMotorManager.stop();
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
        hardware.drivetrain.setVelocity(velocity);
        hardware.drivetrain.setRotation(rotation);

        if (rightBumper.isReleased()) {
            if (servoPositionIndex + 1 < servoPositions.length) {
                servoPositionIndex++;
                hardware.deliveryLeft.setPosition(servoPositions[servoPositionIndex]);
                hardware.deliveryRight.setPosition(servoPositions[servoPositionIndex] + 0.04);
            }
        }

        if (leftBumper.isReleased()) {
            if (servoPositionIndex > 0) {
                servoPositionIndex--;
                hardware.deliveryLeft.setPosition(servoPositions[servoPositionIndex]);
                hardware.deliveryRight.setPosition(servoPositions[servoPositionIndex] + 0.04);
            }
        }

        if (yButton.isReleased()) {
            if (hardware.blockGrabber.getPosition() == 0.0) hardware.blockGrabber.setPosition(1.0);
            else hardware.blockGrabber.setPosition(0.0);
        }

        if (xButton.isReleased()) {
            if (hardware.platformServoManagers.getPosition() == 0.0) hardware.platformServoManagers.setPosition(1.0);
            else hardware.platformServoManagers.setPosition(0.0);
        }

        /*ServoState deliveryServoState = hardware.deliveryServoManager.getServoState();
        if (leftBumper.isReleased()) {
            hardware.deliveryServoManager.setServoState(ServoState.getServoStateFromInt(deliveryServoState.getId() - 1));
        } else if (rightBumper.isReleased()) {
            hardware.deliveryServoManager.setServoState(ServoState.getServoStateFromInt(deliveryServoState.getId() + 1));
        }*/

        if (rightTrigger.isToggled()) {
            hardware.intakeMotorManager.setMotorsVelocity(1.0);
        }
        if (rightTrigger.isReleased()) {
            hardware.intakeMotorManager.setMotorsVelocity(0.0);
        }

        if (rightStickButton.isReleased()) {
            if (hardware.blockGrabber.getPosition() == 0.0) hardware.blockGrabber.setPosition(0.5);
            else hardware.blockGrabber.setPosition(0.0);
        }

        //if (gamepad1.a) rightMotion = false;
        //if (gamepad1.b) rightMotion = true;

        xButton.input(gamepad1.x);
        yButton.input(gamepad1.y);
        leftBumper.input(gamepad1.left_bumper);
        rightBumper.input(gamepad1.right_bumper);
        rightTrigger.input(gamepad1.right_trigger > 0);
        rightStickButton.input(gamepad1.right_stick_button);

        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", rotation);
        telemetry.addData("Driving Mode", rightMotion ? "RIGHT" : "LEFT");
        telemetry.addData("Servo State", servoPositionIndex + " " + positionNames[servoPositionIndex] + " " + servoPositions[servoPositionIndex]);
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
