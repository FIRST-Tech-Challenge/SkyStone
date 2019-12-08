package org.firstinspires.ftc.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

import static org.firstinspires.ftc.robotlib.state.ServoState.CARRY;
import static org.firstinspires.ftc.robotlib.state.ServoState.CRADLE;
import static org.firstinspires.ftc.robotlib.state.ServoState.FLOOR;
import static org.firstinspires.ftc.robotlib.state.ServoState.ONEBLOCKDEPOSIT;
import static org.firstinspires.ftc.robotlib.state.ServoState.TWOBLOCKDEPOSIT;
import static org.firstinspires.ftc.robotlib.state.ServoState.TWOBLOCKHOVER;

@TeleOp(name="Experimental Mecanum TELEOP (12069)", group="Linear Opmode")
public class MecanumTeleOp extends OpMode
{
    private MecanumHardwareMap hardware;
    private ElapsedTime elapsedTime;

    // TeleOp States
    private boolean rightMotion = true;

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

        hardware.deliveryLeft.reset();
        hardware.deliveryRight.reset();
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

        if (leftBumper.isReleased()) {
            hardware.deliveryLeft.decrement();
            hardware.deliveryRight.decrement();
        } else if (rightBumper.isReleased()) {
            hardware.deliveryLeft.increment();
            hardware.deliveryRight.increment();
        }

        if (rightTrigger.isToggled()) {
            hardware.intakeMotorManager.setMotorsVelocity(1.0);
        }
        if (rightTrigger.isReleased()) {
            hardware.intakeMotorManager.setMotorsVelocity(0.0);
        }

        if (rightStickButton.isReleased()) {
            if (hardware.blockGrabber.getPosition() == 0.0) hardware.blockGrabber.setPosition(1.0);
            else hardware.blockGrabber.setPosition(0.0);
        }

        //if (gamepad1.a) rightMotion = false;
        //if (gamepad1.b) rightMotion = true;

        leftBumper.input(gamepad1.left_bumper);
        rightBumper.input(gamepad1.right_bumper);
        rightTrigger.input(gamepad1.right_trigger > 0);
        rightStickButton.input(gamepad1.right_stick_button);
        yButton.input(gamepad1.y);
        xButton.input(gamepad1.x);

        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", rotation);
        telemetry.addData("Driving Mode", rightMotion ? "RIGHT" : "LEFT");
        telemetry.addData("Left Delivery Servo State", hardware.deliveryLeft.getState().toString());
        telemetry.addData("Right Delivery Servo State", hardware.deliveryRight.getState().toString());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
