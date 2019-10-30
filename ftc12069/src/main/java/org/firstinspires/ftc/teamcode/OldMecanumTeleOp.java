package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;

@TeleOp(name="OldMecanumTeleOp", group="Linear Opmode")
public class OldMecanumTeleOp extends OpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;
    private boolean rightMotion = true;
    private boolean servoUp = true;
    private boolean resetServo = true;

    @Override
    public void init()
    {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();

        robotHardware.servoManager.reset();
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
            course = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI/2;
            velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            rotation = gamepad1.left_stick_x;
        } else {
            course = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI/2;
            velocity = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            rotation = gamepad1.right_stick_x;
        }

        robotHardware.drivetrain.setCourse(course);
        robotHardware.drivetrain.setVelocity(velocity);
        robotHardware.drivetrain.setRotation(rotation);

        if (resetServo) {
            robotHardware.servoManager.setPosition(1.0);
        } else if (servoUp) {
            robotHardware.servoManager.setPosition(0.7);
        } else {
            robotHardware.servoManager.setPosition(0.6);
        }

        if (gamepad1.a) rightMotion = false;
        if (gamepad1.b) rightMotion = true;
        if (gamepad1.y) resetServo = true;
        if (gamepad1.dpad_up) servoUp = true;
        if (gamepad1.dpad_down) servoUp = false;

        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", rotation);
        telemetry.addData("Driving Mode", rightMotion ? "RIGHT" : "LEFT");
        telemetry.addData("Servo Pos", servoUp ? "UP (1)" : "DOWN (0)");
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}