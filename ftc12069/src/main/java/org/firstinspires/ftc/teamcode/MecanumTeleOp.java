package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;
import org.firstinspires.ftc.robotlib.state.ServoState;

@TeleOp(name="Basic Mecanum (12069)", group="Linear Opmode")
public class MecanumTeleOp extends OpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;
    private boolean rightMotion = true;
    private ServoState servoState = ServoState.STOWED;

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

        switch (servoState) {
            case STOWED:
                robotHardware.servoManager.setPosition(1.0);
            case UP:
                robotHardware.servoManager.setPosition(0.9);
            case DOWN:
                robotHardware.servoManager.setPosition(0.6);
        }

        if (gamepad1.a) rightMotion = false;
        if (gamepad1.b) rightMotion = true;
        if (gamepad1.dpad_up) {
            servoState = ServoState.getServoStateFromInt(servoState.getLevel() + 1);
        }
        if (gamepad1.dpad_down) {
            servoState = ServoState.getServoStateFromInt(servoState.getLevel() - 1);
        }

        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", rotation);
        telemetry.addData("Driving Mode", rightMotion ? "RIGHT" : "LEFT");
        telemetry.addData("Servo Pos", servoState.toString() + " (" + servoState.getLevel() + ")");
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
