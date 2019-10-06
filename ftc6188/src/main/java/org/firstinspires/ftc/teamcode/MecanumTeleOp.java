package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum TeleOp", group="TeleOp")
public class MecanumTeleOp extends OpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;

    @Override
    public void init()
    {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();
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
        double movementSpeed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4;
        double robotRotation = gamepad1.right_stick_x;

        robotHardware.driveFrontLeft.setPower(movementSpeed * Math.cos(robotAngle) + robotRotation);
        robotHardware.driveFrontRight.setPower(movementSpeed * Math.sin(robotAngle) + robotRotation);
        robotHardware.driveRearLeft.setPower(movementSpeed * Math.cos(robotAngle) + robotRotation);
        robotHardware.driveRearRight.setPower(movementSpeed * Math.cos(robotAngle) + robotRotation);

        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
