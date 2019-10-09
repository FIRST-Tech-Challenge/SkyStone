package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.hardwaremap.MecanumHardwareMap;

@TeleOp(name="MecanumDrivetrain TeleOp", group="TeleOp")
public class MecanumTeleOp extends OpMode
{
    private MecanumHardwareMap robotHardware;
    private MecanumDrivetrain robotDrivetrain;
    private ElapsedTime elapsedTime;

    @Override
    public void init()
    {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        robotDrivetrain = new MecanumDrivetrain(robotHardware.motorList);
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
        robotDrivetrain.setCourse(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2);
        robotDrivetrain.setVelocity(Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y));
        robotDrivetrain.setRotation(-gamepad1.left_stick_x);

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
