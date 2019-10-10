package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.hardwaremap.MecanumHardwareMap;

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
        robotHardware.drivetrain.setCourse(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2);
        robotHardware.drivetrain.setVelocity(Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y));
        robotHardware.drivetrain.setRotation(-gamepad1.left_stick_x);

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
