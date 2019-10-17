package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.hardwaremap.HeadingableMecanumHardwareMap;
import org.firstinspires.ftc.robotlib.util.PIDTuner;

@TeleOp (name="Headingable Mecanum PIDTuner", group="Headingable")
public class HeadingableMecanumPIDTuner extends OpMode
{
    private PIDTuner tuner;

    @Override
    public void init()
    {
        HeadingableMecanumHardwareMap robotHardware = new HeadingableMecanumHardwareMap(this.hardwareMap);
        tuner = new PIDTuner(robotHardware.drivetrain, (PIDController) robotHardware.controller.algorithm, this.gamepad1, this.telemetry);
    }

    @Override
    public void loop()
    {
        tuner.update();
    }
}
