package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.monitor.MonitorIMU;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;

import java.util.Locale;

@Autonomous(name="help me", group="Linear Opmode")
public class TestAuto extends AutoOpMode {
    @Override
    public void preInit() {
        super.preInit();
        //if you're pro, do this
        //driver.setTest(false);
    }

    @Override
    public void beforeLoop() {
        for(DcMotor motor : driver.getMotors())
            telemetry.addLine("Motor: " + motor.getCurrentPosition());
        float[] floats = MonitorIMU.getXYZAngle();
        telemetry.addLine(String.format(Locale.ENGLISH, "Angles: %f %f %f", floats[0], floats[1], floats[2]));
        telemetry.update();

    }

    @Override
    public void run() {
        driver.turn(0.5, 180, this);
        driver.turn(0.5, 90, this);
    }
}
