package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;

@Autonomous(name ="Radiax", group="Playground")
public class RadiaxPlayground extends LinearOpMode {

    DriveTrain drive = new DriveTrain();

    @Override
    public void runOpMode() {

        drive.initDriveTrain(this);
        waitForStart();

        drive.encoderMove(this, 24, 2, 45);

        sleep(1000);
        drive.snowWhite();
    }
}
