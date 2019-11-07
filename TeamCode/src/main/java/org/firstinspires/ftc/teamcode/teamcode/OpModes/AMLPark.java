package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;

@Autonomous(name="Park", group="Auto Basic")
public class AMLPark extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain.initDriveTrain(this);

        waitForStart();


        driveTrain.encoderDrive(this, .5, 60, 60, 5);
    }
}
