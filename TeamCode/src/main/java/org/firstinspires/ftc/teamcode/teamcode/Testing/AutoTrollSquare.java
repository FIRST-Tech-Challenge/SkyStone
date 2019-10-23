
package org.firstinspires.ftc.teamcode.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;

@Autonomous(name ="Troll Auto Square", group="Auto Basic")
public class AutoTrollSquare extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;
    private boolean right = true;
    private boolean left = false;

    DriveTrain drive = new DriveTrain();

    @Override
    public void runOpMode() {

        drive.initDriveTrain();
        waitForStart();

        drive.encoderMove(this, 24, 5, 45);
        drive.encoderMove(this, 24, 5, 225);

       drive.snowWhite();
    }

}