
package org.firstinspires.ftc.teamcode.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;

@Autonomous(name ="Troll Auto Square", group="Auto Basic")
public class AutoTrollSquare extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drive = new DriveTrain();

    @Override
    public void runOpMode() {
        drive.encoderDrive(1, 24, 24, 3 );
        drive.encoderStrafe(true, 1, 24, 24, 2);
        drive.encoderDrive(1, -24, -24, 3);
        drive.encoderStrafe(false, 1, 24, 24, 2);
    }

}