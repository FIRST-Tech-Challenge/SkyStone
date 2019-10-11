package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Sensors;

@Autonomous(name ="troll Basic Blue Green Path", group="Auto Basic")
public class TrollBotGreenPathing extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();

    @Override
    public void runOpMode() {

        drive.initDriveTrain(this);
        drive.resetEncoders();

        waitForStart();

        //drive.encoderDrive(this,.7, 24, 24, 3);
        switch (1) {
            case 1:
           /*     drive.encoderDrive(this, .7, 24, 24, 3);

                drive.encoderDrive(this, .6, -24, -24,3);

               // drive.encoderStrafe(this, false, .6, 72, 72, 4);

                drive.turnPID(this,180, true, .01, .01, .01, 2000);

                drive.encoderDrive(this,.5, -24, -24, 3);

                drive.turnPID(this,90, false, .01, .01, .01, 2000);

                drive.encoderDrive(this, .5, -10, -10, 3);

               // drive.encoderStrafe(this,true, .6, 48, 48, 4);

                drive.encoderDrive(this,1, 96, 96, 4);

                drive.turnPID(this,90, false, .01, .01, .01, 2000);

                drive.encoderDrive(this,.7, 24, 24, 3);

                drive.encoderDrive(this,.6, -24, -24,3);

              //  drive.encoderStrafe(this,false, .75, 96, 96, 4);

                drive.turnPID(this,180, true, .01, .01, .01, 2000);

                drive.encoderDrive(this,.5, -24, -24, 3);

              */  //drive.turnPID(this,90, false, .01, .01, .01, 2000);
                //rest of code for EVERYTHING
                break;
            case 2:

                //rest of code for EVERYTHING

                break;
            case 3:
                //rest of code for EVERYTHING

                break;
        }


        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
