
package org.firstinspires.ftc.teamcode.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;

@Autonomous(name ="Troll Auto Square", group="Auto Basic")
public class AutoTrollSquare extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;
    private boolean right = true;
    private boolean left = false;

    DriveTrain drive = new DriveTrain();
    Intake intake = new Intake();

    @Override
    public void runOpMode() {

        drive.initDriveTrain(this);
        //intake.initIntakeAuto(this);
        //drive.RunAsFloat();
        waitForStart();

        /*intake.autoIntake(10);
        drive.encoderMove(this, 24, 5, .5);
        sleep(1000);
        drive.strafeMove(this, 24, 5, -1);
        sleep(1000);
        drive.encoderMove(this, 24, 5, -.5);
        sleep(1000);
        drive.strafeMove(this, 24, 5, 1);
        sleep(1000);
*/
        drive.strafeMove(this, 24, 2, .5);
        //drive.partyMode();


       /* drive.initDriveTrain(this);
        waitForStart();

        drive.encoderDrive(this, 1, 24, 24, 3 );
        drive.snowWhite();
        sleep(1000);
        telemetry.addData("1st method ", runtime);
        telemetry.update();

        //drive.encoderDrive(this,  1, 24, 24, 2);
        sleep(1000);
        telemetry.addData("2nd method ", runtime);
        telemetry.update();

        drive.encoderDrive(this, 1, -24, -24, 3);
        drive.snowWhite();
        sleep(1000);
        telemetry.addData("3rd method ", runtime);
        telemetry.update();

        drive.encoderDrive(this, -1, 24, 24, 2);*/
       drive.snowWhite();
    }

}