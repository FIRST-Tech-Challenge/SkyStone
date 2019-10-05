
package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Vuforia;

@Autonomous(name ="Basic Blue Green Path", group="Auto Basic")
public class GreenPathing_Basic extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drivetrain = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    Vuforia vuf = new Vuforia();

    @Override
    public void runOpMode() {

        drivetrain.initDriveTrain(this);
        drivetrain.initDriveTrain(this);
        drivetrain.resetEncoders();

        intake.initIntake(this);
        outtake.initOuttake(this);
        vuf.initVuforia(this);
        vuf.initVuforia(this);
        waitForStart();

       /* drivetrain.encoderDrive(this, driveSpeed,  10,   10);

        switch (vuf.senseSkystone(this)) {
            case 1:
                drivetrain.encoderDrive(this, driveSpeed,  5,   5);
                intake.compliantIntake_Auto(drivetrain,1, true);
                //rest of code for EVERYTHING
                break;
            case 2:
                drivetrain.encoderDrive(this, driveSpeed,  5,   5);
                intake.compliantIntake_Auto(drivetrain,1, true);

                //rest of code for EVERYTHING

                break;
            case 3:
                drivetrain.encoderDrive(this, driveSpeed,  5,   5);
                intake.compliantIntake_Auto(drivetrain,1, true);

                //rest of code for EVERYTHING

                break;
                */

        }

        //intake.compliantIntake_Auto(drivetrain,1, true);

        //drivetrain.encoderDrive(this, driveSpeed, -48,  2);
        //drivetrain.encoderDrive(this, driveSpeed,144,  6);
        //drivetrain.encoderDrive(this, driveSpeed,-96,  4);

        //sleep(1000);

        //telemetry.addData("Path", "Complete");
        //telemetry.update();
    }

