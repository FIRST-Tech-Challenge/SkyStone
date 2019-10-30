
package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.ZeroMap;

@Autonomous(name ="AML Blue Green Path", group="Auto Basic")
public class AMLPathingBlue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime trueTime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    ZeroMap vuf = new ZeroMap();

    double zeroOffset = 5.75;
    double kI;
    double kP;
    double kD;

    double valuationMeasure;
    double safetyBuffer;

    double robotLength = 17.1;
    double robotWidth = 18.1;
    double greatLength;
    double dicot = Math.sqrt((robotLength * robotLength) +
            (robotWidth * robotWidth));
    double clearance;

    double focusLine = 48 - robotLength;
    double clutchBuffer = 3.5;
    double thetaBit = (Math.atan(12/focusLine)) / 2;
    double trueHypo = (focusLine - clutchBuffer) / Math.cos(thetaBit);
    double falseHypo = focusLine - clutchBuffer;

    //Shuffle Position in Vu
    int shufflePos;

    @Override
    public void runOpMode() {


        sensors.initSensors(this);
        drive.initDriveTrain(this);
        intake.initIntakeAuto(this);
        outtake.initOuttakeAuto(this);
        vuf.zeroInit(this);

        if (robotWidth >= robotLength) {
            greatLength = robotWidth;
        }
        else {
            greatLength = robotLength;
        }

        clearance = (dicot - greatLength) / 2;

        waitForStart();

        trueTime.reset();

       // vuf.zeroOut();

        switch (3) {

            case 3:
                drive.turnPID(this, 90, true, .01, .01, .01, 3000);
                drive.encoderDrive(this, -1, 24, 24, 2);
                drive.strafeMove(this, 48, 2, 1); //hopefully right
                intake.autoIntake(1);
                drive.strafeMove(this, 24, 2, -1); //hopefully left
                drive.encoderDrive(this, 1, 24, 24, 2);

                break;


            case 2:
                drive.turnPID(this, 90, true, .01, .01, .01, 3000);
                drive.strafeMove(this, 48, 2, 1); //hopefully right
                intake.autoIntake(1);
                drive.strafeMove(this, 8, 1, -.7);

            case 1:
                drive.turnPID(this, 90, true, .01, .01, .01, 3000);
                drive.encoderDrive(this, 1, 16, 16, 2);
                drive.strafeMove(this, 48, 2, 1); //hopefully right
                intake.autoIntake(1);
                drive.strafeMove(this, 24, 2, -1); //hopefully left
                drive.encoderDrive(this, -1, 16, 16, 2);
                break;
        }

        drive.encoderMove(this, 72, 3, -1);
        drive.strafeMove(this, 24, 2, -1); //hopefully left
        outtake.hookAuto(this);
        outtake.outTake_Auto(this);
        drive.encoderMove(this, 24, 2, 1);


    }
}

