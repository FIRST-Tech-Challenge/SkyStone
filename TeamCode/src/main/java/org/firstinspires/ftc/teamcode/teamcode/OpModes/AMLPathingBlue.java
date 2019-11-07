
package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.VisionWebcam;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.ZeroMap;

@Autonomous(name ="AML Blue Green Path", group="Auto Basic")
public class AMLPathingBlue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    VisionWebcam vuf = new VisionWebcam();

    public BNO055IMU gyro;
    public Orientation angles;

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

    double offset = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        sensors.initSensors(this);

        drive.initDriveTrain(this);
        intake.initIntakeAuto(this);
        outtake.initOuttakeAuto(this);
        vuf.initVision(this);

        if (robotWidth >= robotLength) {
            greatLength = robotWidth;
        }
        else {
            greatLength = robotLength;
        }

        clearance = (dicot - greatLength) / 2;



        waitForStart();
        if(vuf.senseBlue(this) == "left")
            offset = 20;
        else if(vuf.senseBlue(this) == "right")
            offset = -20;
        else
            offset = 0;


        //lift up
        outtake.raiseLiftAuto(this);

        //lift out
        outtake.rightVex.setPower(.5);
        outtake.leftVex.setPower(-.5);
        sleep(5000);
        outtake.leftVex.setPower(0);
        outtake.rightVex.setPower(0);

        drive.encoderDrive(this, -.7, -10, -10, 5);


        if(offset == -20) {
            drive.strafeMove(this, 20, 5, .5);
        }
        else if(offset == 20) {
            drive.strafeMove(this, 20, 5, -.5);
        }


        //drive to block
        drive.encoderDrive(this, -.7, -40, -40, 5);
        //sleep(1000);

        //lift down
        outtake.lowerLiftAuto(this);

        //tighten
        outtake.rightVex.setPower(-.5);
        outtake.leftVex.setPower(.5);
        sleep(2250);
        outtake.leftVex.setPower(0);
        outtake.rightVex.setPower(0);

        //drive back
        drive.encoderDrive(this, .6, 12, 12, 5);
        //sleep(1000);

        //strafe across bridge
        drive.strafeMove(this, 100 - offset, 10, -1);
//        sleep(1000);

        //loosen
        outtake.rightVex.setPower(.5);
        outtake.leftVex.setPower(-.5);
        sleep(1500);
        outtake.leftVex.setPower(0);
        outtake.rightVex.setPower(0);

        //lift up
        outtake.raiseLiftAuto(this);

        drive.gyroTurn(this, 360, false, 4000);


        //drive out of way
        drive.encoderDrive(this, .7, 12, 12, 5);
        //      sleep(1000);


        //lift down
        outtake.lowerLiftAuto(this);

        //strafe to stone 2
        drive.strafeMove(this, 40, 10, .5);
        //drive.gyroTurn(this, 0, false, 4000);



       /* //lift up
        outtake.raiseLiftAuto(this);

        sleep(500);
        //drive to stone
        drive.encoderDrive(this, -.7, -37.5, -37.5, 5);

        //lift down
        outtake.lowerLiftAuto(this);

        //tighten
        outtake.rightVex.setPower(-.5);
        outtake.leftVex.setPower(.5);
        sleep(1000);
        outtake.leftVex.setPower(0);
        outtake.rightVex.setPower(0);

        //drive back
        drive.encoderDrive(this, .7, 20, 20, 5);
        sleep(1000);

        //strafe across bridge
        drive.strafeMove(this, 145 - offset, 10, -1);

        //raise lift
        outtake.raiseLiftAuto(this);

        drive.strafeMove(this, 7 - offset, 10, 1);

        *//*//*/ /*
        drive.encoderDrive(this, .5, 12, 12, 5);
    //    sleep(1000);

        //lower lift
        outtake.lowerLiftAuto();

        //park
        drive.strafeMove(this, -24, 2, 5);



*/

        /*drive.encoderDrive(this, -.5, 24, 24, 2);
        drive.strafeMove(this, 24, 2, -.5); //hopefully left
        outtake.hookLeft.setPosition(1);
        outtake.hookRight.setPosition(1);
        drive.strafeMove(this, 36, 2, .5);
        outtake.hookLeft.setPosition(0);
        outtake.hookRight.setPosition(0);
        //outtake.outTake_Auto(this);
        drive.encoderDrive(this, .5, 24, 24, 2);


*/
    }
}

