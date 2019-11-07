
package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name ="AML red Green Path", group="Auto Basic")
public class AMLPathingRed extends LinearOpMode {

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
    double offset = 0;

    //Shuffle Position in Vu
    int shufflePos;

    @Override
    public void runOpMode() throws InterruptedException {


        sensors.initSensors(this);

        drive.initDriveTrain(this);
        intake.initIntakeAuto(this);
        outtake.initOuttakeAuto(this);
        //vuf.initVision(this);

        if (robotWidth >= robotLength) {
            greatLength = robotWidth;
        }
        else {
            greatLength = robotLength;
        }

        clearance = (dicot - greatLength) / 2;

        /*if(vuf.sense(this) == "left")
            offset = -16;
        else if(vuf.sense(this) == "right")
            offset = 16;
        else
            offset = 0;
*/

        waitForStart();

        waitForStart();
        /*if(vuf.senseBlue(this) == "left")
            offset = 20;
        else if(vuf.senseBlue(this) == "right")
            offset = -20;
        else
            offset = 0;
*/
        offset = 0;

        //lift up
        outtake.raiseLiftAuto(this);

        //lift out
        outtake.rightVex.setPower(.5);
        outtake.leftVex.setPower(-.5);
        sleep(5500);
        outtake.leftVex.setPower(0);
        outtake.rightVex.setPower(0);

       // drive.strafeMove(this, offset, 5, .7);

        //drive to block
        drive.encoderDrive(this, -.7, -50.5, -50.5, 5);
        //sleep(1000);

        //lift down
        outtake.lowerLiftAuto(this);

        //tighten
        outtake.rightVex.setPower(-.5);
        outtake.leftVex.setPower(.5);
        sleep(3000);
        outtake.leftVex.setPower(0);
        outtake.rightVex.setPower(0);

        //drive back
        drive.encoderDrive(this, .6, 14, 14, 5);
        //sleep(1000);

        //strafe across bridge
        drive.strafeMove(this, 100 - offset, 10, 1);
//        sleep(1000);

        //loosen
        outtake.rightVex.setPower(.5);
        outtake.leftVex.setPower(-.5);
        sleep(1500);
        outtake.leftVex.setPower(0);
        outtake.rightVex.setPower(0);

        //lift up
        outtake.raiseLiftAuto(this);

        drive.gyroTurn(this, 0, true, 4000);


        //drive out of way
        drive.encoderDrive(this, .7, 13, 13, 5);
        //      sleep(1000);


        //lift down
        outtake.lowerLiftAuto(this);

        drive.strafeMove(this, 20, 10, -.8);

        drive.encoderDrive(this, -.5, -20, -20, 3);

       /* //strafe to stone 2
        drive.strafeMove(this, 144 - offset, 10, -.8);
        //drive.gyroTurn(this, 0, false, 4000);

        //drive.gyroTurn(this, 360, false , 300);

        //lift up
        outtake.raiseLiftAuto(this);

        sleep(500);
        //drive to stone
        drive.encoderDrive(this, -.7, -40, -40, 5);

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
        //sleep(1000);

        //strafe across bridge
        drive.strafeMove(this, 146 - offset, 10, 1);

        //raise lift
        outtake.raiseLiftAuto(this);

        drive.strafeMove(this, 5 - offset, 10, -1);

        */
       /*move back
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

