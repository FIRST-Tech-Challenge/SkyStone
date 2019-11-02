
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
import org.firstinspires.ftc.teamcode.teamcode.Hardware.ZeroMap;

@Autonomous(name ="AML red Green Path", group="Auto Basic")
public class AMLPathingRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    ZeroMap vuf = new ZeroMap();

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

    @Override
    public void runOpMode() {


        sensors.initSensors(this);

        drive.initDriveTrain(this);
        intake.initIntakeAuto(this);
        outtake.initOuttakeAuto(this);
       // vuf.zeroInit(this);

        if (robotWidth >= robotLength) {
            greatLength = robotWidth;
        }
        else {
            greatLength = robotLength;
        }

        clearance = (dicot - greatLength) / 2;

        waitForStart();


        switch (3) {

            case 3:
                drive.encoderDrive(this, .5, 24, 24, 2);

                drive.gyroTurn(this, 180, true, 2000);
                drive.snowWhite();
                drive.encoderDrive(this, -.5, -24, -24, 2);
                outtake.Auto_Outtake(this);
                drive.encoderDrive(this, .5, 15, 15, 2);
                drive.gyroTurn(this, 90, false, 1000);
                drive.encoderDrive(this, -.5, -72, -72, 2);
                drive.strafeMove(this, 24, 2, .5);

                drive.strafeMove(this, 24, 2, .5);
                drive.encoderDrive(this, .5, 88, 88, 2);
                drive.gyroTurn(this, 90, false, 2000);
                /*outtake.liftLeft.setPower(.5);
                outtake.liftRight.setPower(.5);
                sleep(1000);
                outtake.liftLeft.setPower(0);
                outtake.liftRight.setPower(0);*/
                drive.encoderDrive(this, -.5, -39, -39, 2);
                /*outtake.liftLeft.setPower(-.5);
                outtake.liftRight.setPower(-.5);
                sleep(1000);
                outtake.liftLeft.setPower(0);
                outtake.liftRight.setPower(0);*/
                drive.encoderDrive(this, .5, 15, 15, 2);
                drive.gyroTurn(this, 90, true, 2000);
                drive.encoderDrive(this, -.5, -90, -90, 2);


                break;


            case 2:
                drive.turnPID(this, 90, true, .01, .01, .01, 3000);
                drive.strafeMove(this, 48, 2, .5); //hopefully right
                intake.autoIntake(1);
                drive.strafeMove(this, 8, 1, -.5);

            case 1:
                drive.turnPID(this, 90, true, .01, .01, .01, 3000);
                drive.encoderDrive(this, .5, 16, 16, 2);
                drive.strafeMove(this, 48, 2, .5); //hopefully right
                intake.autoIntake(1);
                drive.strafeMove(this, 24, 2, -.5); //hopefully left
                drive.encoderDrive(this, -.5, 16, 16, 2);
                break;
        }

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

