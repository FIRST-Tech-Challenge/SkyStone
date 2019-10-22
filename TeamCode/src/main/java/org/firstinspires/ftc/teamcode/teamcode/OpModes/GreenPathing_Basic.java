
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
    private ElapsedTime trueTime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    Vuforia vuf = new Vuforia();

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

        drive.initDriveTrain();

        if (robotWidth >= robotLength) {
            greatLength = robotWidth;
        }
        else {
            greatLength = robotLength;
        }

        clearance = (dicot - greatLength) / 2;

        waitForStart();

        trueTime.reset();

        //if (vuf.VuBrowse()[3] == 1) {
            if (shufflePos == 3) {
                drive.encoderMove(this, zeroOffset, 2,  90);

                drive.encoderMove(this, trueHypo, 2,  thetaBit);

                //Intake

                drive.encoderMove(this, -trueHypo, 2,  180 + thetaBit);
            }
            else if (shufflePos == 1) {
                drive.encoderMove(this, zeroOffset, 2,  90);

                drive.encoderMove(this, trueHypo, 2,  360 - thetaBit);

                //Intake

                drive.encoderMove(this, -trueHypo, 2, 180 - thetaBit);
            }
            else {
                drive.encoderMove(this, zeroOffset, 2,  90);

                drive.encoderMove(this, falseHypo, 2,  0);

                //Intake

                drive.encoderMove(this, -falseHypo, 2,  180);
            }
        //}

       // else {
            drive.encoderMove(this, zeroOffset, 2,  90);

            drive.encoderMove(this, falseHypo, 2,  0);

            //Intake

            drive.encoderMove(this, -falseHypo, 2,  180);
        //}

            runtime.reset();

            drive.encoderMove (this, 24, 2, 270);

            valuationMeasure += runtime.milliseconds();

            drive.encoderMove (this, 24 - robotLength, 2, 0);

            drive.turnPID(this,90, false, kP, kI, kD, 2000);

            //outtake.outTake_Auto(drive);

            outtake.hookAuto(drive);

            runtime.reset();

            drive.encoderMove (this, 24, 2, 270);

            drive.encoderMove (this, 12, 2, 180);

            drive.turnPID(this,90, true, kP, kI, kD, 2000);

            valuationMeasure += runtime.milliseconds() + safetyBuffer;

            if (trueTime.milliseconds() < 30 - valuationMeasure) {
                drive.encoderMove(this, 24, 2, 90);

                //if (vuf.VuBrowse()[3] == 1) {
                    if (shufflePos == 3) {
                        drive.encoderMove(this, zeroOffset, 2,  90);

                        drive.encoderMove(this, trueHypo, 2,  thetaBit);

                        //Intake

                        drive.encoderMove(this, -trueHypo, 2,  180 + thetaBit);
                    }
                    else if (shufflePos == 1) {
                        drive.encoderMove(this, zeroOffset, 2,  90);

                        drive.encoderMove(this, trueHypo, 2,  360 - thetaBit);

                        //Intake

                        drive.encoderMove(this, -trueHypo, 2, 180 - thetaBit);
                    }
                    else {
                        drive.encoderMove(this, zeroOffset, 2,  90);

                        drive.encoderMove(this, falseHypo, 2,  0);

                        drive.encoderMove(this, -falseHypo, 2,  180);
                    }
                //}

                //else {
                    drive.encoderMove(this, zeroOffset, 2,  90);

                    drive.encoderMove(this, falseHypo, 2,  0);

                    drive.encoderMove(this, -falseHypo, 2,  180);
                }

                drive.encoderMove(this, 72, 3, 270);

                drive.turnPID(this,90, false, kP, kI, kD, 2000);

                //outtake.outTake_Auto(drive);

                drive.encoderMove(this, 24, 2, 180);
            //}

            sleep(1000);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

