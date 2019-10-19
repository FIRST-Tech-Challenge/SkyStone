
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

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    Vuforia vuf = new Vuforia();

    double robotLength;
    double focusLine = 48 - robotLength;
    double clutchBuffer = 3.5;
    double thetaBit = (Math.atan(12/focusLine)) / 2;
    double trueHypo = (focusLine - clutchBuffer) / Math.cos(thetaBit);
    double falseHypo = focusLine - clutchBuffer;

    int shufflePos;

    @Override
    public void runOpMode() {

        drive.initDriveTrain(this);

        waitForStart();

        //!!!ADD INTAKE!!!
        if (vuf.VuBrowse()[3] == 1) {
            if (shufflePos == 3) {
                drive.encoderMove(this, trueHypo, 2, 1, thetaBit);

                drive.encoderMove(this, -trueHypo, 2, 1, 180 + thetaBit);
            }
            else if (shufflePos == 1) {
                drive.encoderMove(this, trueHypo, 2, 1, 360 - thetaBit);

                drive.encoderMove(this, -trueHypo, 2, 1, 180 - thetaBit);
            }
            else {
                drive.encoderMove(this, falseHypo, 2, 1, 0);

                drive.encoderMove(this, -falseHypo, 2, -1, 180);
            }
        }

            drive.encoderDrive(this, .7, 24, 24, 3);

            drive.encoderDrive(this, .6, -24, -24,3);

            drive.turnPID(this,180, true, .01, .01, .01, 2000);

            drive.encoderDrive(this,.5, -24, -24, 3);

            drive.turnPID(this,90, false, .01, .01, .01, 2000);

            drive.encoderDrive(this, .5, -10, -10, 3);

            drive.encoderDrive(this,1, 96, 96, 4);

            drive.turnPID(this,90, false, .01, .01, .01, 2000);

            drive.encoderDrive(this,.7, 24, 24, 3);

            drive.encoderDrive(this,.6, -24, -24,3);

            drive.turnPID(this,180, true, .01, .01, .01, 2000);

            drive.encoderDrive(this,.5, -24, -24, 3);

            drive.turnPID(this,90, false, .01, .01, .01, 2000);

            sleep(1000);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

