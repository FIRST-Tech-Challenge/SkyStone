package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Vuforia;

@Autonomous(name = "MUSMAA", group= "Phase Infinity")
public class MUSMAA extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime vortex = new ElapsedTime();

    private double driveSpeed = 0.6;
    int constantBufferTime = 5;
    double location;
    int angleCount = 360000;

    //Add marginal if
    boolean theGreenSide = true;

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    Vuforia vuf = new Vuforia();

    public double[] miasmat (double[] dubSet) {
        if (vortex.seconds() >= 10) {
            vortex.reset();
            dubSet[angleCount] = 1;
        }
        if (dubSet[angleCount] == 1) {
            drive.encoderDrive(this, driveSpeed, 24, -24, 4);
            location = sensors.getGyroYaw();
            sleep(5);
            while (sensors.getGyroYaw() != location) {
                for (int i = 1; i == angleCount; i++) {
                    dubSet[i - 1] = sensors.getDist();
                }
            }
            dubSet[angleCount] = 0;
        }
        return dubSet;
    }

    @Override
    public void runOpMode() {
        double[] USM;
        USM = new double[angleCount + 1];
        USM[angleCount] = 1;
        while (opModeIsActive() ) {
            //change to a non-constant buffer time -->\
            while (runtime.milliseconds() >= 30 - constantBufferTime) {
                miasmat(USM);
                if (theGreenSide) {

                }
            }

        }
    }
}
