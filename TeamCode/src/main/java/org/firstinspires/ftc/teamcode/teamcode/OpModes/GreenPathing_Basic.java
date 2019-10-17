
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

    @Override
    public void runOpMode() {

        drive.initDriveTrain(this);

        waitForStart();

        drive.encoderMove(this, 24, 3, .7);
        switch (1) {
            case 1:
                drive.encoderMove(this, 24, 2, .9);

                drive.encoderMove(this, 24, 2, -.9);

                drive.turnPID(this,180, true, .01, .01, .01, 2000);

                drive.encoderMove(this, 24, 3, -.5);

                drive.turnPID(this,90, false, .01, .01, .01, 2000);

                drive.encoderMove(this, 10, 3, -.5);

                drive.strafeMove(this, 48, 4, .6);

                drive.encoderMove(this, 96, 4, 1);

                drive.turnPID(this,90, false, .01, .01, .01, 2000);

                drive.encoderMove(this, 24, 3, .7);

                drive.encoderMove(this, 24, 3, -.6);

                drive.strafeMove(this, 96, 4, -.75);

                drive.turnPID(this,180, true, .01, .01, .01, 2000);

                drive.encoderMove(this, 24, 3, -.5);

                drive.turnPID(this,90, false, .01, .01, .01, 2000);
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
