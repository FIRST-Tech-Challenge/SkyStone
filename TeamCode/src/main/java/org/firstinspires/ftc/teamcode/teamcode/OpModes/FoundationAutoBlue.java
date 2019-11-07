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

@Autonomous(name ="AML Blue Green Foundation Move", group="Auto Basic")
public class FoundationAutoBlue extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private double driveSpeed = 0.6;

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();
    Intake intake = new Intake();
    Outtake out = new Outtake();

    @Override
    public void runOpMode() throws InterruptedException {

        sensors.initSensors(this);
        drive.initDriveTrain(this);
        out.initOuttakeAuto(this);

        waitForStart();

       // sleep(5000);
        out.raiseLiftAuto(this);
        drive.encoderDrive(this, .4, -10, -10, 4);
        drive.strafeMove(this, 20, 5, -.6);
        drive.encoderDrive(this, .4, -60, -60, 4);
        sleep(500);
        out.hookLeft.setPosition(1);
        out.hookRight.setPosition(1);
        sleep(3000);
        drive.encoderDrive(this, .4, 72, 72, 10);
        //drive.strafeMove(this, 40, 5, -.7);
        out.hookLeft.setPosition(0);
        out.hookRight.setPosition(0);
        sleep(500);
        drive.strafeMove(this, 70, 5, .7);
        out.lowerLiftAuto(this);
        drive.strafeMove(this, 40 , 5, .7);


        drive.snowWhite();
    }
}
