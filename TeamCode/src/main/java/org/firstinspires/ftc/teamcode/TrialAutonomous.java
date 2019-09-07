package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Trial Autonomous", group="Trial")
public class TrialAutonomous extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare hardware mapping
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection((DcMotor.Direction.FORWARD));

        waitForStart();

        //List robot instructions here
        driveForward(1, 5000);
        turnLeft(1, 5000);
        turnRight(1, 5000);
        driveBackward(1, 5000);
        stopDriving();
    }
        // Methods

        public void driveForward(double power, int time)
        {
            leftDrive.setPower(power);
            rightDrive.setPower(power);
            sleep(time);
        }
        public void turnLeft(double power, int time)
        {
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
            sleep(time);
        }
        public void turnRight(double power, int time)
        {
            turnLeft(-power, time);
        }
        public void driveBackward(double power, int time)
        {
            driveForward(-power, time);
        }
        public void stopDriving()
        {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }





}
