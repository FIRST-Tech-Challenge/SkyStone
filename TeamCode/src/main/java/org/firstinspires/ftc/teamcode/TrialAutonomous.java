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


    }
        // Methods

        public void driveForward(double power)
        {
            leftDrive.setPower(power);
            rightDrive.setPower(power);
        }
        public void turnLeft(double power)
        {
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
        }
        public void turnRight(double power)
        {
            turnLeft(-power);
        }
        public void driveBackward(double power)
        {
            driveForward(-power);
        }
        public void stopDriving()
        {
            driveForward(0);
        }





}
