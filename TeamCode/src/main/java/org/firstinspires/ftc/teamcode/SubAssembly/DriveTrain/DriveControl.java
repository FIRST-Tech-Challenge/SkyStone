package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveControl {
    private LinearOpMode opmode = null;
    private DcMotor FrontRightM = null;
    private DcMotor FrontLeftM = null;
    private DcMotor BackRightM = null;
    private DcMotor BackLeftM = null;

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;
        opMode.telemetry.addLine("Drive Control initialize");
        opMode.telemetry.update();

        opmode = opMode;
        hwMap = opMode.hardwareMap;
        FrontLeftM = hwMap.dcMotor.get("leftFrontMotor");
        FrontRightM = hwMap.dcMotor.get("rightFrontMotor");
        BackLeftM = hwMap.dcMotor.get("leftRearMotor");
        BackRightM = hwMap.dcMotor.get("rightRearMotor");

        // Set the drive motor direction
        FrontLeftM.setDirection(DcMotor.Direction.FORWARD);
        FrontRightM.setDirection(DcMotor.Direction.REVERSE);
        BackLeftM.setDirection(DcMotor.Direction.FORWARD);
        BackRightM.setDirection(DcMotor.Direction.REVERSE);

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        //setting power now means setting the speed
        FrontLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRightM.setPower(0);
        FrontLeftM.setPower(0);
        BackRightM.setPower(0);
        BackLeftM.setPower(0);
    }

    public void joystick(double LF, double RF, double LR, double RR) {
        FrontLeftM.setPower(LF);
        FrontRightM.setPower(RF);
        BackLeftM.setPower(LR);
        BackRightM.setPower(RR);
    }
}
