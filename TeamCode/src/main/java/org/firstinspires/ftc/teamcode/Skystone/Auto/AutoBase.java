package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

public class AutoBase extends LinearOpMode {
    protected Robot robot;
    protected Vision vision;
    protected long currentTime;

    public void initLogic() {
        //Init's robot
        robot = new Robot(this.hardwareMap, this.telemetry, this);
        robot.setAutoStopIntake(true);
        vision = new Vision(this);

        robot.driveModule.driveMotorsBrakeOnZero();
        robot.initServos();

        robot.driveModule.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveModule.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hardwareCollection.outtakeSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void runOpMode() {
    }
}



