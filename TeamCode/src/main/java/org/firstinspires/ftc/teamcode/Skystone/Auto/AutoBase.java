package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

public class AutoBase extends LinearOpMode {
    protected Robot robot;
    protected Vision vision;
    protected long currentTime;
    Position2D position2D;

    public void initLogic() {
        //Init's robot
        robot = new Robot(this.hardwareMap, this.telemetry, this);
        vision = new Vision(this);

        robot.driveMotorsBreakZeroBehavior();
        robot.initServos();

        robot.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        position2D = new Position2D(robot);
    }

    @Override
    public void runOpMode() {
    }

    protected void intake(boolean intake) {
        if (intake) {
            robot.getIntakeLeft().setPower(1);
            robot.getIntakeRight().setPower(1);
        } else {
            robot.getIntakeLeft().setPower(0);
            robot.getIntakeRight().setPower(0);
        }
    }
}



