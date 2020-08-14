package org.firstinspires.ftc.teamcode.Skystone.Test;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.ThreadLoop;

@Deprecated
@TeleOp
@Disabled

public class TestOdo extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        initLogic();
        ThreadLoop t = new ThreadLoop(robot,robot.linearOpMode);

        waitForStart();
        t.run();

        while (opModeIsActive()) {
            telemetry.addLine("x: " + robot.odometryModule.worldX);
            telemetry.addLine("y: " + robot.odometryModule.worldY);
            telemetry.addLine("ang: " + Math.toDegrees(robot.odometryModule.worldAngle));
        }

        t.terminate();
    }

    private void initLogic() {
        robot = new Robot(hardwareMap, telemetry, this);
    }
}
