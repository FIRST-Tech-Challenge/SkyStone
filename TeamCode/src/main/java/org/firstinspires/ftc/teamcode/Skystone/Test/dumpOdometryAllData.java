package org.firstinspires.ftc.teamcode.Skystone.Test;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Disabled
@TeleOp
public class dumpOdometryAllData extends LinearOpMode {
    Robot robot;
    Position2D position2D;

    @Override
    public void runOpMode() {
        initLogic();

        waitForStart();

        telemetry.addLine("Press A to log data and end program.");
        telemetry.update();

        long startTime = SystemClock.elapsedRealtime();

        position2D.startOdometry();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                break;
            }
        }

        robot.writeToFile("" + startTime, "allData.txt", robot.getOdometryAllData());

        sleep(1000);
    }

    private void initLogic() {
        robot = new Robot(hardwareMap, telemetry, this);

        position2D = new Position2D(robot);
    }
}
