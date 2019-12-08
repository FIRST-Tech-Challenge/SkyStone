package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

@Autonomous(name="RedRight", group ="LinearOpmode")
public class RedRight extends AutoBase {
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        initLogic();

        waitForStart();
        while (opModeIsActive()) {
//            if (robot.getTrayDistance().getDistance(DistanceUnit.CM) < 6) {
//                robot.intake(false);
//            } else {
//                robot.intake(true);
//            }
        }

    }
}