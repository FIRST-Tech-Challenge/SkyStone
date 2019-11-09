package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueLeft", group ="LinearOpmode")
public class BlueLeft extends AutoBase {
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        initLogic();

        // Determine position of SkyStone
        int vuforiaPosition = robot.detectTensorflow();

        // Go to the Skystone and intake it
        goToSkystone(vuforiaPosition,0);

        // Move to the other side of the skybridge
        robot.moveToPoint(0, 47, 1, 0, Math.toRadians(20));
        telemetry.addLine("return");
        telemetry.update();
        telemetry.addLine("DONEEEEE");
        telemetry.update();

        // Deposit the stone, then retract the outtake
        depositStone(robot);
        retractOuttake(robot);

        // Move to the second set of three stones, in anticipation of picking up the second Skystone
        robot.moveToPoint(0,24,1,1,Math.toRadians(0));

        // Go to the Skystone and intake it
        goToSkystone(vuforiaPosition,1);

        // Move to the other side of the Skybridge
        robot.moveToPoint(0, 47, 1, 0, Math.toRadians(20));


        // Deposit the Skystone and retract the outtake arm
        depositStone(robot);
        retractOuttake(robot);

        // Return for more Stones
        // TODO: tensorflow for more stones
    }
}