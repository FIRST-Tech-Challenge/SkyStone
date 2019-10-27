package org.firstinspires.ftc.teamcode.Skystone.Auto;

public class BlueLeft extends AutoBase {
    // transport two skystones and other stones if time permits
    @Override
    public void runOpMode() {
        initLogic();
        robot.goToVuforia();
        robot.moveToPoint(0, 30, 1, 1, 180);
    }
}

    // park in building zone (even if other team is in corner)