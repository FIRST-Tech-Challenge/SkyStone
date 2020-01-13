package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Maccabot;

@Autonomous(name="TankDriveAuto", group="Needham")
//@Disabled
public class TankAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Maccabot robot = new Maccabot(this);

        robot.initializeRobot();

        telemetry.addLine("INIT");

        waitForStart();

        int error = 1000 - robot.getEncoderPositions()[0];
        while (error > 0) {
            error = 1000 - robot.getEncoderPositions()[0];
            telemetry.addLine("MOVING");
            robot.mecanumDrive(0, 0.004*error, 0);
        }
        telemetry.addLine("END");
    }
}
