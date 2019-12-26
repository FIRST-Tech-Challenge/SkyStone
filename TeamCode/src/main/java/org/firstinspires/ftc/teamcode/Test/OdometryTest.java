package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Odometry Test", group = "Smart")
public class OdometryTest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Crane.setupType.encoder);

        int startPosition = rob.rightSuck.getCurrentPosition();
        while(opModeIsActive()){

            telemetry.addData("Current Position: ", rob.rightSuck.getCurrentPosition() - startPosition);
            telemetry.update();
        }
    }
}
