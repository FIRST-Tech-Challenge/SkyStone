package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousConstructionRed", group="Linear Opmode")
public class AutonomousConstructionRed extends Movement {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpModeImpl() {
        // TODO Set up frontServo in movement

        waitForStart();
        runtime.reset();
        //goForward()



    }
}
