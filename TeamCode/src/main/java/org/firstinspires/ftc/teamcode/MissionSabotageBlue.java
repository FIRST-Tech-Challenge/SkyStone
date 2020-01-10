package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 1/20/2017.
 */
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "MissionSabotageBlue", group = "Autonomous")
public class MissionSabotageBlue extends Autonomous {
    @Override
    protected void runPath() {
        move(30,0.5, 0);
        move(10, 0.2, 0);
        //pivot(-90, 0.5);
    }
}
