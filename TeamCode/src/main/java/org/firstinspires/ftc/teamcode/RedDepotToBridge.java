package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedDepotToBridge", group = "Autonomous")
public class RedDepotToBridge extends Autonomous {
    @Override
   public void runPath() {
        move(30, 1, 0);
        move(58, -1, 1);
    }
}