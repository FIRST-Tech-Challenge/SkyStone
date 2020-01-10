package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueDepotToBridge", group = "Autonomous")
public class BlueDepotToBridge extends Autonomous {
    @Override
    public void runPath() {
        move(30*1.5, 1, 0);
        move(43*1.5, 1, 1);
    }
}