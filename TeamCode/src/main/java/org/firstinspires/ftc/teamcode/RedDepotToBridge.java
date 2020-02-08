package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedDepotToBridge", group = "Autonomous")
public class RedDepotToBridge extends Autonomous {
    @Override
    public void runPath() {
        slideMove(0.5);
       /* if(getPosition()){
            move(5, 1, 0);

        }
        else {
            sleep(300000000);
        }*/

        move(33,-1,1); //left //was 46
    }
}