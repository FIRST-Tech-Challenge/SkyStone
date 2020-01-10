package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Last Minute Vuforia", group = "Autonomous")
public class LastMinuteVuforia extends Autonomous {
    @Override
    public void runPath() {

        int orientation = getOrientation();

        if (orientation == 1){
            move (10, -0.7, 0);
        }
        else if (orientation == 2){
            //pivot(10, -0.7);
            move (10, 0.7, 0);
        }
        else if (orientation == 3) {
            //pivot(20, -0.7);
            move(10, 0.7, 0);
        }
        else if (orientation == 10){
           // pivot(20, -0.7);
            //pivot (-20, 0.7);
        }
        else {
           // pivot (360, 0.7);
        }
    }
}