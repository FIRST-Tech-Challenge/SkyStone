package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedDepotToBridge", group = "Autonomous")
public class RedDepotToBridge extends Autonomous {
    @Override
   public void runPath() {

        move(30, 1, 0);
        move(53, -1, 1);






        /*
        int orientation = getOrientation();

        if (orientation == 1){
            move(5, 0.7, 0);
            //pivot(40, 0.7);
            move(25, 0.7, 0);
           // pivot(70,-0.7);
            move(25, 0.7, 0);
            markerKnock();
            move(5, 0.7, 0);
            move(5, -0.7, 0);
          //  pivot(60, -0.7);
            move(10, 0.7, 0);
          //  pivot(60, -0.7);
            move(75, 0.7, 0);

        }
        else if (orientation == 2){
            move(50, 0.7, 0);
            markerKnock();
           // pivot(50, -0.7);
            move(15, 0.7, 0);
           // pivot(60, -0.7);
            move(75, 0.7, 0);
        }
        else if (orientation == 3) {
            move(5, 0.7, 0);
         //   pivot(40, -0.7);
            move(25, 0.7, 0);
          //  pivot(50,0.7);
            move(25, 0.7, 0);
            markerKnock();
            move(10, -0.7, 0);
          //  pivot(120, -0.7);
            move(10, 0.7, 0);
          //  pivot(60, -0.7);
            move(75, 0.7, 0);

//            pivot(30, -0.7);
//            move(20, 0.7);
//            pivot(30, 0.7);
//            move (27, 0.7);
        }
        else {
            move (27, 0.7, 0);
        }


//        move(10, -0.7);
//        pivot(45,-0.7);
//        move(30,-0.7);


//        move (30, 0.7);
//        //moveMarkerStick(50);//knockem
//        move (-5, -0.7);
//        pivot(95,-0.7);
//        move(65, 0.7);
//        //moveMarkerStick(-50);//close
*/
    }
}