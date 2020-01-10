package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 2/3/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Scan", group = "Autonomous")
public class Scan extends Autonomous {
    @Override
    public void runPath() {

        boolean flag = false;
        boolean left = true;
        int counter = 0;


        while (flag == false) {

            if (getOrientation() == 1) {
                //pivot(45, 0.7);
                flag = true;
            } else if (getOrientation() == 2) {
                move(5, 0.7, 0);
                flag = true;
            } else if (getOrientation() == 3) {
              //  pivot(45, -0.7);
                flag = true;
            } else if (getOrientation() == 0){

                if (counter < 30){
                    if (left == true) {
                       // pivot(1, 0.5);
                        counter = counter + 1;
                    }
                    else{
                      //  pivot (-1, 0.5);
                        counter = counter + 1;
                    }
                }
                else {
                    counter = 0;
                    left = !left;
                }

            }
        }


        //sleep(100000);

    }
}
