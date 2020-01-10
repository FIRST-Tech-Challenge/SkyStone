package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 2/3/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Do VuforiaDetection", group = "Autonomous")
public class VuforiaDetection extends Autonomous {
    @Override
    public void runPath() {
        final int MAX_ANGLE = 20;
        boolean flag = false;
        int angle = 0;
        int orientation = -1;

        while ((flag==false) && (angle <= MAX_ANGLE) && opModeIsActive()){

            orientation = getOrientation();
            sleep (1000);

            if(orientation == 1){
                telemetry.addData("Position", "left");
                telemetry.update();
                //pivot(-20, 0.7);
                move(10, 0.7, 0);
                flag=true;
            }
            else if(orientation == 2){
                telemetry.addData("Position", "center");
                telemetry.update();
                move(10, 0.7, 0);
                flag=true;
            }
            else if(orientation == 3){
                telemetry.addData("Position", "right");
                telemetry.update();
                //pivot(20, 0.7);
                move(10, 0.7, 0);
                flag=true;
            }
            else if (orientation == 0){
                telemetry.addData("Position","none");
                telemetry.update();
                //pivot(1, 0.5);
                angle += 1;
            }
            else {
                telemetry.addData("Position", "No Return");
                telemetry.update();
            }
    }
       /* final int MAX_ANGLE = 90;

        boolean flag = false;
        int angle = 0;
        int orientation = -1;

//        while (orientation == -1){
//            orientation = getOrientation();
//        }

        telemetry.addData("Position", "out");
        while (((flag == false) && (angle <= MAX_ANGLE) && opModeIsActive())) {
            //(angle <= 20)
            orientation = getOrientation();
            sleep(500);

            if (orientation == 1) {
                telemetry.addData("Position", "Left");
                telemetry.update();
                sleep(1000);
                pivot(-45, 0.7);
                move(5, 0.7);
              //  flag = true;
            } else if (orientation == 2) {
                telemetry.addData("Position", "Center");
                telemetry.update();
                sleep(1000);
                move(5, 0.7);
             //   flag = true;
            } else if (orientation == 3) {
                telemetry.addData("Position", "Right");
                telemetry.update();
                sleep(1000);
                pivot(45, 0.7);
                move(5, 0.7);
              //  flag = true;
            } else {
                telemetry.addData("Position", "none");
                telemetry.update();
                sleep(1000);
                pivot(1, 0.5);
                angle += 1;
            }*/


    //sleep(100000);
}}



