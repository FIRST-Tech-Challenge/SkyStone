//package org.firstinspires.ftc.teamcode;
//
///**
// * Created by student on 10/26/2017.
// */
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "MissionUnloadBlueRight", group = "Autonomous")
//public class  MissionUnloadBlueRight extends Autonomous {
//    @Override
//    public void runPath(){
//
//        moveColorSensor(0);
//        while (opModeIsActive()) {
//            if (isRed() == true){
//
//                move(5, 0.7);
//                move(-5, 0.7);
//            }
//            else if (isBlue() == true){
//                move(-5, 0.7);
//                move(5, 0.7);
//            }
//
//        }
//        moveColorSensor(1);
//        move(-24,0.7);
//        pivot(90,0.7);
//        move(-12,0.7);
//        moveArm(1, 1000);
//        move(-7, 0.7);
//
//        sleep(10000);
//        }}