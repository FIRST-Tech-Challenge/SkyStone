//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//
///**
// * Created by student on 10/26/2017.
// */
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "MissionUnloadBlueLeft", group = "Autonomous")
//public abstract class MissionUnloadBlueLeft extends Autonomous implements ColorSensor {
//    @Override
//    public void runPath() {
//        moveColorSensor(0);
//        while (opModeIsActive()) {
//            if (isBlue() == true){
//
//                move(5, 0.7);
//                move(-5, 0.7);
//            }
//            else if (isRed() == true){
//                move(-5, 0.7);
//                move(5, 0.7); }
//
//        }
//        moveColorSensor(1);
//        move(-24,0.7);
//        pivot(-90,0.7);
//        move(-12,0.7);
//        pivot(90, 0.7);
//        moveArm(1, 1000);
//        move(-7, 0.7);
//
//        sleep(10000);
//
//}}