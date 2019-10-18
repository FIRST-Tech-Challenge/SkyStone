//package org.firstinspires.ftc.teamcode.mainops;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.libraries.AutoLib;
//import org.firstinspires.ftc.teamcode.libraries.Constants;
//
///*
// * Title: AutoBlueCraterBase
// * Date Created: 11/23/2018
// * Date Modified: 2/22/2019
// * Author: Rahul, Poorvi, Varnika
// * Type: Main
// * Description: Starts on blue crater latcher
// */
//
//@Autonomous(group = "Main")
//public class AutoCraterBase extends LinearOpMode {
//    private AutoLib autoLib;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initialize();
////        autoLib.landOnGround();
//
//
//        // Tensorflow
//        Constants.GoldObjectPosition gold = autoLib.readGoldObjectPosition();
//
//        if (gold == Constants.GoldObjectPosition.LEFT) {
//            autoLib.calcMove(5,.1f, Constants.Direction.RIGHT);
////            telemetry.addData("pos", "Left");
////            autoLib.calcMove(10, .8f);
////            autoLib.calcTurn(45, .8f);
////            autoLib.calcMove(60, .8f);
////            autoLib.calcMove(-28, .8f);
////            autoLib.calcTurn(-190, .8f);
////            autoLib.calcMove(-67, .8f);
////            autoLib.calcTurn(96, .8f);
////            autoLib.calcMove(-105, .8f);
////            autoLib.depositMarker();
////            Thread.sleep(1000);
////            autoLib.calcMove(100, .8f);
////            autoLib.setPositionintakeMinerals();
////            autoLib.moveLinearSlideToDepot(900);
//
//        } else if (gold == Constants.GoldObjectPosition.RIGHT) {
//            telemetry.addData("pos", "Right");
//              Thread.sleep(1000);
////            autoLib.calcMove(10, .8f);
////            autoLib.calcTurn(-45, .8f);
////            autoLib.calcMove(60, .8f);
////            autoLib.calcMove(-28, .8f);
////            autoLib.calcTurn(-79, .8f);
////            autoLib.calcMove(-119, .8f);
////            autoLib.calcTurn(58, .8f);
////            autoLib.calcMove(-95, .8f);
////            autoLib.depositMarker();
////            Thread.sleep(1000);
////            autoLib.calcTurn(12, .8f);
////            autoLib.calcMove(115, .8f);
////            autoLib.setPositionintakeMinerals();
////            autoLib.moveLinearSlideToDepot(900);
//
//        } else if (gold == Constants.GoldObjectPosition.CENTER) {
//            telemetry.addData("pos", "Center");
//            telemetry.update();
//            Thread.sleep(1000);
////            autoLib.calcMove(10, .8f);
////            autoLib.calcTurn(-5, .8f);
////            autoLib.calcMove(49, .8f);
////            autoLib.calcMove(-19, .8f);
////            autoLib.calcTurn(-115, .8f);
////            autoLib.calcMove(-92, .8f);
////            autoLib.calcTurn(60, .8f);
////            autoLib.calcMove(-106, .8f);
////            autoLib.depositMarker();
////            Thread.sleep(1000);
////            autoLib.calcTurn(6,.8f);
////            autoLib.calcMove(129, .8f);
////            autoLib.setPositionintakeMinerals();
////            autoLib.moveLinearSlideToDepot(900);
//
////        } else {
////            telemetry.addData("pos", "Nothing");
//
//        }
////        telemetry.update();
//
//    }
//
//    private void initialize() {
//        telemetry.addData("Status", "Initializing...");
//        telemetry.update();
//
//        autoLib = new AutoLib(this);
//
//        telemetry.addData("Status", "Ready");
//        telemetry.update();
//        waitForStart();
//
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//    }
//}
