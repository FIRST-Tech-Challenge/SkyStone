//package org.firstinspires.ftc.teamcode.gamecode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
//import org.firstinspires.ftc.teamcode.robots.Felix;
//
///**
// * Created by Aila on 2017-12-10.
// */
//@Disabled
//@Autonomous
//public class FelixSimpleAuto extends AutoOpMode {
//
//    private Felix felix = null;
//
//    @Override
//    public void runOp() throws InterruptedException {
//
//        felix = new Felix();
//
//        waitForStart();
//
//        felix.releaseGlyph();
//        sleep(1000);
//        felix.holdGlyph();
//        sleep(1000);
//
//        felix.lift(3000, 0.8);
//        felix.drop(3500, -0.8);
//
//        felix.forwardDistance(500, 0.4);
//        felix.backwardDistance(500, 0.4);
//
//        /*
//        felix.imuTurnR(90, 0.4);
//        sleep(1000);
//
//
//        felix.forwardDistance(100, 0.4);
//        sleep(1000);
//        */
//    }
//}
