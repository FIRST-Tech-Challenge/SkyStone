package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.FXTAnalogUltrasonicSensor;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
//import org.firstinspires.inspection.RcInspectionActivity;

@Autonomous
public class ArmstrongTest extends AutoOpMode {
    @Override

    public void runOp() throws InterruptedException {
        //init phase
        Armstrong armstrong = new Armstrong();
//        armstrong.motorL.setReverse(!armstrong.motorL.isReversed());

        FXTAnalogUltrasonicSensor ultrasonic = new FXTAnalogUltrasonicSensor("ultrasonic");


        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Resetting Encoders");    //
        //telemetry.update();


        telemetry.addData("Status", "Initialized");


        waitForStart();
//        armstrong.forwardDistance(300, 0.3);

        while (opModeIsActive()) {
            Orientation orient = armstrong.imu.getAngularOrientation();
            telemetry.addData("first angle", orient.firstAngle);
        }


//        armstrong.motorL.resetEncoder();
//        armstrong.motorR.resetEncoder();
//        armstrong.forwardDistance(100, 0.3);
//        sleep(1000);
////        telemetry.addData("ml", armstrong.motorL.getPosition());
//        telemetry.addData("left", armstrong.motorL.getPosition());
//        telemetry.addData("right", armstrong.motorR.getPosition());
//        armstrong.forwardDistance(200, 0.3);
//        //this is after the driver presses play
//        telemetry.addData("Status", "Play");
//       armstrong.forwardDistance(100, 0.3);
//        telemetry.addData("left", armstrong.motorL.getPosition());
//        telemetry.addData("right", armstrong.motorR.getPosition());
//        sleep(400);
//        armstrong.forwardDistance(100, 0.3);
//        sleep(1000);


//        armstrong.motorR.resetEncoder();
//        armstrong.motorL.resetEncoder();
//        while (opModeIsActive()) {
////
//            telemetry.addData("ultrasonic", Math.round(ultrasonic.getDistance()));}


//        armstrong.forwardDistance(300,0.5);
//        armstrong.LeftSample();
//        armstrong.forwardDistance(100, 0.5);
//        armstrong.LeftWingStore();
//        armstrong.forwardDistance(200, 0.5);


//
//
//
//        armstrong.forwardDistance(300, 0.5);
////            telemetry.addData("right", armstrong.motorR.getPosition());
////            telemetry.addData("left", armstrong.motorL.getPosition());
////            telemetry.update();
//
////            ......
//            armstrong.forwardDistance(100, 0.3);
//        }
    }
}

//            while (armstrong.motorL.getPosition() < 1000 && opModeIsActive()) {
//                armstrong.motorL.setPower(1);
//            }
//            armstrong.motorL.stop();
//
//
//            while (armstrong.motorR.getPosition() < 1000 && opModeIsActive()) {
//                armstrong.motorR.setPower(1);
//            }
//            armstrong.motorR.stop();
//        }
//    }
//}







//        RC.t.addData(System.nanoTime());
////);


//        armstrong.imuTurnR(88, 0.3);
//        sleep(808);
//        armstrong.stop();




//       while (opModeIsActive()) {
//            if (armstrong.digitalTouch.getState()) {
//                telemetry.addData("status", "not");
//            } else {
//                telemetry.addData("sa", "touched");
//            }
//
//        }

//        armstrong.reverseDriveSystem();
//        armstrong.imuTurnR(180, 0.3);
//
//
//       // telemetry.addData('dklgjl', armstrong.driveL.getCurrentPosition());
//
//



//        armstrong.imuTurnR(180, 0.3);


//        while (opModeIsActive()){
//        RC.t.addData("LEFT", armstrong.motorL.getPosition());
//        RC.t.addData("RIGHT", armstrong.motorR.getPosition());
//        }


//
//        armstrong.forward(0.1);
//        sleep(500);
//        armstrong.stop();
//        sleep(100);
//        armstrong.forward(0.5);
//        sleep(500);
//        armstrong.stop();
//        sleep(100);
//        armstrong.forward(1);
//        sleep(500);
//        armstrong.stop();

            //armstrong.turnL(0.5);
            // sleep(250);

//

//        while (opModeIsActive()) {
//            Orientation orient = armstrong.imu.getAngularOrientation();
//
//            telemetry.addData("first angle",orient.firstAngle);
//            telemetry.addData("second angle", orient.secondAngle);
//            telemetry.addData("third angle", orient.thirdAngle);
//            telemetry.update();
//            sleep(10);
//        }
//


            //sleep(10);
            //armstrong.3(10);
            //sleep(1000);

//
//        }

