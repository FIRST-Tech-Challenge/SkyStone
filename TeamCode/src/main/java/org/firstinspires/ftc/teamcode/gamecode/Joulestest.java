package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous
public class Joulestest extends AutoOpMode {
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ColorSensor colorSensor;
        ColorSensor colorSensorDown;
        DistanceSensor StoneDist;
        int STRAFESTATE = 0;
        int STONESTATE = 0;
        int i = 1;

        telemetry.addData("Status", "initialized");
        colorSensor = hardwareMap.colorSensor.get("colour");
        colorSensorDown = hardwareMap.colorSensor.get("ColourDown");
        StoneDist = hardwareMap.get(DistanceSensor.class, "Distance");
        waitForStart();

        joules.DriveForwardEnc(0.5, 2000);
        sleep(100000);




//      while (opModeIsActive()) {



//            RC.t.addData("red", colorSensor.red());
//            RC.t.addData("stone dist", StoneDist.getDistance(DistanceUnit.MM));
//          RC.t.addData("Stone dist", StoneDist.getDistance(DistanceUnit.MM)); //distance sensor
            //RC.t.addData("bLUE", colorSensor.blue()); // colour sensor for stones blue value
//          RC.t.addData("rED", colorSensorDown.red()); // colour sensor down red value
//          RC.t.addData("Green", colorSensor.green()); // colour sensor for stones green value
//          RC.t.addData("Luminosity", colorSensor.alpha()); // colour sensor for stones luminosity value
            //  RC.t.addData("color sum", colorSensor.argb()); // colour sensor for stones full colour value
//
////            while (colorSensor.argb() < 10){
////                joules.DriveBackward(0.5);
////            }
////            if (colorSensor.argb() > 10) {
////                RC.t.addData("True");
////                while (colorSensor.red() > 20) {
////                    joules.DriveBackward(0.3);
////                    RC.t.addData("backwards");
////                }
////                joules.Stop();
////                RC.t.addData("stop");
////                //here add the skystone grabber to go down
////            }
//        }
            //maybe change to a double pronged 1f statment
            //if (argb sees a block and if red is small

//        while (colorSensor.alpha()>100){
//            joules.DriveBackward(0.3);
//            RC.t.addData("diving backwards");
//            }
//        joules.StoneDown();
//        sleep(1000);
//        joules.StrafeRight(0.2);
//        sleep(3000);
//        joules.Stop();
//        }

            //        while (opModeIsActive()) {
//            //RC.t.addData("bLUE", colorSensor.blue());
//            RC.t.addData("rED", colorSensorDown.red());
//            //RC.t.addData("Green", colorSensor.green());
//            //RC.t.addData("Luminosity", colorSensor.alpha());
//            //RC.t.addData("combined colour value", colorSensor.argb());
//
////            while (colorSensor.argb() < 10){
////                joules.DriveBackward(0.5);
////            }
////            if (colorSensor.argb() > 10) {
////                RC.t.addData("True");
////                while (colorSensor.red() > 20) {
////                    joules.DriveBackward(0.3);
////                    RC.t.addData("backwards");
////                }
////                joules.Stop();
////                RC.t.addData("stop");
////                //here add the skystone grabber to go down
////            }
//        }
            //maybe change to a double pronged 1f statment
            //if (argb sees a block and if red is small

//        while (colorSensor.alpha()>100){
//            joules.DriveBackward(0.3);
//            RC.t.addData("diving backwards");
//            }
//        joules.StoneDown();
//        sleep(1000);
//        joules.StrafeRight(0.2);
//        sleep(3000);
//        joules.Stop();
//        }

//        }
    }
}
