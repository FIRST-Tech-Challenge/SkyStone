package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.westtorrancerobotics.lib.MecanumController;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.RIGHT;

@Autonomous (name = "Timed Auto For Park", group = "none")
public class TimedAutoForPark extends LinearOpMode {

//    private MecanumController driveTrain;
//    double aParam = 315;
//    double binvParam = 0.605;
//    double cParam = 169.7;
//    double maxDist = 6.0; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = Robot.getInstance();
        bot.init(hardwareMap);
        bot.foundationGrabber.setGrabbed(LEFT,false);
        bot.foundationGrabber.setGrabbed(RIGHT,false);
        bot.lift.idle();
        bot.lift.zero();
        bot.runtime.reset();

//        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
//        RevColorSensorV3 color1 = new RevColorSensorV3(
//                hardwareMap.get(RevColorSensorV3.class, "color1").getDeviceClient()) {
//            @Override
//            protected double inFromOptical(int rawOptical) {
//                return Math.pow((rawOptical - cParam) / aParam, -binvParam);
//            }
//        };
//        RevColorSensorV3 color2 = new RevColorSensorV3(
//                hardwareMap.get(RevColorSensorV3.class, "color2").getDeviceClient()) {
//            @Override
//            protected double inFromOptical(int rawOptical) {
//                return Math.pow((rawOptical - cParam) / aParam, -binvParam);
//            }
//        };
//        RevColorSensorV3 color3 = new RevColorSensorV3(
//                hardwareMap.get(RevColorSensorV3.class, "color3").getDeviceClient()) {
//            @Override
//            protected double inFromOptical(int rawOptical) {
//                return Math.pow((rawOptical - cParam) / aParam, -binvParam);
//            }
//        };
//
//        color1.rawOptical();
//        color2.rawOptical();
//        color3.rawOptical();

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        int DST_CONST = 46;
        bot.driveTrain.spinDrive(0, 1, 0);
        sleep((long) 15 * DST_CONST);
        bot.driveTrain.spinDrive(0,0, 0);
        sleep(1000);
        bot.driveTrain.spinDrive(0, -0.5, 0);
        sleep((long) (12*(DST_CONST)));
        //The code above moves the bot back to starting position
        //servoTest.setPosition(1);
        bot.driveTrain.spinDrive(1,0, 0);
        sleep((long) (65*(DST_CONST)));
        bot.driveTrain.spinDrive(0,0, 0);
        //servoTest.setPosition(0);
        sleep(1000);
        bot.driveTrain.spinDrive(-1,0, 0);
        sleep((long) (30*(DST_CONST)));
        bot.driveTrain.spinDrive(0,0, 0);

//        while (opModeIsActive()) {
//            double avg = 0;
//            double redToBlueAvg = 0;
//            double greenToBlueAvg = 0;
//            for (int i = 0; i < 10; i++) {
//                double initial = Math.pow((color1.rawOptical() - cParam) / aParam, -binvParam);
//                avg = avg + initial;
//                redToBlueAvg += (color1.red() / color1.blue());
//                greenToBlueAvg += (color1.green() / color1.blue());
//            }
//            double actualAvg = avg / 10;
//            if (runtime.seconds() < 2) {
//                redToBlueAvg /= 10;
//                greenToBlueAvg /= 10;
//            }
//
//            telemetry.addData("R", color1.red());
//            telemetry.addData("G", color1.green());
//            telemetry.addData("B", color1.blue());
//            telemetry.addData("InputData", actualAvg);
//            telemetry.update();
//            sleep(500);
//            //Reads color values and sends them to driver station
//            if (actualAvg < maxDist && (color1.red() / color1.blue()) > redToBlueAvg && (color1.green() / color1.blue()) > greenToBlueAvg) {
//                driveTrain.translate(0, 0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
//                telemetry.update();
////                telemetry.addData("R", color.red());
////                telemetry.addData("G", color.green());
////                telemetry.addData("B", color.blue());
//                sleep(2000);
//            }
//
//            //The if statement above allows the robot to detect yellow to find a block and move the servo accordingly
//            sleep(1);
//        }
    }

        private boolean readsYellow (RevColorSensorV3 color){
//            if (actualAvg < maxDist) {
//                if ((color.red() / color.blue()) > redToBlueAvg && (color.green() / color.blue()) > greenToBlueAvg) {
//                    return true;
//                } else {
//                    return false;
//                }
//            }
            return false;
        }
}