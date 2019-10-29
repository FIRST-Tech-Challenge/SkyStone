package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

@Autonomous (name = "Timed Auto For Park", group = "none")
public class TimedAutoForPark extends LinearOpMode {

    private MecanumController driveTrain;
    double aParam = 315;
    double binvParam = 0.605;
    double cParam = 169.7;
    double maxDist = 6.0; // inchesrev control hub

    @Override
    public void runOpMode() throws InterruptedException {

        //DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "left1");
        /*DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "left2");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "right1");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "right2");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        MecanumDrive train = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack, null);
        driveTrain = new MecanumController(train);
*/

        Servo servoTest = hardwareMap.get(Servo.class, "armTune");
        RevColorSensorV3 rightColor = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "color").getDeviceClient()) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam)/aParam, -binvParam);
            }
        };

        //will return an error because 2nd color sensor hasn't been hardware mapped yet
        RevColorSensorV3 leftColor = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "color").getDeviceClient()) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam)/aParam, -binvParam);
            }
        };
        color.rawOptical();

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        int DST_CONST = 46;
        servoTest.setPosition(0);
        //driveTrain.translate(0.2, 0.75, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        while (opModeIsActive()) {
            double avg = 0;
            double redToBlueAvg;
            double greenToBlueAvg;
            double redBlueRatioLeft;
            double redBlueRatioRight;
            double greenBlueRatioRight;
            double greenBlueRatioLeft;
            for (int i = 0; i < 10; i++){
                double initial = Math.pow((rightColor.rawOptical()-cParam)/aParam, -binvParam);
                avg = avg + initial;
                redToBlueAvg += (rightColor.red()/rightColor.blue())
                greenToBlueAvg += (rightColor.green()/rightColor.blue())
            }
            double actualAvg = avg/10;
            if (runtime.seconds() < 2) {
                redToBlueAvg /= 10;
                greenToBlueAvg /=10;
            }

            telemetry.addData("R", rightColor.red());
            telemetry.addData("G", rightColor.green());
            telemetry.addData("B", rightColor.blue());
            telemetry.addData("InputData", actualAvg);
            telemetry.update();
            sleep(250);
            //Reads color values and sends them to driver station
            redBlueRatioRight = (rightColor.red()/rightColor.blue());
            redBlueRatioLeft =  (leftColor.red()/leftColor.blue());
            greenBlueRatioLeft = (rightColor.green()/rightColor.blue());
            greenBlueRatioRight (leftColor.green()/leftColor.blue())
            //should test logic when two sensors are implemented
            //also implemented time limit so that servos won't run randomly
            if (actualAvg < maxDist && runtime.seconds() < 15 {
                if ((redBlueRatioRight) > redToBlueAvg && (greenBlueRatioRight) > greenToBlueAvg && (redBlueRatioLeft) > redToBlueAvg && (greenBlueRatioLeft) > greenToBlueAvg) {
                    telemetry.addData("Middle Block Skystone", true);
                    //moves right and sets left servo power to 1
                } else if ((redBlueRatioRight) > redToBlueAvg && (greenBlueRatioRight) > greenToBlueAvg && (redBlueRatioLeft - redToBlueAvg) < 1 && (greenBlueRatioLeft - greenToBlueAvg) < 1 {
                    telemetry.addData("Left Block Skystone", true);
                    //sets left servo to 1
                } else {
                    //maybe other if logic here if necessary? Don't think its needed
                    telemetry.addData("Right Block Skystone", true);
                    //sets right servo to 1
                }
                telemetry.update();
            }


            //The if statement above allows the robot to detect yellow to find a block and move the servo accordingly
        }

//        driveTrain.translate(0,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        //driveTrain.translate(-0.2, -0.5, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
//        sleep((long) (15*(DST_CONST)));
        //The code above moves the bot back to starting position
//        driveTrain.translate(0,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
//        servoTest.setPosition(1);
//        sleep(1000);
//        driveTrain.translate(0,-1, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
//        sleep((long) (15*(DST_CONST)));
//        driveTrain.translate(1,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
//        sleep((long) (65*(DST_CONST)));
//        driveTrain.translate(0,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
//        servoTest.setPosition(0);
//        sleep(1000);
//        driveTrain.translate(-1,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
//        sleep((long) (21*(DST_CONST)));
//        driveTrain.translate(0,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
    }

}