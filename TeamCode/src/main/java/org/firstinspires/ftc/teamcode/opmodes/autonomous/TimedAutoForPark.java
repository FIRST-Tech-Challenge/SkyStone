package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.MecanumDriveImpl;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

@Autonomous (name = "Timed Auto For Park", group = "none")
public class TimedAutoForPark extends LinearOpMode {

    private MecanumController driveTrain;
    double aParam = 315;
    double binvParam = 0.605;
    double cParam = 169.7;
    double maxDist = 6.0; // inches

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "left1");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "left2");
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

        Servo Test = hardwareMap.get(Servo.class, "armTune");

//        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        RevColorSensorV3 color1 = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "color1").getDeviceClient()) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam) / aParam, -binvParam);
            }
        };
        RevColorSensorV3 color2 = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "color2").getDeviceClient()) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam) / aParam, -binvParam);
            }
        };
        RevColorSensorV3 color3 = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "color3").getDeviceClient()) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam) / aParam, -binvParam);
            }
        };

        color1.rawOptical();
        color2.rawOptical();
        color3.rawOptical();

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        int DST_CONST = 46;
        Test.setPosition(0);
        //driveTrain.translate(0.2, 0.75, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        sleep((long) 27 * DST_CONST);
        //need to fix so that its around ~2 cm away from blocks

        while (opModeIsActive()) {
            double avg = 0;
            double redToBlueAvg = 0;
            double greenToBlueAvg = 0;
            for (int i = 0; i < 10; i++) {
                double initial = Math.pow((color1.rawOptical() - cParam) / aParam, -binvParam);
                avg = avg + initial;
                redToBlueAvg += (color1.red() / color1.blue());
                greenToBlueAvg += (color1.green() / color1.blue());
            }
            double actualAvg = avg / 10;
            if (runtime.seconds() < 2) {
                redToBlueAvg /= 10;
                greenToBlueAvg /= 10;
            }

            telemetry.addData("R", color1.red());
            telemetry.addData("G", color1.green());
            telemetry.addData("B", color1.blue());
            telemetry.addData("InputData", actualAvg);
            telemetry.update();
            sleep(500);
            //Reads color values and sends them to driver station
            if (actualAvg < maxDist && (color1.red() / color1.blue()) > redToBlueAvg && (color1.green() / color1.blue()) > greenToBlueAvg) {
                driveTrain.translate(0, 0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
                telemetry.update();
//                telemetry.addData("R", color.red());
//                telemetry.addData("G", color.green());
//                telemetry.addData("B", color.blue());
                sleep(2000);
            }
//            else {
//                telemetry.addData("Detects it's too close", "true");
//                telemetry.update();
//            }

            //The if statement above allows the robot to detect yellow to find a block and move the servo accordingly
            sleep(1);
        }
    }

        private boolean readsYellow (RevColorSensorV3 color){
            if (actualAvg < maxDist && (color.red() / color.blue()) > redToBlueAvg && (color.green() / color.blue()) > greenToBlueAvg) {
                return true;
            } else {
                return false;
            }
        }
}