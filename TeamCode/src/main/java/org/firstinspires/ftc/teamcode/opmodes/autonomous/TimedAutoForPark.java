package org.firstinspires.ftc.teamcode.opmodes.autonomous;

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

        Servo servoTest = hardwareMap.get(Servo.class, "armTune");

//        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        RevColorSensorV3 color = new RevColorSensorV3(
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
        sleep((long) 27*DST_CONST);
        //need to fix so that its around ~2 cm away from blocks
        while (opModeIsActive()) {
            double avg = 0;
            double avgRed = 0;
            double avgGreen = 0;
            double avgBlue = 0;
            double initialRtoB = 0;
            double initialGtoB = 0;
            double actualAvg = 0;
            for (int i = 0; i < 10; i++) {
                double initial = Math.pow((color.rawOptical() - cParam) / aParam, -binvParam);
                avg = avg + initial;
                avgBlue += color.blue();
                avgGreen += color.green();
                avgRed += color.red();
                //2.05 RtoB
                //2.94 GtoB with yellow block
            }
            actualAvg = avg / 10;
            avgBlue /= 10;
            avgGreen /= 10;
            avgRed /= 10;
            if (runtime.seconds() < 2) {
                initialRtoB = (avgRed)/(avgBlue);
                initialGtoB = (avgGreen)/(avgBlue);
            }

            telemetry.addData("R", avgRed);
            telemetry.addData("G", avgGreen);
            telemetry.addData("B", avgBlue);
            telemetry.addData("RtoB", avgRed / avgBlue);
            telemetry.addData("GtoB", avgGreen / avgBlue);
            telemetry.addData("InputData", actualAvg);
            //telemetry.addData("Distance", color.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(500);
            //Reads color values a// nd sends them to driver station//red to blue => 1.7 - 2.5
            //green to blue => 3 - 4

            if ((avgRed / avgBlue) > initialRtoB && (avgGreen / avgBlue) > initialGtoB && actualAvg < maxDist) {
//                servoTest.setPosition(1);
//                driveTrain.translate(0, 0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
                telemetry.addData("This detects yellow block", 5);
//                telemetry.update();
//                sleep(2000);
            }
//            else {
//                telemetry.addData("Detects it's too close", "true");
//                telemetry.update();
//            }

            //The if statement above allows the robot to detect yellow to find a block and move the servo accordingly
            sleep(1);
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