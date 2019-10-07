package org.firstinspires.ftc.teamcode.opmodes.autonomous;

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

        ColorSensor color = hardwareMap.get(ColorSensor.class, "color");
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "color");

        double secs;
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        double DST_CONST = 46;
//        secs * spedConst; 0.9 sec/30 inch
        driveTrain.translate(0, 1, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        while (runtime.seconds() < (30*DST_CONST/1000)){
            telemetry.addData("R", color.red());
            telemetry.addData("G", color.green());
            telemetry.addData("B", color.blue());
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(1);
        }
        runtime.reset();
        driveTrain.translate(0,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        servoTest.setPosition(1);
        sleep(1000);
        driveTrain.translate(0,-1, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        sleep((long) (15*(DST_CONST)));
        driveTrain.translate(1,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        sleep((long) (65*(DST_CONST)));
        driveTrain.translate(0,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        servoTest.setPosition(0);
        sleep(1000);
        driveTrain.translate(-1,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);
        sleep((long) (21*(DST_CONST)));
        driveTrain.translate(0,0, MecanumDrive.TranslationMethod.CONSTANT_SPEED);

    }
}
