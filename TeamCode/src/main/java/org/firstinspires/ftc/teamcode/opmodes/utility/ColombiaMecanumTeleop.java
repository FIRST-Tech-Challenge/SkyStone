package org.firstinspires.ftc.teamcode.opmodes.utility;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

@TeleOp(name = "Colombia Mecanum Drive", group = "none")
public class ColombiaMecanumTeleop extends OpMode {

    private MecanumController driveTrain;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double turn = deadZone(gamepad1.right_stick_x);
        double y = -deadZone(gamepad1.left_stick_y);
        double x = deadZone(gamepad1.left_stick_x);
        driveTrain.spinDrive(x, y, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);

    }

    private double deadZone(double original) {
        return Math.abs(original) < 0.12 ? 0 : original;
    }
}
