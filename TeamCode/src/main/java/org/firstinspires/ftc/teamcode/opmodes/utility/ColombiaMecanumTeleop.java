package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

@TeleOp(name = "Colombia Mecanum Drive", group = "none")
public class ColombiaMecanumTeleop extends OpMode {

    private MecanumController driveTrain;
    
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

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
    }

    private double deadZone(double original) {
        return Math.abs(original) < 0.12 ? 0 : original;
    }
}
