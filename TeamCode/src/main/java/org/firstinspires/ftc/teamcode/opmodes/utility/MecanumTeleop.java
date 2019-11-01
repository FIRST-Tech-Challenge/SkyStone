package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.TestableGyro;
import org.firstinspires.ftc.teamcode.lib.MecanumDriveImpl;
import org.westtorrancerobotics.lib.Angle;
import org.westtorrancerobotics.lib.Location;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

@TeleOp(name = "Drive Only", group = "none")
public class MecanumTeleop extends OpMode {

    private MecanumController driveTrain;
    private ElapsedTime runtime;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        MecanumDrive train = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack, TestableGyro.NULL);
        driveTrain = new MecanumController(train);

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double turn = deadZone(gamepad1.right_stick_x);
        double y = -deadZone(gamepad1.left_stick_y);
        double x = deadZone(gamepad1.left_stick_x);
        driveTrain.spinDrive(x, y, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);

        telemetry.addData("Time elapsed", runtime.seconds());
        telemetry.update();
    }

    private double deadZone(double original) {
        return Math.abs(original) < 0.12 ? 0 : original;
    }
}
