package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

public class Robot {
    public final ElapsedTime runtime;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    public final ModernRoboticsI2cGyro gyro;
    public final MecanumController driveTrain;

    private Servo foundationGrabber;

    public final Lift lift;

    public Robot(HardwareMap hardwareMap) {

        runtime = new ElapsedTime();

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        MecanumDrive train = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack, gyro);
        driveTrain = new MecanumController(train);

        foundationGrabber = hardwareMap.get(Servo.class, "foundation");
        foundationGrabber.scaleRange(0.4, 0.9);
        foundationGrabber.setDirection(Servo.Direction.FORWARD);

    }
}
