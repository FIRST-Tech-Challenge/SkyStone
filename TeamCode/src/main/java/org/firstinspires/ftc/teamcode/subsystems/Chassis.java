package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassis extends Subsystem {
    // Vars
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public Double wheelRadius;
    public Double robotRadius;

    // Constructors
    public Chassis(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("front_left_drive");
        frontRight = hardwareMap.dcMotor.get("front_right_drive");
        backLeft = hardwareMap.dcMotor.get("back_left_drive");
        backRight = hardwareMap.dcMotor.get("back_right_drive");
        initMotors(new DcMotor[]{frontLeft, frontRight, backLeft, backRight});
        initChassis();
    }

    public void initChassis() {
        reset();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Methods
    public void run(double targetAngle, double turn, double power) {
        final double turnAngle = targetAngle - Math.PI / 4;
        frontLeft.setPower(power * Math.cos(turnAngle) + turn);
        frontRight.setPower(power * Math.sin(turnAngle) - turn);
        backLeft.setPower(power * Math.sin(turnAngle) + turn);
        backRight.setPower(power * Math.cos(turnAngle) - turn);
    }

    public void runDistance(double distance, double targetAngle, double turn, double power) {
        final double turnAngle = targetAngle - Math.PI / 4;
        final double wheelDistance = (Math.sqrt(2) / wheelRadius) * distance;
        final double robotTurn = robotRadius * turn;
        frontLeft.setTargetPosition((int) (wheelDistance * Math.cos(turnAngle) + robotTurn));
        frontRight.setTargetPosition((int) (wheelDistance * Math.sin(turnAngle) - robotTurn));
        backLeft.setTargetPosition((int) (wheelDistance * Math.sin(turnAngle) + robotTurn));
        backRight.setTargetPosition((int) (wheelDistance * Math.cos(turnAngle) - robotTurn));
        setMotorPowers(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runRotations(double rotations, double power) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reset();
        setMotorPowers(power);
        while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(1440 * rotations)) ;
        setMotorPowers(0);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Returns true if the moving is done
}
