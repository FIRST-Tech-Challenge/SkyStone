package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Chassis extends Subsystem {
    //Vars
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;
    final double wheelRadius = 24.0;
    final double robotRadius = 5.08;
    //Constructors
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

    //Methods
    public void runChassis(double targetAngle, double turn, double power) {
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
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Returns true if the moving is done
    public boolean runDistanceCheck(int targetError) {
        return targetError > getAverageMotorError();
    }
}
