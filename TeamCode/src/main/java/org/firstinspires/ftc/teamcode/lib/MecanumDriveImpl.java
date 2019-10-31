package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.TestableGyro;
import org.westtorrancerobotics.lib.MecanumDrive;
import org.westtorrancerobotics.lib.Angle;

public class MecanumDriveImpl implements MecanumDrive {

    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightBack;
    private final TestableGyro gyro;

    public MecanumDriveImpl(DcMotorEx leftFront, DcMotorEx leftBack,
                            DcMotorEx rightFront, DcMotorEx rightBack,
                            TestableGyro gyro) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.gyro = gyro;
    }

    @Override
    public void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        leftFront.setPower(frontLeft);
        leftBack.setPower(backLeft);
        rightFront.setPower(frontRight);
        rightBack.setPower(backRight);
    }

    @Override
    public double getFrontLeftPower() {
        return leftFront.getPower();
    }

    @Override
    public double getBackLeftPower() {
        return leftBack.getPower();
    }

    @Override
    public double getFrontRightPower() {
        return rightFront.getPower();
    }

    @Override
    public double getBackRightPower() {
        return rightBack.getPower();
    }

    @Override
    public long getFrontLeftEncoder() {
        return leftFront.getCurrentPosition();
    }

    @Override
    public long getFrontRightEncoder() {
        return rightFront.getCurrentPosition();
    }

    @Override
    public long getBackLeftEncoder() {
        return leftBack.getCurrentPosition();
    }

    @Override
    public long getBackRightEncoder() {
        return rightBack.getCurrentPosition();
    }

    @Override
    public Angle getGyro() {
        return gyro.getHeading();
    }

    @Override
    public double getWheelbaseWidth() {
        return 14.544475;
    }

    @Override
    public double getWheelDiameter() {
        return 4;
    }

    @Override
    public double getEncoderTicksPerRevolution() {
        return 560;
    }
}
