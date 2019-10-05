package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
public class Chassis {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    public Chassis(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive) {
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
    }

    public DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public void setFrontLeftDrive(DcMotor frontLeftDrive) {
        this.frontLeftDrive = frontLeftDrive;
    }

    public DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public void setFrontRightDrive(DcMotor frontRightDrive) {
        this.frontRightDrive = frontRightDrive;
    }

    public DcMotor getBackLeftDrive() {
        return backLeftDrive;
    }

    public void setBackLeftDrive(DcMotor backLeftDrive) {

        this.backLeftDrive = backLeftDrive;
    }

    public DcMotor getBackRightDrive() {
        return backRightDrive;
    }

    public void setBackRightDrive(DcMotor backRightDrive) {
        this.backRightDrive = backRightDrive;
    }

    public void setFrontLeftPower(double power){
        frontLeftDrive.setPower(power);
    }

    public void setFrontRightPower(double power){
        frontRightDrive.setPower(power);
    }

    public void setBackLeftPower(double power){
        backLeftDrive.setPower(power);
    }

    public void setBackRightPower(double power){
        backRightDrive.setPower(power);
    }

    public void setAllPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        setFrontLeftPower(frontLeftPower);
        setFrontRightPower(frontRightPower);
        setBackLeftPower(backLeftPower);
        setBackRightPower(backRightPower);
    }
}
