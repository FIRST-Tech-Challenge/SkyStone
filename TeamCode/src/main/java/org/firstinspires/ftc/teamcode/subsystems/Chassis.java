package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Chassis extends Subsystem {
    //Vars
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    //Constructors
    public Chassis(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("front_left_drive");
        frontRight = hardwareMap.dcMotor.get("front_right_drive");
        backLeft = hardwareMap.dcMotor.get("back_left_drive");
        backRight = hardwareMap.dcMotor.get("back_right_drive");
        initMotors(new DcMotor[]{frontLeft, frontRight, backLeft, backRight});
    }

    //Methods

    public void runChassis(final double angle, final double turn, final double power) {
        frontLeft.setPower(power * Math.cos(angle) + turn);
        frontRight.setPower(power * Math.sin(angle) - turn);
        backLeft.setPower(power * Math.sin(angle) + turn);
        backRight.setPower(power * Math.cos(angle) - turn);
    }


}
