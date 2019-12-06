package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.subsystemutils.Subsystem;

public class TwinstickMecanum implements Subsystem {
    DcMotor leftFMotor;
    DcMotor rightFMotor;
    DcMotor leftBMotor;
    DcMotor rightBMotor;
    Gamepad driveController;

    public void setPower(double leftStickY, double leftStickX, double rightStickX) {
        double leftF, leftB, rightF, rightB; // front & back, left & right
        leftF = leftStickY;
        rightF = leftStickY;
        leftB = leftStickY;
        rightB = leftStickY;

        leftF += leftStickX;
        rightF += -leftStickX;
        leftB += -leftStickX;
        rightB += leftStickX;

        leftF += rightStickX;
        rightF += -rightStickX;
        leftB += rightStickX;
        rightB += -rightStickX;

        double max = Math.max(Math.max(Math.abs(leftF), Math.abs(rightF)), Math.max(Math.abs(leftB), Math.abs(rightB)));

        if (max > 1) {
            leftF = leftF / max;
            rightF = rightF / max;
            leftB = leftB / max;
            rightB = rightB / max;
        }
        leftFMotor.setPower(leftF);
        rightFMotor.setPower(rightF);
        leftBMotor.setPower(leftB);
        rightBMotor.setPower(rightB);
    }

    public TwinstickMecanum(Gamepad driveController,
                            DcMotor leftFrontDrive,
                            DcMotor rightFrontDrive,
                            DcMotor leftRearDrive,
                            DcMotor rightRearDrive) {

        this.driveController = driveController;
        this.leftFMotor = leftFrontDrive;
        this.rightFMotor = rightFrontDrive;
        this.rightBMotor = rightRearDrive;
        this.leftBMotor = leftRearDrive;
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {
        setPower(driveController.left_stick_y, driveController.left_stick_x, -driveController.right_stick_x);
    }

}

