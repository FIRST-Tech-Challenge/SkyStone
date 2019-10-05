package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
public class Intake {
    DcMotor leftIntake;
    DcMotor rightIntake;

    public Intake(DcMotor leftIntake, DcMotor rightIntake) {
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
    }

    public DcMotor getLeftIntake() {
        return leftIntake;
    }

    public void setLeftIntake(DcMotor leftIntake) {
        this.leftIntake = leftIntake;
    }

    public DcMotor getRightIntake() {
        return rightIntake;
    }

    public void setRightIntake(DcMotor rightIntake) {
        this.rightIntake = rightIntake;
    }
}
