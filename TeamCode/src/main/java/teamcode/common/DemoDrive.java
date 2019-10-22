package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DemoDrive {

    private DcMotor leftDrive, rightDrive;
    public DemoDrive(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, DemoComponentNames.LEFT_DRIVE);
        rightDrive = hardwareMap.get(DcMotor.class, DemoComponentNames.RIGHT_DRIVE);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void leftContinuous(double power){
        leftDrive.setPower(power);
    }

    public void rightContinuous(double power) {
        rightDrive.setPower(power);
    }

}
