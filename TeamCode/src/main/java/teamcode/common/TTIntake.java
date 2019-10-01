package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TTIntake {

    private final DcMotor left, right;

    public TTIntake(DcMotor left, DcMotor right) {
        this.left = left;
        this.right = right;
    }

    public void setPower(double power) {
        left.setPower(1.0);
        right.setPower(-1.0);
    }

}
