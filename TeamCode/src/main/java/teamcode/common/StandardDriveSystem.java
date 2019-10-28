package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StandardDriveSystem {

    private static final double INCHES_TO_TICKS = -64.29;
    private static final double DEGREES_TO_TICKS = -8.888755566;
    /**
     * Maximum number of ticks a motor's current position must be away from it's target for it to
     * be considered near its target.
     */
    private static final double TICK_ERROR_TOLERANCE = 25.0;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final DcMotor[] motors;

    public StandardDriveSystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, TTHardwareComponentNames.FRONT_LEFT_DRIVE);
        frontRight = hardwareMap.get(DcMotor.class, TTHardwareComponentNames.FRONT_RIGHT_DRIVE);
        backLeft = hardwareMap.get(DcMotor.class, TTHardwareComponentNames.BACK_LEFT_DRIVE);
        backRight = hardwareMap.get(DcMotor.class, TTHardwareComponentNames.BACK_RIGHT_DRIVE);
        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void drive(double inches, double power) {
        int ticks = (int) (inches * INCHES_TO_TICKS);
        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while (!nearTarget()) ;
        brake();
    }

    public void turn(double degrees, double power) {
        int ticks = (int) (degrees * DEGREES_TO_TICKS);
        frontLeft.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while (!nearTarget()) ;
        brake();
    }

    public void brake() {
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }

    private boolean nearTarget() {
        for (DcMotor motor : motors) {
            int targetPosition = motor.getTargetPosition();
            int currentPosition = motor.getCurrentPosition();
            double ticksFromTarget = Math.abs(targetPosition - currentPosition);
            if (ticksFromTarget >= TICK_ERROR_TOLERANCE) {
                return false;
            }
        }
        return true;
    }

}
