package DemoCodes.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DemoDrive {

    private final DcMotor leftDrive, rightDrive;
    private final DcMotor[] motors;

    private final double INCHES_TO_TICKS = 23.00040882;
    private final double DEGREES_TO_TICKS = 8.8;

    /**
     * Maximum number of ticks a motor's current position must be away from it's target for it to
     * be considered near its target.
     */
    private static final double TICK_ERROR_TOLERANCE = 25.0;
    /**
     * Proportional.
     */
    private static final double P = 2.5;
    /**
     * Integral.
     */
    private static final double I = 0.1;
    /**
     * Derivative.
     */
    private static final double D = 0.0;

    public DemoDrive(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, DemoComponentNames.LEFT_DRIVE);
        rightDrive = hardwareMap.get(DcMotor.class, DemoComponentNames.RIGHT_DRIVE);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new DcMotor[]{leftDrive, rightDrive};
        setPID();
    }

    public void continuous(double vertical, double turn) {
        leftDrive.setPower(vertical);
        rightDrive.setPower(vertical);
        if (turn != 0) {
            leftDrive.setPower(-turn);
            rightDrive.setPower(turn);
        }
    }
    private void setPID() {
        PIDCoefficients coefficients = new PIDCoefficients();
        coefficients.i = I;
        coefficients.p = P;
        coefficients.d = D;
        for (DcMotor motor : motors) {
            DcMotorEx ex = (DcMotorEx) motor;
            ex.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients);
        }
    }

    public void vertical(double inches, double speed) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (inches * INCHES_TO_TICKS);

        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
        }
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (DcMotor motor : motors) {
            motor.setPower(speed);
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

    public void turn(double degrees, double speed) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (degrees * DEGREES_TO_TICKS);
        leftDrive.setTargetPosition(ticks);
        rightDrive.setTargetPosition(-ticks);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }

        while (!nearTarget()) ;
        //brake();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

}