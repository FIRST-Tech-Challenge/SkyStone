package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.TimerTask;

import static fi.iki.elonen.NanoHTTPD.Method.HEAD;

public class TTDriveSystem {

    // correct ticks = current ticks * expected distance / actual distance-
    // Formula for determing correct ticks: current ticks * expected distance / actual distance
    // conversion constants
    private static final double INCHES_TO_TICKS_VERTICAL = -43.4641507685;
    private static final double INCHES_TO_TICKS_LATERAL = 40;
    private static final double INCHES_TO_TICKS_DIAGONAL = 90.0;
    private static final double DEGREES_TO_TICKS = -9.39;

    private static final double TICKS_WITHIN_TARGET = 30.0;
    private static final int MAX_TICKS_PER_SECOND = 40;
    private static final double SPEED_ADJUST_WITH_ENCODERS_PERIOD = 0.1;
    private static final double DECELERATION_TICKS = 1000;
    private static final int ACCELERATION_TICKS = 1000;
    private static final double MINIMUM_ENCODERS_POWER = 0.1;


    //    // correct ticks = current ticks * correct distance / current distance
    //private static final double INCHES_TO_TICKS_VERTICAL = -43.46;
    //private static final double INCHES_TO_TICKS_LATERAL = 47.06;
    //private static final double INCHES_TO_TICKS_DIAGONAL = -64.29;
    //private static final double DEGREES_TO_TICKS = -9.80;
    /**
     * Maximum number of ticks a motor's current position must be away from it's target for it to
     * be considered near its target.
     */
    private static final double TICK_ERROR = 25.0;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor[] motors;

    public TTDriveSystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        correctDirections();
    }

    private void correctDirections() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void continuous(Vector2 velocity, double turnSpeed) {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double direction = velocity.getDirection();
        double power = velocity.magnitude();

        double angle = -direction + Math.PI / 4;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        double maxPow = Math.sin(Math.PI / 4);

        double frontLeftPow = (power * sin - turnSpeed) / maxPow;
        double frontRightPow = (power * cos + turnSpeed) / maxPow;
        double backLeftPow = (power * cos - turnSpeed) / maxPow;
        double backRightPow = (power * sin + turnSpeed) / maxPow;

        frontLeft.setPower(frontLeftPow);
        frontRight.setPower(frontRightPow);
        backLeft.setPower(backLeftPow);
        backRight.setPower(backRightPow);

        Telemetry telemetry = TTOpMode.getOpMode().telemetry;
        telemetry.addData("fl", frontLeftPow);
        telemetry.addData("fr,", frontRightPow);
        telemetry.addData("bl", backLeftPow);
        telemetry.addData("br", backRightPow);
        telemetry.update();
    }

    public void vertical(double inches, double speed) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (inches * INCHES_TO_TICKS_VERTICAL);

        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
        }
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }

        while (!nearTarget()) ;
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void lateral(double inches, double speed) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (inches * INCHES_TO_TICKS_LATERAL);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }

        while (!nearTarget()) ;
        brake();
    }

    /**
     * Drives at an angle whose reference angle is 45 degrees and lies in the specified quadrant.
     *
     * @param quadrant 0, 1, 2, or 3 corresponds to I, II, III, or IV respectively
     * @param inches   the inches to be travelled
     * @param speed    [0.0, 1.0]
     */
    public void diagonal(int quadrant, double inches, double speed) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (inches * INCHES_TO_TICKS_DIAGONAL);
        int[] targets = new int[4];
        double[] powers = new double[4];

        switch (quadrant) {
            case 0:
                // forward right
                targets[0] = ticks;
                targets[3] = ticks;

                powers[0] = speed;
                powers[3] = speed;
                break;
            case 1:
                // forward left
                targets[1] = ticks;
                targets[2] = ticks;

                powers[1] = speed;
                powers[2] = speed;
                break;
            case 2:
                // backward left
                targets[0] = -ticks;
                targets[3] = -ticks;

                powers[0] = speed;
                powers[3] = speed;
                break;
            case 3:
                // backward right
                targets[1] = -ticks;
                targets[2] = -ticks;

                powers[1] = speed;
                powers[2] = speed;
                break;
            default:
                throw new IllegalArgumentException("quadrant must be 0, 1, 2, or 3");
        }

        for (int i = 0; i < 4; i++) {
            DcMotor motor = motors[i];
            motor.setTargetPosition(targets[i]);
        }
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 4; i++) {
            DcMotor motor = motors[i];
            motor.setPower(powers[i]);
        }

        while (!nearTarget()) ;
        brake();
    }

    /**
     * @param degrees degrees to turn clockwise
     * @param speed   [0.0, 1.0]
     */
    public void turn(double degrees, double speed) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (degrees * DEGREES_TO_TICKS);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);
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
            if (ticksFromTarget > TICK_ERROR) {
                return false;
            }
        }

        //TTTimer.scheduleAtFixedRate(speedAdjustTask, SPEED_ADJUST_WITH_ENCODERS_PERIOD);
        while (!nearTarget()) ;
        //speedAdjustTask.cancel();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return true;
    }

    private void adjustMotorPower(DcMotor motor, double maxPower) {
        int currentTicks = motor.getCurrentPosition();
        int targetTicks = motor.getTargetPosition();
        double nextPower = calculateNextPower(currentTicks, targetTicks, motor.getPower(), maxPower);
        motor.setPower(nextPower);
    }

    private double calculateNextPower(int currentTicks, int targetTicks, double currentPower, double maxPower) {
        currentTicks = Math.abs(currentTicks);
        targetTicks = Math.abs(targetTicks);
        TTOpMode.getOpMode().telemetry.addData("current power", currentPower);
        double nextPower;
        if (currentTicks < ACCELERATION_TICKS) {
            double acceleration = Math.pow(MAX_TICKS_PER_SECOND, 2) / (2 * ACCELERATION_TICKS);
            nextPower = currentPower + acceleration * SPEED_ADJUST_WITH_ENCODERS_PERIOD;
            TTOpMode.getOpMode().telemetry.addData("power", currentPower);
            if (nextPower < MINIMUM_ENCODERS_POWER) {
                nextPower = MINIMUM_ENCODERS_POWER;
            }
        } else if (currentTicks > targetTicks - DECELERATION_TICKS) {
            double deceleration = -Math.pow(MAX_TICKS_PER_SECOND, 2) / (DECELERATION_TICKS - 2 * TICKS_WITHIN_TARGET);
            TTOpMode.getOpMode().telemetry.addData("decceleration", deceleration);
            nextPower = currentPower + deceleration * SPEED_ADJUST_WITH_ENCODERS_PERIOD;
            //nextPower = MINIMUM_ENCODERS_POWER;
            TTOpMode.getOpMode().telemetry.addData("next power", nextPower);
            if (nextPower <= 0.2){
                nextPower = 0.2;
            }

        } else {
            nextPower = 0.75;
        }
        if (nextPower > maxPower) {
            nextPower = 0.75;
        }

        return nextPower;
    }


    private void setRunMode(DcMotor.RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

}