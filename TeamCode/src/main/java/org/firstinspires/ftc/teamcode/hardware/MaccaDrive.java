package org.firstinspires.ftc.teamcode.hardware;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

/***
 * This class is an interface for the hardware of the Maccabots Mk. 1 drivetrain. To use this
 * class, ensure that the following is true:
 * - Motor Port 0 has the front left drive motor plugged in and configured as "front_left"
 * - Motor Port 1 has the front right drive motor plugged in and configured as "front_right"
 * - Motor Port 2 has the back left drive motor plugged in and configured as "back_left"
 * - Motor Port 3 has the back right drive motor plugged in and configured as "back_right"
 * - All motors have their encoders plugged in properly.
 *
 * This class supports both Driver-Controller and Autonomous functionality. In the AUTO mode,
 * the drivetrain behaves like a differential tank drive. Holonomic capability is only available
 * in the DRIVER mode.
 *
 * This class has a structure created surrounding the
 */
@Config
public class MaccaDrive {

    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    private DcMotorEx front_left, front_right, back_left, back_right;
    private List<DcMotorEx> driveMotors;
    private BNO055IMU imu;

    private boolean isGyroTurnBusy;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    public static double velocity_kP = 29;
    public static double velocity_kI = 0;
    public static double velocity_kD = 12;
    public static double velocity_kF = 11.9;
    public static double position_kP = 1;

    public enum TelemetryLevel { SILENT, MINIMAL, FULL }

    /*
     * As configured for MaccaDrive Mk. 1, there are 553.28 encoder ticks per wheel rotation.
     * A drivetrain with wheels 100mm in diameter will travel 12.37 inches per wheel rotation.
     */
    private static double WHEEL_RADIUS = 1.9685; // inches, converted from 100mm
    private static double TRACK_WIDTH = 13.3858; // inches, converted from 340mm
    private static double GEAR_RATIO = 5.2 * 2.0 * 1.9;
    private static int COUNTS_PER_REV = 28;
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);

    /**
     * MaccaDrive is designed to be capable of independent operation as a drivetrain, but is
     * most often initialized into some sort of a hardware class (i.e. Maccabot). When initialized
     * as a subclass of Maccabot, feed the parent OpMode from the hardware class through.
     */
    public MaccaDrive(OpMode parentOpMode) {
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }

    /**
     * Initializes the drivetrain and outputs statuses to telemetry.
     * @param isAuto true if the drivetrain is running autonomously
     */
    public void initializeDrive(boolean isAuto) {
        if (isAuto) {
            parentOpMode.telemetry.addLine("MaccaDrive initializing in AUTO mode");
        } else {
            parentOpMode.telemetry.addLine("MaccaDrive initializing in DRIVER mode");
        }

        // Get drive motors
        front_left = hardwareMap.get(DcMotorEx.class, "front_left"); // Port 0
        front_right = hardwareMap.get(DcMotorEx.class, "front_right"); // Port 1
        back_left = hardwareMap.get(DcMotorEx.class,"back_left"); // Port 2
        back_right = hardwareMap.get(DcMotorEx.class,"back_right"); // Port 3

        driveMotors = Arrays.asList(front_left, front_right, back_left, back_right);

        // Reverse back motors to accomodate for mounting orientation
        driveMotors.get(2).setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);

        // When the motors stop, they need to really stop.
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Clears encoders from previous runs
        // runToPosition() is used for auto navigation and velocity PID smooths teleop driving
        if (isAuto) {
            setTargetsTicks(0, 0);
            setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
            // PID coefficients need to be tuned when major hardware changes are made.
            // TODO tune coefficients for MaccaDrive
            setCoefficients(velocity_kP, velocity_kI, velocity_kD, velocity_kF, position_kP);
        } else { setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER); }
        // Initialize IMU if in AUTO mode
        if (isAuto) {
            parentOpMode.telemetry.addLine("Initializing IMU...");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU"; // value logs accessible through ADB Logcat
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            parentOpMode.telemetry.addLine("IMU Initialization Complete.");
        }

        parentOpMode.telemetry.addLine("MaccaDrive initialization successful.");
    }

    /* *******************
     * GETTERS AND SETTERS
     *********************/

    /***
     * @param mode the mode to which the drive motors should be set
     */
    private void setMotorModes(DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : driveMotors) {
            motor.setMode(mode);
        }
    }
    /***
     * Sets the coefficients of the internal velocity and position PIDF control loops. These
     * values are found through experimentation and can be easily tuned using the FTC dashboard.
     *
     * @param v_kP Velocity Proportional Constant (normally between 20 and 40)
     * @param v_kI Velocity Integral Constant (normally 0)
     * @param v_kD Velocity Derivative Constant (normally between 10 and 20)
     * @param v_kF Velocity FeedForward Constant (normally between 10 and 15)
     * @param p_kP Position Proportional Constant
     */
    private void setCoefficients(double v_kP, double v_kI, double v_kD, double v_kF, double p_kP) {
        for (DcMotorEx motor : driveMotors) {
            motor.setVelocityPIDFCoefficients(v_kP, v_kI, v_kD, v_kF);
            motor.setPositionPIDFCoefficients(p_kP);
        }
    }

    public void updateCoefficientsFromConfigutation() {
        setCoefficients(velocity_kP, velocity_kI, velocity_kD, velocity_kF, position_kP);
    }

    /**
     * @return the angle of the robot in degrees, from 0 to 360.
     */
    public double getOrientation() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle; // integrates the delta logic into the final angle
        lastAngles = angles;
        return globalAngle;
    }

    /* *********
     * TELEMETRY
     ***********/

    public void composeTelemetry(TelemetryLevel telemetryLevel) {
        switch (telemetryLevel) {
            case SILENT:
                break;
            case MINIMAL:
                parentOpMode.telemetry.addData("Orientation", getOrientation());
                break;
            case FULL:
                parentOpMode.telemetry.addData("Orientation", getOrientation());
                parentOpMode.telemetry.addLine();
                parentOpMode.telemetry.addData("FL Power",driveMotors.get(0).getPower());
                parentOpMode.telemetry.addData("FR Power",driveMotors.get(1).getPower());
                parentOpMode.telemetry.addData("BL Power",driveMotors.get(2).getPower());
                parentOpMode.telemetry.addData("BR Power",driveMotors.get(3).getPower());
                parentOpMode.telemetry.addLine();
                parentOpMode.telemetry.addData("FL Pos",driveMotors.get(0).getCurrentPosition());
                parentOpMode.telemetry.addData("FR Pos",driveMotors.get(1).getCurrentPosition());
                parentOpMode.telemetry.addData("BL Pos",driveMotors.get(2).getCurrentPosition());
                parentOpMode.telemetry.addData("BR PoS",driveMotors.get(3).getCurrentPosition());

        }
    }

    public void addMotorPowersToTelemetry() {
        parentOpMode.telemetry.addData("FL Power",driveMotors.get(0).getPower());
        parentOpMode.telemetry.addData("FR Power",driveMotors.get(1).getPower());
        parentOpMode.telemetry.addData("BL Power",driveMotors.get(2).getPower());
        parentOpMode.telemetry.addData("BR Power",driveMotors.get(3).getPower());
    }

    public void addMotorPositionsToTelemetry() {
        parentOpMode.telemetry.addData("FL Pos",driveMotors.get(0).getCurrentPosition());
        parentOpMode.telemetry.addData("FR Pos",driveMotors.get(1).getCurrentPosition());
        parentOpMode.telemetry.addData("BL Pos",driveMotors.get(2).getCurrentPosition());
        parentOpMode.telemetry.addData("BR PoS",driveMotors.get(3).getCurrentPosition());
    }

    /* ******************
     * INTERNAL UTILITIES
     ********************/

    public static double encoderTicksToInches(double ticks) {
        return ((WHEEL_RADIUS * 2 * Math.PI)/(COUNTS_PER_REV * GEAR_RATIO) * ticks);
    }

    public static int inchesToEncoderTicks(double inches) {
        return (int) ((COUNTS_PER_REV*GEAR_RATIO)/(WHEEL_RADIUS * 2 * Math.PI) * inches);
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getAchieveableMaxRPMFraction();
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return (MOTOR_CONFIG.getMaxRPM() * COUNTS_PER_REV / 60.0);
    }

    @Deprecated
    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / getTicksPerSec();
    }

    /*
     * HARDWARE INTERFACE COMMANDS
     */
    /***
     * @param flPower Front Left Power (-1 to 1)
     * @param frPower Front Right Power (-1 to 1)
     * @param blPower Back Left Power (-1 to 1)
     * @param brPower Back Right Power (-1 to 1)
     */
    public void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        driveMotors.get(0).setPower(flPower);
        driveMotors.get(1).setPower(frPower);
        driveMotors.get(2).setPower(blPower);
        driveMotors.get(3).setPower(brPower);
    }

    /***
     * Stops the motors hard. Use in emergencies.
     */
    public void emergencyStop() {
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        setMotorPowers(0,0,0,0);
    }

    /* **********************
     * TELEOP DRIVING METHODS
     ************************/

    /***
     * Using Inverse Kinematics, this method calculates and communicates motor powers to transform
     * the MaccaDrive in all of its degrees of freedom.
     * @param vtX X velocity (-1 to 1)
     * @param vtY Y velocity (-1 to 1)
     * @param vR rotation velocity (-1 to 1)
     */
    public void arcadeMecanumDrive(double vtX, double vtY, double vR){
        double flValue = vtY + vtX - vR;
        double frValue = vtY - vtX + vR;
        double blValue = vtY - vtX - vR;
        double brValue = vtY + vtX + vR;

        setMotorPowers(flValue, frValue, blValue, brValue);
    }

    /* **************************
     * AUTONOMOUS DRIVING METHODS
     ****************************/

    /**
     * Sets the targets for each side of the drivetrain in encoder ticks.
     * @param leftTarget Left Target in Encoder Ticks
     * @param rightTarget Right Target in Encoder Ticks
     */
    public void setTargetsTicks(int leftTarget, int rightTarget) {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parentOpMode.telemetry.addData("Setting Left Target: ", leftTarget);
        parentOpMode.telemetry.addData("Setting Right Target: ", rightTarget);
        parentOpMode.telemetry.update();
        driveMotors.get(0).setTargetPosition(leftTarget);
        driveMotors.get(2).setTargetPosition(leftTarget);
        driveMotors.get(1).setTargetPosition(rightTarget);
        driveMotors.get(3).setTargetPosition(rightTarget);
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
    }
    /**
     * Wrapper for {@link #setTargetsTicks(int, int)}.
     * @param targetsPair an integer pair: first is left target, second is right target.
     */
    public void setTargetsTicks(Pair<Integer, Integer> targetsPair) {
        setTargetsTicks(targetsPair.first, targetsPair.second);
    }

    /**
     * Runs the motors to their target at the given velocity. When turning around center or driving
     * straight, the two parameters should be the same. They should only be different when driving
     * in an arc. Remember that movement in reverse requires negative velocities.
     * @param velocityLeft Drive Left Side Velocity (encoder ticks per second)
     * @param velocityRight Drive Right Side Velocity (encoder ticks per second)
     */
    public void runToTargets(double velocityLeft, double velocityRight) {
        driveMotors.get(0).setVelocity(velocityLeft);
        driveMotors.get(2).setVelocity(velocityLeft);
        driveMotors.get(1).setVelocity(velocityRight);
        driveMotors.get(3).setVelocity(velocityRight);
    }

    public void runToTargetsInches(double velocityInchesLeft, double velocityInchesRight) {
        runToTargets(inchesToEncoderTicks(velocityInchesLeft), inchesToEncoderTicks(velocityInchesRight));
    }

    /**
     * Logical wrapper for {@link #runToTargets(double, double)}. Calculates an appropriate maximum
     * velocity
     * @param maxVelocity
     */
    public void arcToTargets(double maxVelocity) {
        int leftTarget = Math.abs(driveMotors.get(0).getTargetPosition());
        int rightTarget = Math.abs(driveMotors.get(1).getTargetPosition());
        if (rightTarget > leftTarget) {
            double slowVelocity = (maxVelocity * leftTarget) / rightTarget;
            runToTargets(slowVelocity, maxVelocity);
        } else if (leftTarget > rightTarget) { // left side of arc is bigger
            double slowVelocity = (maxVelocity * rightTarget) / leftTarget;
            runToTargets(maxVelocity, slowVelocity);
        } else {
            runToTargets(maxVelocity, maxVelocity);
        }
    }

    /**
     * Uses the built-in REV BNO055 IMU to turn the robot to a specified angle.
     * @param angle The target angle (in degrees
     */
    public void gyroTurnTo(double angle) {
        double error = angle - getOrientation();
        if (driveMotors.get(0).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (error > 0.1) {
            // vR value contains a tuned constant kP = 0.004, must be tuned with hardware changes
            // TODO tune gyro turning kP
            arcadeMecanumDrive(0, 0, error * 0.004);
        }
        arcadeMecanumDrive(0, 0, 0);
    }

    /**
     * Sets the targets for each side of the drivetrain in inches.
     * @param leftTarget Left Target in Inches
     * @param rightTarget Right Target in Inches
     */
    public void setTargetsInches(double leftTarget, double rightTarget) {
        setTargetsTicks(inchesToEncoderTicks(leftTarget), inchesToEncoderTicks(rightTarget));
    }

    /**
     * Wrapper for {@link #setTargetsInches(double, double)}.
     * @param targetsPair an integer pair: first is left target, second is right target.
     */
    public void setTargetsInches(Pair<Double, Double> targetsPair) {
        setTargetsInches(targetsPair.first, targetsPair.second);
    }

    /**
     * REMEMBER: r_out / v_out = r_in / v_in. Therefore, v_in = (v_out * r_in) / r_out.
     * @param radius the radius of the arc. Positive values are to the right of the robot.
     * @param direction the direction in which the robot should move: -1 is negative, 1 is positive
     * @return a pair of doubles: first is left target, second is right target
     */
    public Pair<Double, Double> calculateArcLengths(double radius, int direction) {
        double left_length = direction * (2 * Math.PI * (radius + TRACK_WIDTH / 2));
        double right_length = 2 * direction * (Math.PI * (radius - TRACK_WIDTH / 2));
        return new Pair<>(left_length, right_length);
    }

    public boolean isDriveBusy() {
        return driveMotors.get(0).isBusy() || driveMotors.get(1).isBusy();
    }

}
