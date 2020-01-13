package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

/***
 * This class is an interface for the hardware of the Maccabots Mk. 1 drivetrain, MaccaDrive. To
 * use this class, ensure that the following is true:
 * - Motor Port 0 has the front left drive motor plugged in and configured as "front_left"
 * - Motor Port 1 has the front right drive motor plugged in and configured as "front_right"
 * - Motor Port 2 has the back left drive motor plugged in and configured as "back_left"
 * - Motor Port 3 has the back right drive motor plugged in and configured as "back_right"
 * - All motors have their encoders plugged in properly.
 *
 * This class supports both Driver-Controller and Autonomous functionality. In the AUTO mode,
 * the drivetrain behaves like a differential tank drive. Holonomic capability is only available
 * in the DRIVER mode.
 */
@Config
public class MaccaDrive {

    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    private DcMotorEx front_left, front_right, back_left, back_right;
    private List<DcMotorEx> driveMotors;
    private BNO055IMU imu;

    public static double velocity_kP, velocity_kI, velocity_kD, velocity_kF, position_kP;

    static double WHEEL_RADIUS = 1.9685;
    static double GEAR_RATIO = 5.2 * 2.0 * 1.9;
    static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);

    /**
     * MaccaDrive is designed to be capable of independent operation as a drivetrain, but is
     * most often initialized into some sort of a hardware class (i.e. Maccabot). When initialized
     * as a subclass of Maccabot, feed the parent OpMode from the hardware class through.
     */
    public MaccaDrive(OpMode parentOpMode) {
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
        driveMotors = Arrays.asList(front_left, front_right, back_left, back_right);
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

        // Reverse back motors to accomodate for mounting orientation
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // When the motors stop, they need to really stop.
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Clears encoders from previous runs
        // runToPosition() is used for auto navigation and velocity PID smooths teleop driving
        if (isAuto) {
            setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        } else { setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER); }

        // PID coefficients need to be tuned when major hardware changes are made.
        // TODO tune coefficients for MaccaDrive
        setCoefficients(velocity_kP, velocity_kI, velocity_kD, velocity_kF, position_kP);

        parentOpMode.telemetry.addLine("MaccaDrive initialization successful.");
    }

    /* *********
     * UTILITIES
     ***********/

    /***
     * @param mode the mode to which the drive motors should be set
     */
    public void setMotorModes(DcMotorEx.RunMode mode) {
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
    public void setCoefficients(double v_kP, double v_kI, double v_kD, double v_kF, double p_kP) {
        for (DcMotorEx motor : driveMotors) {
            motor.setVelocityPIDFCoefficients(v_kP, v_kI, v_kD, v_kF);
            motor.setPositionPIDFCoefficients(p_kP);
        }
    }

    public void updateCoefficientsFromConfigutation() {
        setCoefficients(velocity_kP, velocity_kI, velocity_kD, velocity_kF, position_kP);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
    }

    public static int inchesToEncoderTicks(double inches) {
        return (int) ((inches * MOTOR_CONFIG.getTicksPerRev()) / (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO));
    }

    public static double inchesToDegrees(double inches) {
        return inches * WHEEL_RADIUS * 2 / 360;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getAchieveableMaxRPMFraction();
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return (MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0);
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
        front_left.setPower(flPower);
        front_right.setPower(frPower);
        back_left.setPower(blPower);
        back_right.setPower(brPower);
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
     * Sets the targets for each side of the drivetrain.
     * @param leftTarget
     * @param rightTarget
     */
    public void setLinearTargetsTicks(int leftTarget, int rightTarget) {
        parentOpMode.telemetry.addData("Setting Left Target: ", leftTarget);
        parentOpMode.telemetry.addData("Setting Right Target: ", rightTarget);
        front_left.setTargetPosition(leftTarget);
        back_left.setTargetPosition(leftTarget);
        front_right.setTargetPosition(rightTarget);
        back_right.setTargetPosition(rightTarget);
    }

    public void setLinearTargetsInches(double leftTarget, double rightTarget) {
        setLinearTargetsTicks(inchesToEncoderTicks(leftTarget), inchesToEncoderTicks(rightTarget));
    }

    /**
     * A simple single-velocity method for running the robot in straight lines or turning.
     * @param inchesPerSecond Travel velocity in inches per second
     */
    public void runToTargets(double inchesPerSecond) {
        for (DcMotorEx motor: driveMotors) {
            motor.setVelocity(inchesToDegrees(inchesPerSecond), AngleUnit.DEGREES);
        }
    }

    public boolean isDriveBusy() {
        return front_left.isBusy() || front_right.isBusy();
    }

}
