package org.firstinspires.ftc.teamcode.PID.mecanum;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.localizer.TrackingWheelLocalizerWithIMU;
import org.firstinspires.ftc.teamcode.PID.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.getMotorVelocityF;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hubLeft, hubRight;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors, motorsLeft, motorsRight;
    private BNO055IMU imu;
    private String TAG = "SampleMecanumDriveREVOptimized";
    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap, boolean strafe) {
        super(strafe);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hubLeft = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hubRight = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "frontLeft");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "backLeft");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "backRight");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        motorsLeft = Arrays.asList(leftFront, leftRear);
        motorsRight = Arrays.asList(rightRear, rightFront);
        RobotLog.dd(TAG, "SampleMecanumDriveREVOptimized created");

        for (ExpansionHubMotor motor : motors) {
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DriveConstants.BRAKE_ON_ZERO?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            RobotLog.dd(TAG,"MOTOR_VELO_PID!=0, to setPIDCoefficients");
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE); //???
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        //setLocalizer(new TrackingWheelLocalizerWithIMU(hardwareMap, imu));
        if (DriveConstantsPID.RUN_USING_IMU_LOCALIZER) {
            RobotLog.dd(TAG, "to setLocalizer to imu");
            setLocalizer(new TrackingWheelLocalizerWithIMU(hardwareMap, imu));
        }
        else
            RobotLog.dd(TAG, "not using imu");

        if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
            RobotLog.dd(TAG, "to setLocalizer to StandardTrackingWheelLocalizer");
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        }
        else
            RobotLog.dd(TAG, "not using Odometry localizer");
    }
    @Override
    public void setBrakeonZeroPower(boolean flag) {
        for (ExpansionHubMotor motor : motors) {
            if (flag == true)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        RobotLog.dd(TAG, "setBrakeonZeroPower " + flag);    }
    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        RobotLog.dd(TAG, "getPIDCoefficients:\np: " + Double.toString(coefficients.p) + " i: " + coefficients.i
                + " d: " + coefficients.d);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        double t = getMotorVelocityF();
        RobotLog.dd(TAG, "setPIDCoefficients:\nkP: " + Double.toString(coefficients.kP) + " kI: " + coefficients.kI
                    + " kD: " + coefficients.kD + " MotorVelocityF: " + Double.toString(t));
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, t
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkDataLeft = hubLeft.getBulkInputData();
        RevBulkData bulkDataRight = hubRight.getBulkInputData();
        if ((bulkDataLeft == null) || (bulkDataRight == null)) {
            RobotLog.dd(TAG, "bulk data = null");
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motorsLeft) {
            double t1 = bulkDataLeft.getMotorCurrentPosition(motor);
            double t2 = encoderTicksToInches(t1);
            RobotLog.dd(TAG, "getWheelPositions: " + "position: " + Double.toString(t1) + " inches: " + Double.toString(t2));

            wheelPositions.add(t2);
        }
        for (ExpansionHubMotor motor : motorsRight) {
            double t1 = bulkDataRight.getMotorCurrentPosition(motor);
            double t2 = encoderTicksToInches(t1);
            RobotLog.dd(TAG, "getWheelPositions: " + "position: " + Double.toString(t1) + " inches: " + Double.toString(t2));

            wheelPositions.add(t2);
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkDataLeft = hubLeft.getBulkInputData();
        RevBulkData bulkDataRight = hubRight.getBulkInputData();
        if ((bulkDataLeft == null)||(bulkDataRight == null)) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : motorsLeft) {
            double t1 = bulkDataLeft.getMotorVelocity(motor);
            double t2 = encoderTicksToInches(t1);
            RobotLog.dd(TAG, "getWheelVelocities: " + "velocity: " + Double.toString(t1) + " inches: " + Double.toString(t2));
            
            wheelVelocities.add(t2);
        }
        for (ExpansionHubMotor motor : motorsRight) {
            double t1 = bulkDataRight.getMotorVelocity(motor);
            double t2 = encoderTicksToInches(t1);
            RobotLog.dd(TAG, "getWheelVelocities: " + "velocity: " + Double.toString(t1) + " inches: " + Double.toString(t2));

            wheelVelocities.add(t2);
        }
        return wheelVelocities;
    }
    @Override
    public List<Double> getMotorPowers(List<DcMotorEx> motors) {
        List<Double> motorPowers = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            double t = motor.getPower();
            RobotLog.dd(TAG, "getMotorPowers: " + "power: " + Double.toString(t));

            motorPowers.add(t);
        }
        return motorPowers;
    }
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        RobotLog.dd(TAG, "setMotorPowers "+"leftFront: " + Double.toString(v));
        RobotLog.dd(TAG, "setMotorPowers "+"leftRear: "+Double.toString(v1));
        RobotLog.dd(TAG, "setMotorPowers "+"rightRear: "+Double.toString(v2));
        RobotLog.dd(TAG, "setMotorPowers"+"rightFront: "+Double.toString(v3));
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        double t = imu.getAngularOrientation().firstAngle;
        RobotLog.dd(TAG, "getRawExternalHeading: " + Double.toString(t));
        return t;
    }
}
