package org.firstinspires.ftc.teamcode.PID.mecanum;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.IMUBufferReader;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.localizer.TrackingWheelLocalizerWithIMU;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.PID.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.PID.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.getMotorVelocityF;

/*
 * Simple mecanum drive hardware implementation for REV hardware. If your hardware configuration
 * satisfies the requirements, SampleMecanumDriveREVOptimized is highly recommended.
 */
public class SampleMecanumDriveREV extends SampleMecanumDriveBase {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private IMUBufferReader imuReader;
    private float lastIMU = 0;

    private static String TAG = "SampleMecanumDriveREV";

    public SampleMecanumDriveREV(HardwareMap hardwareMap, boolean strafe) {
        super(strafe);
        create_instance(hardwareMap, strafe);
    }


    private void create_instance(HardwareMap hardwareMap, boolean strafe) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        RobotLogger.dd(TAG, "SampleMecanumDriveREV created");

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DriveConstantsPID.BRAKE_ON_ZERO ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            RobotLogger.dd(TAG, "MOTOR_VELO_PID!=0, to setPIDCoefficients %f, %f, %f" , MOTOR_VELO_PID.kP,
                    MOTOR_VELO_PID.kI, MOTOR_VELO_PID.kD);
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        imuReader = IMUBufferReader.getSingle_instance(hardwareMap);

        if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL == true) {
            RobotLogger.dd(TAG, "to setLocalizer to StandardTrackingWheelLocalizer");
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        } else {
            setLocalizer(new MecanumLocalizer(this, DriveConstantsPID.RUN_USING_IMU_LOCALIZER));
            RobotLogger.dd(TAG, "use default 4 wheel localizer");
        }
    }

    @Override
    public void setBrakeonZeroPower(boolean flag) {
        for (DcMotorEx motor : motors) {
            if (flag == true)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        RobotLogger.dd(TAG, "setBrakeonZeroPower " + flag);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        RobotLogger.dd(TAG, "getPIDCoefficients:\np: " + Double.toString(coefficients.p) + " i: " + coefficients.i
                + " d: " + coefficients.d);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        double t = getMotorVelocityF();
        RobotLogger.dd(TAG, "setPIDCoefficients:\nkP: " + Double.toString(coefficients.kP) + " kI: " + coefficients.kI
                + " kD: " + coefficients.kD + " MotorVelocityF: " + Double.toString(t));
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, t
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            double t1 = motor.getCurrentPosition();
            double t2 = encoderTicksToInches(t1);
            //RobotLogger.dd(TAG, motor.getDeviceName() + "getWheelPositions: " + "position: " + Double.toString(t1) +
            //      " inches: " + Double.toString(t2));

            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            double t1 = motor.getVelocity();
            double t2 = encoderTicksToInches(t1);
            RobotLogger.dd(TAG, "getWheelVelocities: " + "velocity: " + Double.toString(t1) + " inches: " + Double.toString(t2));
            wheelVelocities.add(t2);
        }
        return wheelVelocities;
    }


    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        RobotLogger.dd(TAG, "setMotorPowers " + "leftFront: " + Double.toString(v));
        leftFront.setPower(v);
        RobotLogger.dd(TAG, "setMotorPowers " + "leftRear: " + Double.toString(v1));
        leftRear.setPower(v1);
        RobotLogger.dd(TAG, "setMotorPowers " + "rightRear: " + Double.toString(v2));
        rightRear.setPower(v2);
        RobotLogger.dd(TAG, "setMotorPowers" + "rightFront: " + Double.toString(v3));
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        float t = lastIMU;
        try {
            RobotLogger.dd(TAG, "to getRawExternalHeading");
            t = imuReader.getLatestIMUData();
            RobotLogger.dd(TAG, "getRawExternalHeading: " + Double.toString(t));

        } catch (Exception e) {
            e.printStackTrace();
        }
        lastIMU = t;
        return t;
    }
    public void finalize() throws Throwable {
        imuReader.cleanUP();
    }
}
