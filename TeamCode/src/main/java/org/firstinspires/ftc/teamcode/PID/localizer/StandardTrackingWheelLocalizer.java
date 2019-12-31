package org.firstinspires.ftc.teamcode.PID.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class  StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1560.0;
    public static double WHEEL_RADIUS = 1.25; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -5.49; // in; offset of the lateral wheel
    private String TAG = "StandardTrackingWheelLocalizer";

    private DcMotor leftEncoder, rightEncoder, frontEncoder;




    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(1.0, DriveConstantsPID.ODOMETRY_TRACK_WIDTH / 2, 0), // left
                new Pose2d(1.0, -DriveConstantsPID.ODOMETRY_TRACK_WIDTH / 2, 0), // right
                new Pose2d(DriveConstantsPID.ODOMERY_FORWARD_OFFSET, -0.7, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("leftIntake");
        rightEncoder = hardwareMap.dcMotor.get("liftTwo");
        frontEncoder = hardwareMap.dcMotor.get("rightIntake");
        RobotLog.dd(TAG, "StandardTrackingWheelLocalizer created");
    }

    public static double encoderTicksToInches(int ticks) {
        RobotLog.dd("StandardTrackingWheelLocalizer", "encoderTicksToInches: " + " ticks: " +
                Double.toString(ticks) + " inches: " + Double.toString(WHEEL_RADIUS * 2 * Math.PI *
                GEAR_RATIO * ticks / DriveConstantsPID.odoEncoderTicksPerRev));
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / DriveConstantsPID.odoEncoderTicksPerRev;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int x = leftEncoder.getCurrentPosition();
        int y = rightEncoder.getCurrentPosition();
        int z = frontEncoder.getCurrentPosition();
        RobotLog.dd(TAG, "getWheelPositions");
        RobotLog.dd(TAG, "leftEncoder: " + x);
        RobotLog.dd(TAG, "rightEncoder: " + y);
        RobotLog.dd(TAG, "frontEncoder: " + (-1)*z);
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(-frontEncoder.getCurrentPosition())
        );
    }
}
