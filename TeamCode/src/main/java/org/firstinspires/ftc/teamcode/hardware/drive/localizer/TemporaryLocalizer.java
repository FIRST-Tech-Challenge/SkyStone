package org.firstinspires.ftc.teamcode.hardware.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.encoderTicksToInches;

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
public class TemporaryLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1.49606; // in
    public static double GEAR_RATIO = -2 / 3.0; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    private ExpansionHubEx hub;
    private BNO055IMU imu;

    private ExpansionHubMotor leftEncoder, frontEncoder;

    public TemporaryLocalizer(HardwareMap hardwareMap, BNO055IMU imu) {

        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-FORWARD_OFFSET, -LATERAL_DISTANCE/2, Math.toRadians(90)) // front
        ));

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftEncoder = hardwareMap.get(ExpansionHubMotor.class,"leftFront");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class,"leftRear");

        this.imu = imu;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * 1.13663852),
                encoderTicksToInches(-1 * frontEncoder.getCurrentPosition() * 1.149731)
        );
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
