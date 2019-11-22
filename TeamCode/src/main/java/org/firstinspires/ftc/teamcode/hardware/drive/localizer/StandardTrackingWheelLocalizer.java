package org.firstinspires.ftc.teamcode.hardware.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

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
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {

    private ExpansionHubEx hub;

    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1.49606; // in
    public static double GEAR_RATIO = -2/3; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    private ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, -LATERAL_DISTANCE / 2, Math.toRadians(90)) // front
        ));

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftEncoder = hardwareMap.get(ExpansionHubMotor.class,"leftFront");
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class,"rightFront");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class,"leftRear");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0);
        }

        return Arrays.asList(
                encoderTicksToInches(bulkData.getMotorCurrentPosition(leftEncoder)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(rightEncoder)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(frontEncoder))
        );
    }
}
