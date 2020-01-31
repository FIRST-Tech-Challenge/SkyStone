package org.firstinspires.ftc.teamcode.PID.localizer;

import android.support.annotation.NonNull;
import android.util.TimingLogger;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.encoderTicksToInches;

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
    private ExpansionHubEx hubMotors;
    //private ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    //private List<ExpansionHubMotor> motors;

    //private DcMotor leftEncoder0, rightEncoder0, frontEncoder0;
    private List<DcMotor> motors;

    private IMUBufferReader imuReader;

    Pose2d poseEstimate_new = new Pose2d(0, 0, 0);

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(1.0, DriveConstantsPID.ODOMETRY_TRACK_WIDTH / 2, 0), // left
                new Pose2d(1.0, -DriveConstantsPID.ODOMETRY_TRACK_WIDTH / 2, 0), // right
                new Pose2d(DriveConstantsPID.ODOMERY_FORWARD_OFFSET, -0.7, Math.toRadians(90)) // front
        ));
        if (DriveConstantsPID.USING_BULK_READ) {
            hubMotors = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub3");
            leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
            rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "liftTwo");
            frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        }
        else {
            leftEncoder = hardwareMap.dcMotor.get("leftIntake");
            rightEncoder = hardwareMap.dcMotor.get("liftTwo");
            frontEncoder = hardwareMap.dcMotor.get("rightIntake");
        }
        RobotLogger.dd(TAG, "StandardTrackingWheelLocalizer created");

        motors = Arrays.asList(leftEncoder, rightEncoder, frontEncoder);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        imuReader = IMUBufferReader.getSingle_instance(hardwareMap);
    }

    public static double encoderTicksToInches(int ticks) {
        RobotLogger.dd("StandardTrackingWheelLocalizer", "encoderTicksToInches: " + " ticks: " +
                Double.toString(ticks) + " inches: " + Double.toString(WHEEL_RADIUS * 2 * Math.PI *
                GEAR_RATIO * ticks / DriveConstantsPID.odoEncoderTicksPerRev));
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / DriveConstantsPID.odoEncoderTicksPerRev;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int x, y, z;
        RobotLogger.dd(TAG, "to getOdomWheelPositions (bulk? %d)", (DriveConstantsPID.USING_BULK_READ==true?1:0));

        if (DriveConstantsPID.USING_BULK_READ) {
            RevBulkData bulkData = hubMotors.getBulkInputData();
            if (bulkData == null) {
                RobotLogger.dd(TAG, "bulk data = null");
                return Arrays.asList(0.0, 0.0, 0.0, 0.0);
            }

            x = bulkData.getMotorCurrentPosition(leftEncoder) * (-1);
            y = bulkData.getMotorCurrentPosition(rightEncoder);
            z = bulkData.getMotorCurrentPosition(frontEncoder);
        }
        else
        {
            x = leftEncoder.getCurrentPosition() * (-1);
            y = rightEncoder.getCurrentPosition() ;
            z = frontEncoder.getCurrentPosition();
        }
        RobotLogger.dd(TAG, "getOdomWheelPositions");
        //RobotLogger.dd(TAG, "leftEncoder: " + x);
        //RobotLogger.dd(TAG, "rightEncoder: " + y);
        //RobotLogger.dd(TAG, "frontEncoder: " + (-1)*z);
        return Arrays.asList(
                encoderTicksToInches(x),
                encoderTicksToInches(y),
                encoderTicksToInches(-1*z)
        );
    }

    @Override
    public Pose2d getPoseEstimate() {
        //RobotLogger.dd(TAG, "getPoseEstimate: " + Double.toString(poseEstimate_new.getX()) + ", " + Double.toString(poseEstimate_new.getY()) + ", " +
        //        Double.toString(poseEstimate_new.getHeading()));
        return poseEstimate_new;
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        super.setPoseEstimate(pose2d);
        RobotLogger.dd(TAG, "setPoseEstimate: X "+Double.toString(pose2d.getX())+ ", Y "+Double.toString(pose2d.getY()));
    }

    @Override
    public void update() {
        super.update();
        Pose2d s_poseEstimate=super.getPoseEstimate();

        if (DriveConstantsPID.RUN_USING_IMU_LOCALIZER == true) {
            RobotLogger.dd(TAG, "to read IMU");
            poseEstimate_new = new Pose2d(s_poseEstimate.getX(), s_poseEstimate.getY(),
                    imuReader.getLatestIMUData());
            RobotLogger.dd(TAG, "using IMU: IMU heading " + Double.toString(poseEstimate_new.getHeading()) + " non-IMU heading: "
            + Double.toString(s_poseEstimate.getHeading()));
        }
        else {
            poseEstimate_new = s_poseEstimate;
            RobotLogger.dd(TAG, "not using IMU for heading");
        }
        RobotLogger.dd(TAG, "poseEstimate: "+Double.toString(poseEstimate_new.getX()) + ", " + Double.toString(poseEstimate_new.getY()) + ", " +
                Double.toString(poseEstimate_new.getHeading()));
    }
    public void finalize() throws Throwable {
        imuReader.cleanUP();
    }
}
