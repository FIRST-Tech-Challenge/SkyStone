package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.List;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstantsPID {

    public static final boolean RUN_USING_PARAMTER_FROM_PROPERTIES = false;

    public static boolean RUN_USING_ODOMETRY_WHEEL = false;
    public static boolean RUN_USING_IMU_LOCALIZER = true;
    public static boolean BRAKE_ON_ZERO = true;
    public static boolean USING_BULK_READ = true;
    public static boolean USING_STRAFE_DIAGNAL = true;
    public static boolean RESET_FOLLOWER = true;
    public static double odoEncoderTicksPerRev = 1565.0;
    public static double imuPollingInterval = 10;

    public static boolean ENABLE_LOGGING = false;
    private static String TAG = "DriveConstants";

    public static double txP = 5.0; //translational x/y co-efficients
    public static double txI = 0.5;
    public static double txD = 0.0;
    public static double tyP = 5.0;
    public static double tyI = 10.0;
    public static double tyD = 0.00001;
    public static double hP = 10;    // heading co-efficients;
    public static double hI = 0.5;
    public static double hD = 0.00001;

    public static double stxP = 20; //translational x/y co-efficients
    public static double stxI = 1;
    public static double stxD = 0.75;
    public static double styP = 15;
    public static double styI = 0.5;
    public static double styD = 1;
    public static double shP = 6;    // heading co-efficients;
    public static double shI = 2;
    public static double shD = 0.4;
    public static double strafeTimeDistanceRatio = 0.093; // duration for power to achieve strafe distance;
    public static double strafeMotorPower = 0.19;
    public static double rear_ratio = 1.105;

    public static double ODOMETRY_TRACK_WIDTH = 14.8;
    public static double ODOMERY_FORWARD_OFFSET = -5.5;
    public static double HARDCODED_TICKS_PER_REV = 383.6; //MOTOR_CONFIG.getTicksPerRev();
    public static double MAX_RPM_FROM_SPEC = 435.0;
    public static double HARDCODED_RPM_RATIO = 0.683; //0.72215; // 0.666;///0.6514;//*MAX_RPM_FROM_SPEC; //283.4; //MOTOR_CONFIG.getMaxRPM();

    /*
     * The type of motor used on the drivetrain. While the SDK has definitions for many common
     * motors, there may be slight gear ratio inaccuracies for planetary gearboxes and other
     * discrepancies. Additional motor types can be defined via an interface with the
     * @DeviceProperties and @MotorType annotations.
     */
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static boolean RUN_USING_ENCODER = true;
    public static double kP = 1.72;
    public static double kI = 0.172;
    public static double kD = 0.0;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(kP, kI, kD);   //35, 0.5, 2.5

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1.0;//(99.5 / 13.7) * (16 / 16); // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14.2;   //17

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0111;   //0.0115
    public static double kA = 0;
    public static double kStatic = 0;
	public static double TEST_DISTANCE = 48;
    public static double TEST_DISTANCE_0 = 24;
	public static double maxVel = 75.0; //90.0
	public static double maxAccel = 40.0;   //35.0
    public static double strafeMaxVel = 70.0; //40.0
    public static double strafeMaxAccel = 35.0;   //20.0
    public static double maxAngVel = 135.0;
    public static double maxAngAccel = 90.0;
	public static boolean keep_vuforia_running = false;
	public static boolean USE_VUFORIA_LOCALIZER = false;
    public static boolean RECREATE_DRIVE_AND_BUILDER = false;
    public static boolean drvCorrection = false;
    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            maxVel, maxAccel, 0.0,
            Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), 0.0
    );

    public static DriveConstraints STRAFE_BASE_CONSTRAINTS = new DriveConstraints(
            strafeMaxVel, strafeMaxAccel, 0.0,    //20.0, 10.0, 0.0
            Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), 0.0
    );

    public static DriveConstraints ROTATION_CONSTRAINTS = new DriveConstraints(
            maxVel, maxAccel, 0.0,
            Math.toRadians(270.0), Math.toRadians(180.0), 0.0
    );

    public static double encoderTicksToInches(double ticks) {
        //double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
        double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / HARDCODED_TICKS_PER_REV; //MOTOR_CONFIG.getTicksPerRev();
        //RobotLog.dd(TAG, "encoderTicksToInches: " + "ticks: " + Double.toString(ticks) + " inches: " + Double.toString(s));
        return s;
    }

    public static double rpmToVelocity(double rpm) {
        double s = rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        RobotLog.dd(TAG, "rpmToVelocity: " + "rpm " + Double.toString(rpm) + " v " + Double.toString(s));
        return s;
    }

    public static double getMaxRpm() {
        RobotLog.dd(TAG, "MOTOR_CONFIG.getAchieveableMaxRPMFraction(): " + Double.toString(MOTOR_CONFIG.getAchieveableMaxRPMFraction()));
        RobotLog.dd(TAG, "MOTOR_CONFIG.getMaxRPM(): " + Double.toString(MOTOR_CONFIG.getMaxRPM()));
        double t = MOTOR_CONFIG.getMaxRPM() *
                (RUN_USING_ENCODER ? MOTOR_CONFIG.getAchieveableMaxRPMFraction() : 1.0);
        t = MAX_RPM_FROM_SPEC * (RUN_USING_ENCODER ? HARDCODED_RPM_RATIO : 1.0);
        RobotLog.dd(TAG, "getMaxRpm: hardcoded to: "+Double.toString((t))+" from: "+Double.toString(MAX_RPM_FROM_SPEC));
        return t;
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        //double t = MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0;
        //double t = MOTOR_CONFIG.getMaxRPM() * HARDCODED_TICKS_PER_REV / 60.0;
        double t = getMaxRpm() * HARDCODED_TICKS_PER_REV / 60.0;
        RobotLog.dd(TAG,  "getTicksPerSec "+Double.toString(t));
        return t;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        RobotLog.dd(TAG, "getTicksPerSec "+Double.toString(getTicksPerSec()));
        return 32767 / getTicksPerSec();
    }

    private static double getTeamCodePropertyValue(String prop_str) {
        double value = Double.MAX_VALUE;
        try {
            Process proc = Runtime.getRuntime().exec(new String[]{"/system/bin/getprop", prop_str});
            BufferedReader reader = new BufferedReader(new InputStreamReader(proc.getInputStream()));
            String t = reader.readLine();
            if (t != null && !t.trim().isEmpty())
            {
                RobotLog.dd(TAG, prop_str + " : "  + t);
                value = Double.parseDouble(t);
                //RobotLog.dd(TAG, "returned "+Double.toString(value));
            }
            else
                RobotLog.dd(TAG, "returned prop str is invalid: " + prop_str);


        } catch(IOException e) {
            RobotLog.dd(TAG, "getprop failed for " + prop_str);
            e.printStackTrace();
        }
        return value;
    }

    private static void printConstants()
    {
        if (MOTOR_VELO_PID == null)
        {
            RobotLog.dd(TAG, "MOTOR_VELO_PID = null, Velocity PID is not used!");
        }
        else
        {
            RobotLog.dd(TAG, "Velocity PID    kP: "  + Double.toString(MOTOR_VELO_PID.kP) + ", kI: "  + Double.toString(MOTOR_VELO_PID.kI) + ", kD: "  + Double.toString(MOTOR_VELO_PID.kD));
        }
        RobotLog.dd(TAG, "(non-strafe) maxVel: %f, maxAccel: %f", maxVel, maxAccel);
        RobotLog.dd(TAG, "(strafe) maxVel: %f, maxAccel: %f", strafeMaxVel, strafeMaxAccel);
        RobotLog.dd(TAG, "xTransitional PID   txP: "+Double.toString(txP) + " txI: "+Double.toString(txI) + " txD: " + Double.toString(txD));
        RobotLog.dd(TAG, "yTransitional PID   tyP: "+Double.toString(tyP) + " tyI: "+Double.toString(tyI) + " tyD: " + Double.toString(tyD));
        RobotLog.dd(TAG, "Heading PID   hP: "+Double.toString(hP) + " hI: "+Double.toString(hI) + " hD: " + Double.toString(hD));
        RobotLog.dd(TAG, "test distance: " + Double.toString(TEST_DISTANCE) + "  " + Double.toString(TEST_DISTANCE_0));
        RobotLog.dd(TAG, "using IMU in localizer? : " + Integer.toString(RUN_USING_IMU_LOCALIZER?1:0));
        RobotLog.dd(TAG, "IMU polling interval? : " + Double.toString(imuPollingInterval));
        RobotLog.dd(TAG, "correcting drv in automonous? : " + Integer.toString(drvCorrection?1:0));
        RobotLog.dd(TAG, "using STRAFE in diagonal move? : " + Integer.toString(USING_STRAFE_DIAGNAL?1:0));
        RobotLog.dd(TAG, "reset follower? : " + Integer.toString(RESET_FOLLOWER?1:0));
        RobotLog.dd(TAG, "using Vuforia in localizer (override IMU and odom)? : " + Integer.toString(USE_VUFORIA_LOCALIZER?1:0));
        RobotLog.dd(TAG, "Driving wheel width? : " + Double.toString(TRACK_WIDTH));
        RobotLog.dd(TAG, "using Odometry? : " + Integer.toString(RUN_USING_ODOMETRY_WHEEL?1:0));
        RobotLog.dd(TAG, "using Bulk read? : " + Integer.toString(USING_BULK_READ?1:0));
        RobotLog.dd(TAG, "recreate drive? : " + Integer.toString(RECREATE_DRIVE_AND_BUILDER?1:0));
        RobotLog.dd(TAG, "Odometry wheel width? : " + Double.toString(ODOMETRY_TRACK_WIDTH));
        RobotLog.dd(TAG, "Odometry forward offset? " + Double.toString(ODOMERY_FORWARD_OFFSET));
        RobotLog.dd(TAG, "Odometry EncoderTicksPerRev? " + Double.toString(odoEncoderTicksPerRev));
        RobotLog.dd(TAG, "Strafing paramters: ");
        RobotLog.dd(TAG, "xTransitional PID   txP: "+Double.toString(stxP) + " txI: "+Double.toString(stxI) + " txD: " + Double.toString(stxD));
        RobotLog.dd(TAG, "yTransitional PID   tyP: "+Double.toString(styP) + " tyI: "+Double.toString(styI) + " tyD: " + Double.toString(styD));
        RobotLog.dd(TAG, "Heading PID   hP: "+Double.toString(shP) + " hI: "+Double.toString(shI) + " hD: " + Double.toString(shD));
        RobotLog.dd(TAG, "strafeTimeDistanceRat: " + Double.toString(strafeTimeDistanceRatio));
        RobotLog.dd(TAG, "strafeMotorPower:  " + Double.toString(strafeMotorPower));
        RobotLog.dd(TAG, "rear_ratio:  " + Double.toString(rear_ratio));
        RobotLog.dd(TAG, "enabling Logging? : " + Integer.toString(ENABLE_LOGGING?1:0));
    }
    public static void updateConstantsFromProperties()
    {
        if (RUN_USING_PARAMTER_FROM_PROPERTIES != true) {
            RobotLog.dd(TAG, "configured to NOT using property values");
            if (MOTOR_VELO_PID == null)
                MOTOR_VELO_PID = new PIDCoefficients(kP, kI, kD);
            if (BASE_CONSTRAINTS == null)
                BASE_CONSTRAINTS = new DriveConstraints(
                        maxVel, maxAccel, 0.0,
                        Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), 0.0
                );
            if (STRAFE_BASE_CONSTRAINTS == null)
                STRAFE_BASE_CONSTRAINTS = new DriveConstraints(
                        strafeMaxVel, strafeMaxAccel, 0.0,
                        Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), 0.0
                );
            printConstants();
            return;
        }

        int v_int = 0;
        double v_double = getTeamCodePropertyValue("debug.ftc.imu");
        if (v_double != Double.MAX_VALUE)
        {
            v_int = (int) v_double;
            RUN_USING_IMU_LOCALIZER = (v_int==0)?false:true;
        }
        v_double = (int) getTeamCodePropertyValue("debug.ftc.recreateDrv");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            RECREATE_DRIVE_AND_BUILDER = (v_int==0)?false:true;
        }

        v_double = getTeamCodePropertyValue("debug.ftc.vuforia");
        if (v_double != Double.MAX_VALUE)
        {
            v_int = (int) v_double;
            USE_VUFORIA_LOCALIZER = (v_int==0)?false:true;
        }
        v_double = (int) getTeamCodePropertyValue("debug.ftc.odom");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            RUN_USING_ODOMETRY_WHEEL = (v_int==0)?false:true;
        }
        v_double = (int) getTeamCodePropertyValue("debug.ftc.logging");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            ENABLE_LOGGING = (v_int==0)?false:true;
        }
        v_double = (int) getTeamCodePropertyValue("debug.ftc.strafeDiag");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            USING_STRAFE_DIAGNAL = (v_int==0)?false:true;
        }
        v_double = (int) getTeamCodePropertyValue("debug.ftc.resetfollow");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            RESET_FOLLOWER = (v_int==0)?false:true;
        }
        v_double = (int) getTeamCodePropertyValue("debug.ftc.bulk");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            USING_BULK_READ = (v_int==0)?false:true;
        }
        v_double = (int) getTeamCodePropertyValue("debug.ftc.drvCorrect");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            drvCorrection = (v_int==0)?false:true;
        }
        v_double = getTeamCodePropertyValue("debug.ftc.brake");
        if (v_double != Double.MAX_VALUE)
        {
            v_int = (int) v_double;
            BRAKE_ON_ZERO = (v_int==0)?false:true;
        }
        v_double = getTeamCodePropertyValue("debug.ftc.imuInterval");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            imuPollingInterval = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.maxVel");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            maxVel = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.maxAccel");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            maxAccel = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.strafeMaxVel");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            strafeMaxVel = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.strafeMaxAccel");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            strafeMaxAccel = v_double;

        BASE_CONSTRAINTS = new DriveConstraints(
                maxVel, maxAccel, 0.0,
                Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), 0.0
        );

        STRAFE_BASE_CONSTRAINTS = new DriveConstraints(
                strafeMaxVel, strafeMaxAccel, 0.0,
                Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), 0.0
        );

        v_double = getTeamCodePropertyValue("debug.ftc.kV");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            kV = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.kP");
        if (v_double != Double.MAX_VALUE)
            kP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.kI");
        if (v_double != Double.MAX_VALUE)
            kI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.kD");
        if (v_double != Double.MAX_VALUE)
            kD = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.odomTrackwidth");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            ODOMETRY_TRACK_WIDTH = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.odomForwardOffset");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            ODOMERY_FORWARD_OFFSET = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.trackwidth");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
            TRACK_WIDTH = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.txP");
        if (v_double != Double.MAX_VALUE)
            txP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.txI");
        if (v_double != Double.MAX_VALUE)
            txI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.txD");
        if (v_double != Double.MAX_VALUE)
            txD = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.tyP");
        if (v_double != Double.MAX_VALUE)
            tyP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.tyI");
        if (v_double != Double.MAX_VALUE)
            tyI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.tyD");
        if (v_double != Double.MAX_VALUE)
            tyD = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.hP");
        if (v_double != Double.MAX_VALUE)
            hP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.hI");
        if (v_double != Double.MAX_VALUE)
            hI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.hD");
        if (v_double != Double.MAX_VALUE)
            hD = v_double;


        v_double = getTeamCodePropertyValue("debug.ftc.stxP");
        if (v_double != Double.MAX_VALUE)
            stxP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.stxI");
        if (v_double != Double.MAX_VALUE)
            stxI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.stxD");
        if (v_double != Double.MAX_VALUE)
            stxD = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.styP");
        if (v_double != Double.MAX_VALUE)
            styP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.styI");
        if (v_double != Double.MAX_VALUE)
            styI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.styD");
        if (v_double != Double.MAX_VALUE)
            styD = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.shP");
        if (v_double != Double.MAX_VALUE)
            shP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.shI");
        if (v_double != Double.MAX_VALUE)
            shI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.shD");
        if (v_double != Double.MAX_VALUE)
            shD = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.strafeMotorPower");
        if (v_double != Double.MAX_VALUE)
            strafeMotorPower = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.strafeTimeDistanceRat");
        if (v_double != Double.MAX_VALUE)
            strafeTimeDistanceRatio = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.rear_ratio");
        if (v_double != Double.MAX_VALUE)
            rear_ratio = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.odoTicksPerRev");
        if (v_double != Double.MAX_VALUE)
            odoEncoderTicksPerRev = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.distance");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
        {
            TEST_DISTANCE = v_double;
        }
        v_double = getTeamCodePropertyValue("debug.ftc.distance0");
        if (v_double != 0 && v_double != Double.MAX_VALUE)
        {
            TEST_DISTANCE_0 = v_double;
        }

        if (MOTOR_VELO_PID == null)
        {
        }
        else {
            MOTOR_VELO_PID = null;
            RobotLog.dd(TAG, "kP, kI, kD has been set, updated this time");
        }

        v_double = getTeamCodePropertyValue("debug.ftc.useOld");
        if (v_double == 1.0)
        {
            RUN_USING_ODOMETRY_WHEEL = true;
            RUN_USING_IMU_LOCALIZER = false;
            BRAKE_ON_ZERO = false;
            odoEncoderTicksPerRev = 1540.0;
            txP = 0.5; //translational x/y co-efficients
            txI = 0;
            txD = 0.11;
            tyP = 1.2;
            tyI = 0;
            tyD = 1.0;
            hP = 2;    // heading co-efficients;
            hI = 0;
            hD = 0.22;
            ODOMETRY_TRACK_WIDTH = 14.6;
            ODOMERY_FORWARD_OFFSET = -5.5;
            HARDCODED_TICKS_PER_REV = 383.6; //MOTOR_CONFIG.getTicksPerRev();
            MAX_RPM_FROM_SPEC = 435.0;
            HARDCODED_RPM_RATIO = 0.683; //0.72215; // 0.666;///0.6514;//*MAX_RPM_FROM_SPEC; //283.4; //MOTOR_CONFIG.getMaxRPM();
            RUN_USING_ENCODER = true;
            kP = 23.0;
            kI = 0.5;
            kD = 3.0;
            WHEEL_RADIUS = 2;
            GEAR_RATIO = 1.0;//(99.5 / 13.7) * (16 / 16); // output (wheel) speed / input (motor) speed
            TRACK_WIDTH = 14.2;   //17
            kV = 0.0166;   //0.0115
            kA = 0;
            kStatic = 0;
            TEST_DISTANCE = 72;
        }
        MOTOR_VELO_PID = new PIDCoefficients(kP, kI, kD);
        printConstants();
    }
    // duration in milli-seconds;
    public static void strafeDistance(HardwareMap hardwareMap, double distance, boolean left) {
        DcMotorEx leftFront, leftRear, rightRear, rightFront;
        List<DcMotorEx> motors;
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        RobotLog.dd(TAG, "set power for strafe: " + Double.toString(strafeMotorPower) + " distance: " + Double.toString(distance));

        long duration = (long) (1000 * distance * strafeTimeDistanceRatio);
        RobotLog.dd(TAG, "set power for strafe: " + Double.toString(strafeMotorPower) + " duration: " + Double.toString(duration));

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (left) {
            leftFront.setPower(-1 * strafeMotorPower);
            leftRear.setPower(strafeMotorPower * rear_ratio);
            rightRear.setPower(-1 * strafeMotorPower * rear_ratio);
            rightFront.setPower(strafeMotorPower);
        } else {
            leftFront.setPower(strafeMotorPower);
            leftRear.setPower(-1 * strafeMotorPower * rear_ratio);
            rightRear.setPower(strafeMotorPower * rear_ratio);
            rightFront.setPower(-1 * strafeMotorPower);
        }

        try {
            Thread.sleep(duration);
        } catch (Exception e) {
            e.printStackTrace();
            // Prints what exception has been thrown
            System.out.println(e);
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        for (DcMotorEx motor : motors) {
            if (BRAKE_ON_ZERO == true)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
