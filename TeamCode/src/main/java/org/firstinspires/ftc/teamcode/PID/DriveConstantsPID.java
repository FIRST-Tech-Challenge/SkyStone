package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

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

    public static boolean RUN_USING_ODOMETRY_WHEEL = true;
    public static boolean RUN_USING_IMU_LOCALIZER = false;
    public static double odoEncoderTicksPerRev = 1540.0;
    private static String TAG = "DriveConstants";
    public static double tP = 0.5; //0.5;
    public static double tI = 0; //0.5;
    public static double tD = 0.12; //0.5;
    public static double hP = 2; //3.5; // heading co-efficiencies;
    public static double hI = 0; //0;
    public static double hD = 0.22; //0;

    public static double ODOMETRY_TRACK_WIDTH = 14.8;
    public static double ODOMERY_FORWARD_OFFSET = 5.5;
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
    public static final boolean RUN_USING_ENCODER = true;
    public static PIDCoefficients MOTOR_VELO_PID = null;   //35, 0.5, 2.5
    public static double kP = 35;
    public static double kI = 0.5;
    public static double kD = 3.0;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = (99.5 / 13.7) * (16 / 16); // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14.2;   //17

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0168;   //0.0115
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            65.0, 40.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );


    public static double encoderTicksToInches(double ticks) {
        double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev(); //2786 ticks per rev
        RobotLog.dd(TAG, "encoderTicksToInches: " + "ticks: " + Double.toString(ticks) + " inches: " + Double.toString(s));
        return s;
    }

    public static double rpmToVelocity(double rpm) {
        RobotLog.dd(TAG, "rpmToVelocity: " + "rpm " + Double.toString(rpm) +
                " v " + Double.toString(rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0));
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        RobotLog.dd(TAG, "MOTOR_CONFIG.getAchieveableMaxRPMFraction(): " + Double.toString(MOTOR_CONFIG.getAchieveableMaxRPMFraction()));
        RobotLog.dd(TAG, "MOTOR_CONFIG.getMaxRPM(): " + Double.toString(MOTOR_CONFIG.getMaxRPM()));
        RobotLog.dd(TAG, "getMaxRpm: hardcoded to: "+Double.toString((435.0 *
                (RUN_USING_ENCODER ? 0.683 : 1.0)))+" from: "+Double.toString(435.0));
        return 435.0 *
                (RUN_USING_ENCODER ? 0.683 : 1.0);
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        RobotLog.dd(TAG,  "getTicksPerSec "+Double.toString(435.0 * MOTOR_CONFIG.getTicksPerRev() / 60.0));
        return (435.0 * MOTOR_CONFIG.getTicksPerRev() / 60.0);
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
                RobotLog.dd(TAG, "returned prop str is invalid");


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

        RobotLog.dd(TAG, "Transitional PID   tP: "+Double.toString(tP) + " tI: "+Double.toString(tI) + " tD: " + Double.toString(tD));
        RobotLog.dd(TAG, "Heading PID   hP: "+Double.toString(hP) + " hI: "+Double.toString(hI) + " hD: " + Double.toString(hD));
        RobotLog.dd(TAG, "using IMU in localizer? : " + Integer.toString(RUN_USING_IMU_LOCALIZER?1:0));
        RobotLog.dd(TAG, "Driving wheel width? : " + Double.toString(TRACK_WIDTH));
        RobotLog.dd(TAG, "using Odometry? : " + Integer.toString(RUN_USING_ODOMETRY_WHEEL?1:0));
        RobotLog.dd(TAG, "Odometry wheel width? : " + Double.toString(ODOMETRY_TRACK_WIDTH));
        RobotLog.dd(TAG, "Odometry forward offset? " + Double.toString(ODOMERY_FORWARD_OFFSET));
        RobotLog.dd(TAG, "Odometry EncoderTicksPerRev? " + Double.toString(odoEncoderTicksPerRev));
    }
    public static void updateConstantsFromProperties()
    {
        if (RUN_USING_PARAMTER_FROM_PROPERTIES != true) {
            RobotLog.dd(TAG, "configured to NOT using property values");
            if (MOTOR_VELO_PID == null)
                MOTOR_VELO_PID = new PIDCoefficients(kP, kI, kD);
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
        v_double = (int) getTeamCodePropertyValue("debug.ftc.odom");
        if (v_double != Double.MAX_VALUE) {
            v_int = (int) v_double;
            RUN_USING_ODOMETRY_WHEEL = (v_int==0)?false:true;
        }
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

        v_double = getTeamCodePropertyValue("debug.ftc.tP");
        if (v_double != Double.MAX_VALUE)
            tP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.tI");
        if (v_double != Double.MAX_VALUE)
            tI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.tD");
        if (v_double != Double.MAX_VALUE)
            tD = v_double;

        v_double = getTeamCodePropertyValue("debug.ftc.hP");
        if (v_double != Double.MAX_VALUE)
            hP = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.hI");
        if (v_double != Double.MAX_VALUE)
            hI = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.hD");
        if (v_double != Double.MAX_VALUE)
            hD = v_double;
        v_double = getTeamCodePropertyValue("debug.ftc.odoTicksPerRev");
        if (v_double != Double.MAX_VALUE)
            odoEncoderTicksPerRev = v_double;

        if (MOTOR_VELO_PID == null)
            MOTOR_VELO_PID = new PIDCoefficients(kP, kI, kD);
        else
            RobotLog.dd(TAG, "kP, kI, kD has been set, not updated this time");

        printConstants();
    }
}
