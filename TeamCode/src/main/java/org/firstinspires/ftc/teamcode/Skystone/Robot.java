package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionState;
import org.firstinspires.ftc.teamcode.Skystone.Modules.DriveModule;
import org.firstinspires.ftc.teamcode.Skystone.Modules.FoundationMoverModule;
import org.firstinspires.ftc.teamcode.Skystone.Modules.IntakeModule;
import org.firstinspires.ftc.teamcode.Skystone.Modules.OdometryModule;
import org.firstinspires.ftc.teamcode.Skystone.Modules.OuttakeModule;
import org.firstinspires.ftc.teamcode.Skystone.Modules.PathModule;
import org.firstinspires.ftc.teamcode.Skystone.Modules.SpoolModule;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.CatmullRomSplineUtils;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Skystone.Constants.*;
import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.angleWrap;

public class Robot {

    public HardwareCollection hardwareCollection;
    public OdometryModule odometryModule;
    public DriveModule driveModule;
    public IntakeModule intakeModule;
    public OuttakeModule outtakeModule;
    public PathModule pathModule;
    public FoundationMoverModule foundationMoverModule;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    //imu
    private BNO055IMU imu;
    private Orientation angles;
    private Position position;

    private boolean isDebug = false;

    private StringBuilder odometryAllData = new StringBuilder();
    private StringBuilder odometryPoints = new StringBuilder();
    private StringBuilder splinePoints = new StringBuilder();
    private StringBuilder waypoints = new StringBuilder();

    /**
     * robot constructor, does the hardwareMaps
     *
     * @param hardwareMap
     * @param telemetry
     * @param linearOpMode
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;
        hardwareCollection = new HardwareCollection(hardwareMap);
        odometryModule = new OdometryModule();
        driveModule = new DriveModule();
        intakeModule = new IntakeModule();
        outtakeModule = new OuttakeModule();
        pathModule = new PathModule();
        foundationMoverModule = new FoundationMoverModule();
    }

    public void update(){
        // put all sensors on rev 2
        hardwareCollection.refreshData2();

        // update all modules
        odometryModule.update(this, hardwareCollection);
        pathModule.update(this);
        driveModule.update(this, hardwareCollection);
        intakeModule.update(hardwareCollection);
        foundationMoverModule.update(hardwareCollection);
    }

    public void initServos() {
        foundationMoverModule.isExtend = false;

        boolean isRetract = true;
        long outtakeExecutionTime = SystemClock.elapsedRealtime();
        long currentTime;

        hardwareCollection.frontClamp.setPosition(FRONTCLAMP_RELEASED);
        hardwareCollection.backClamp.setPosition(BACKCLAMP_CLAMPED);
        hardwareCollection.outtakeExtender.setPosition(OUTTAKE_SLIDE_RETRACTED);
        hardwareCollection.intakePusher.setPosition(PUSHER_RETRACTED);

        while (isRetract && !linearOpMode.isStopRequested()) {
            currentTime = SystemClock.elapsedRealtime();
            if (currentTime - outtakeExecutionTime >= 1500) {
                hardwareCollection.backClamp.setPosition(BACKCLAMP_CLAMPED);

                isRetract = false;
            }
        }
    }

    public void dumpPoints(String directoryName, String tripName) {
        if (!isDebug) {
            return;
        }
        writeToFile("" + directoryName, tripName + "_wayPoints.txt", getWayPoints());
        writeToFile("" + directoryName, tripName + "_odometry.txt", getOdometryPoints());
        writeToFile("" + directoryName, tripName + "_spline.txt", getSplinePoints());
        clearPoints();
    }

    public void clearPoints() {
        splinePoints = new StringBuilder();
        odometryPoints = new StringBuilder();
        waypoints = new StringBuilder();
    }

    public static void writeToFile(String directoryName, String fileName, String data) {
        File captureDirectory = new File(AppUtil.ROBOT_DATA_DIR, "/" + directoryName + "/");
        if (!captureDirectory.exists()) {
            boolean isFileCreated = captureDirectory.mkdirs();
            Log.d("DumpToFile", " " + isFileCreated);
        }
        Log.d("DumpToFile", " hey ");
        File file = new File(captureDirectory, fileName);
        try {
            FileOutputStream outputStream = new FileOutputStream(file);
            OutputStreamWriter writer = new OutputStreamWriter(outputStream);
            try {
                writer.write(data);
                writer.flush();
                Log.d("DumpToFile", data);
            } finally {
                outputStream.close();
                Log.d("DumpToFile", file.getAbsolutePath());
            }
        } catch (IOException e) {
            RobotLog.ee("TAG", e, "exception in captureFrameToFile()");
        }
    }

    public String getOdometryPoints() {
        odometryPoints.insert(0, "x y\n");
        return odometryPoints.toString();
    }

    public String getWayPoints() {
        waypoints.insert(0, "x y vX vY\n");
        return waypoints.toString();
    }

    public String getSplinePoints() {
        splinePoints.insert(0, "x y\n");
        return splinePoints.toString();
    }

    public void addSplinePoints(Point[] data) {
        if (!isDebug) {
            return;
        }
        for (int i = 0; i < data.length; i++) {
            addSplinePoints(data[i].x, data[i].y);
        }
    }

    public void addWaypoints(double[][] data) {
        if (!isDebug) {
            return;
        }
        for (int i = 0; i < data.length; i++) {
            addWaypoints(data[i][0], data[i][1], data[i][2], data[i][3]);
        }
    }

    public void addWaypoints(double x, double y, double vectorX, double vectorY) {
        if (!isDebug) {
            return;
        }
        waypoints.append(x);
        waypoints.append(" ");
        waypoints.append(y);
        waypoints.append(" ");
        waypoints.append(vectorX);
        waypoints.append(" ");
        waypoints.append(vectorY);
        waypoints.append("\n");
    }

    public void addSplinePoints(double x, double y) {
        if (!isDebug) {
            return;
        }
        splinePoints.append(x);
        splinePoints.append(" ");
        splinePoints.append(y);
        splinePoints.append("\n");
    }

    public void addOdometryPoints(double x, double y) {
        if (!isDebug) {
            return;
        }
        odometryPoints.append(x);
        odometryPoints.append(" ");
        odometryPoints.append(y);
        odometryPoints.append("\n");
    }

    public void addOdometryAllData(double leftEncoder, double rightEncoder, double mecanumEncoder, double x, double y, double theta) {
        if (!isDebug) {
            return;
        }
        odometryAllData.append(leftEncoder);
        odometryAllData.append(" ");
        odometryAllData.append(rightEncoder);
        odometryAllData.append(" ");
        odometryAllData.append(mecanumEncoder);
        odometryAllData.append(" ");
        odometryAllData.append(x);
        odometryAllData.append(" ");
        odometryAllData.append(y);
        odometryAllData.append(" ");
        odometryAllData.append(theta);
        odometryAllData.append("\n");
    }

    public String getOdometryAllData() {
        odometryAllData.insert(0, "l r m x y theta\n");
        return odometryAllData.toString();
    }

}