package org.firstinspires.ftc.teamcode.FTC2021;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.FTC2021.Modules.DriveModule;
import org.firstinspires.ftc.teamcode.FTC2021.Modules.FoundationMoverModule;
import org.firstinspires.ftc.teamcode.FTC2021.Modules.IntakeModule;
import org.firstinspires.ftc.teamcode.FTC2021.Modules.OdometryModule;
import org.firstinspires.ftc.teamcode.FTC2021.Modules.OuttakeModule;
import org.firstinspires.ftc.teamcode.FTC2021.Modules.PathModule;

import static org.firstinspires.ftc.teamcode.FTC2021.Accessories.DataDump.dump;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.BACKCLAMP_CLAMPED;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.FRONTCLAMP_RELEASED;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.OUTTAKE_SLIDE_RETRACTED;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.PUSHER_RETRACTED;

public class Robot {

    public HardwareCollection hardwareCollection;
    public OdometryModule odometryModule;
    public DriveModule driveModule;
    public IntakeModule intakeModule;
    public OuttakeModule outtakeModule;
    public PathModule pathModule;
    public FoundationMoverModule foundationMoverModule;

    public boolean isDebug = false;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    //imu
    private BNO055IMU imu;
    private Orientation angles;
    private Position position;

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

    public void update() {
        // put all sensors on rev 2
        hardwareCollection.refreshData2();
        hardwareCollection.updateTime();

        // update all modules
        odometryModule.update(this, hardwareCollection);
        pathModule.update(this);
        driveModule.update(this, hardwareCollection);
        intakeModule.update(this, hardwareCollection);
        outtakeModule.update(this, hardwareCollection);
        foundationMoverModule.update(this, hardwareCollection);

        // TODO does this actually work?
        if (linearOpMode.isStopRequested() && isDebug) {
            dump(this, hardwareCollection.currTime + "_ALL_DATA");
        }
    }

    // TODO move this elsewhere
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
}