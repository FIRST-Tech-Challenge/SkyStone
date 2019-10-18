package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.drive.Direction;

//import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public final class DeviceMap {
    private static DeviceMap INSTANCE;
    private static Telemetry telemetry;

    private DcMotor leftTop, leftBottom, rightTop, rightBottom,
            leftIntake, rightIntake, conveyer;
    private DcMotor[] driveMotors;
    private DcMotor[] intakeMotors;
    private DcMotor[] allMotors;

    private BNO055IMUImpl imu;
    private VuforiaLocalizer vuforia;

    public DeviceMap(final HardwareMap map) {
        //for later

        INSTANCE = this;
        //CompletableFuture.allOf(
            setUpMotors(map);
            //setUpImu(map);
            //setUpVuforia(map);
        //).thenRunAsync(() -> {
            telemetry.addLine("Finished setting up all of the components");
            telemetry.update();
        //}, service);
    }

    /**
     * This will just set up all the driveMotors
     * @param map
     */
    private /*CompletableFuture<Void>*/void setUpMotors(HardwareMap map) {
        //return CompletableFuture.runAsync(() -> {
            telemetry.addLine("Setting up driveMotors");
            telemetry.update();
            leftTop = map.get(DcMotor.class, "LeftTop");
            leftBottom = map.get(DcMotor.class, "LeftBottom");
            rightTop = map.get(DcMotor.class, "RightTop");
            rightBottom = map.get(DcMotor.class, "RightBottom");


            leftIntake = map.get(DcMotor.class, "leftIntake");
            rightIntake = map.get(DcMotor.class, "rightIntake");

            conveyer = map.get(DcMotor.class, "conveyor");

            this.driveMotors = new DcMotor[]{leftTop, rightTop, leftBottom, rightBottom};
            this.intakeMotors = new DcMotor[] {
                     leftIntake, rightIntake, conveyer
            };
            this.allMotors = new DcMotor[]{leftTop, rightTop, leftBottom, rightBottom,
                    leftIntake, rightIntake, conveyer
            };
            for(DcMotor motor : this.allMotors) {
                motor.setPower(0);
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

            rightTop.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBottom.setDirection(DcMotorSimple.Direction.REVERSE);

            conveyer.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addLine("Finished setting up driveMotors");
        //}, service);

    }

    private /*CompletableFuture<Void>*/void setUpImu(HardwareMap map) {
        //return CompletableFuture.runAsync(() -> {
            telemetry.addLine("Setting up imu");
            telemetry.update();
            imu = map.get( BNO055IMUImpl.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;

            imu.initialize(parameters);
            while (!imu.isGyroCalibrated()) {

            }
        //}, service);
    }

    private void setUpVuforia(HardwareMap map) {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Context appContext = map.appContext;
        int cameraMonitorViewId = appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = KEY.V;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    //The methods below get all the driveMotors
    public DcMotor getLeftTop() {
        return leftTop;
    }
    public DcMotor getLeftBottom() {
        return leftBottom;
    }
    public DcMotor getRightTop() {
        return rightTop;
    }
    public DcMotor getRightBottom() {
        return rightBottom;
    }
    public DcMotor[] getDriveMotors() {
        return driveMotors;
    }

    public DcMotor getLeftIntake() {
        return leftIntake;
    }
    public DcMotor getRightIntake() {
        return rightIntake;
    }

    public DcMotor getConveyer() {
        return conveyer;
    }

    public DcMotor[] getIntakeMotors() {
        return intakeMotors;
    }

    public DcMotor[] getAllMotors() {
        return allMotors;
    }

    public BNO055IMUImpl getImu() {
        return imu;
    }
    public VuforiaLocalizer getVuforia() {
        return vuforia;
    }

    public static DeviceMap getInstance() {
        if(INSTANCE == null) throw new RuntimeException("the constructor must be called first");
        return INSTANCE;
    }
    public static DeviceMap getInstance(HardwareMap map) {
        if(INSTANCE == null && map != null) INSTANCE = new DeviceMap(map);
        return INSTANCE;
    }
    public static void setTelemetry(Telemetry ttelemetry) {
        telemetry = ttelemetry;
    }
}
