package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcontroller.ultro.listener.UltroVuforia;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;

//import java.util.concurrent.CompletableFuture;


public final class DeviceMap {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static DeviceMap INSTANCE;
    private static Telemetry telemetry;

    private DcMotor leftTop, leftBottom, rightTop, rightBottom,
            leftIntake, rightIntake, conveyer;
    private DcMotor[] driveMotors;
    private DcMotor[] intakeMotors;
    private DcMotor[] allMotors;

    private BNO055IMUImpl imu;
    private UltroVuforia vuforia;
    private TFObjectDetector tfod;
    private OpenCvCamera camera;

    public DeviceMap(final HardwareMap map) {
        //for later

        INSTANCE = this;
        //CompletableFuture.allOf(
        //).thenRunAsync(() -> {
            telemetry.addLine("Finished setting up all of the components");
            telemetry.update();
        //}, service);
    }
    public void setupAll(HardwareMap map) {
        setUpMotors(map);
        setUpImu(map);
        setUpVuforia(map);
        initTfod(map);

    }
    /**
     * This will just set up all the driveMotors
     * @param map
     */
    public  /*CompletableFuture<Void>*/void setUpMotors(HardwareMap map) {
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

    public /*CompletableFuture<Void>*/void setUpImu(HardwareMap map) {
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

    public void initOpenCV(HardwareMap map) {
        Context appContext = map.appContext;
        int cameraMonitorViewId = appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());
        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();

    }
    public void setUpVuforia(HardwareMap map) {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Context appContext = map.appContext;
        int cameraMonitorViewId = appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXOOSnD/////AAAAGQJxEkkI+EqdsPLGvMTzmRoBoW5g1d+xB7S06ymDvyNs48WqxFYMIeVVTSdkgHLwjsFVHBgVACzNkxwNXQ5zO9ED9CS11B+/cDS7CAbFLYzTlbDsyeX/NaEIOBm9v4ErL7uM6xtTXoKFKyJiFJiRe3ux4A6MXRHrvnkGqaJ9fBle9B2OTuyOe62gv5PFuTvil1DSvBosIXQmTiHosTW39OBcR81+ykeJXeiUA8vwBp1ueAP+9eYTP0U6VWDwRm0dUJ+CbvVIriauyP6pWj7dnCufomc58E7GiJbsLEN+Uj3H7J1uJG3K6O8azwEc+8BKw8tTsEdg+lJ47CbsYR6fFFfHVwA3K193cnC5U/RmRnX0";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = new UltroVuforia(parameters);
    }

    public void initTfod(HardwareMap hardwareMap) {
        if(vuforia == null) {
            telemetry.addLine("vuforia is null!");
            telemetry.update();
            return;
        }else if(!ClassFactory.getInstance().canCreateTFObjectDetector()) {
            telemetry.addLine("TFLOW not supported");
            telemetry.update();
            return;
        }
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
        tfod.activate();
    }

    public void deactivateOpenCV() {
        if(camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
    public void deactivateVuforia() {
        if(vuforia != null)
            vuforia.close();
    }
    public void deactivateTfod() {
        if(tfod != null)
            tfod.deactivate();
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

    public static Telemetry getTelemetry() {
        return telemetry;
    }

    public static void setTelemetry(Telemetry ttelemetry) {
        telemetry = ttelemetry;
    }

    public TFObjectDetector getTfod() {
        return tfod;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }
}
