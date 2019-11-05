package org.firstinspires.ftc.teamcode.teamcode.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class ZeroMap {

    private static final VuforiaLocalizer.CameraDirection cameraChoice = BACK;

    private static final boolean portrait = true;

    public static final String vuKey =
                    "AdzMYbL/////AAABmflzIV+frU0RltL/ML+2uAZXgJiI" +
                    "Werfe92N/AeH7QsWCOQqyKa2G+tUDcgvg8uE8QjHeBZPcpf5hAwlC5qCfvg76eBoaa2b" +
                    "MMZ73hmTiHmr9fj3XmF4LWWZtDC6pWTFrzRAUguhlvgnck6Y4jjM16Px5TqgWYuWnpcxNM" +
                    "HMyOXdnHLlyysyE64PVzoN7hgMXgbi2K8+pmTXvpV2OeLCag8fAj1Tgdm/kKGr0TX86aQsC2" +
                    "RVjToZXr9QyAeyODi4l1KEFmGwxEoteNU8yqNbBGkPXGh/+IIm6/s/KxCJegg8qhxZDgO8110F" +
                    "RzwA5a6EltfxAMmtO0G8BB9SSkApxkcSzpyI0k2LxWof2YZG6x4H";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    VuforiaTrackables targetsSkyStone;
    List<VuforiaTrackable> skyTrack;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private TFObjectDetector tfod;

    private static final float stoneZ = 2.00f * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    public int zeroLock;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    final float forwardDisp  = 4.0f * mmPerInch;
    final float verticalDisp = 8.0f * mmPerInch;
    final float leftDisp     = 0;

    private double[] zeroPoint = new double[5];

    private void setClimate(VuforiaTrackable track, float dx, float dy,
                            float dz, float firstAng, float secondAng, float thirdAng) {
        track.setLocation(OpenGLMatrix
                .translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ,
                        DEGREES, firstAng, secondAng, thirdAng)));
    }

    public void zeroInit(LinearOpMode opMode) {

        //-6, -4, -1

        zeroPoint[3] = 0;

        webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId =
                opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = vuKey;

        parameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        skyTrack = new ArrayList<VuforiaTrackable>();
        skyTrack.addAll(targetsSkyStone);

        setClimate(stoneTarget, 0, 0, stoneZ, 90, 0, -90);

        if (cameraChoice == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (portrait) {
            phoneXRotate = 90;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(forwardDisp, leftDisp, verticalDisp)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX,
                        DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : skyTrack) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera,
                    parameters.cameraDirection);
        }

        targetsSkyStone.activate();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(opMode);
        }

        if (tfod != null) {
            tfod.activate();
        }
    }

    public int zeroBrowse(LinearOpMode opMode) {
        while (!opMode.isStopRequested()) {

            targetVisible = false;
            for (VuforiaTrackable trackable : skyTrack) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    opMode.telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform =
                            ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
                    if (tfod != null) {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            opMode.telemetry.addData("Zeroes Detected",
                                    updatedRecognitions.size());
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                opMode.telemetry.addData(String.format("Labal (%d)", i),
                                        recognition.getLabel());
                                if (recognition.getLabel().equals("Skystone")) {
                                    zeroPoint[1] = recognition.getRight();
                                    zeroPoint[2] = recognition.getBottom();
                                }
                                opMode.telemetry.addData(String.format("  Left, Top (%d)", i),
                                        "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                opMode.telemetry.addData(String.format("  Right, Bottom (%d)", i),
                                        "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                            }
                            opMode.telemetry.update();
                        }
                    }

            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                opMode.telemetry.addData("Pos (in)", "{Z, X, Y} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch,
                        translation.get(2) / mmPerInch);
                zeroPoint[0] = translation.get(0) / mmPerInch;
                zeroPoint[1] = translation.get(1) / mmPerInch;
                zeroPoint[2] = translation.get(2) / mmPerInch;

                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ,
                        DEGREES);
                opMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} =" +
                                " %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle,
                        rotation.thirdAngle);
                zeroPoint[3] = 1;
            }

            else {
                opMode.telemetry.addData("Visible Target", "Lost");
                zeroPoint[0] = 0;
                zeroPoint[1] = 0;
                zeroPoint[2] = 0;
                zeroPoint[3] = 0;
            }

            List<Recognition> recon = tfod.getUpdatedRecognitions();
            if (recon != null && targetVisible) {
                zeroLock = 4;
            }

            else if (recon != null) {
                zeroLock = 3;
            }

            else { zeroLock = 1; }
            opMode.telemetry.update();

            opMode.telemetry.addData("Target :", zeroLock);
            opMode.telemetry.update();
        }

        return zeroLock;
    }

    public void zeroOut() {
        targetsSkyStone.deactivate();

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initTfod(LinearOpMode opMode) {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}