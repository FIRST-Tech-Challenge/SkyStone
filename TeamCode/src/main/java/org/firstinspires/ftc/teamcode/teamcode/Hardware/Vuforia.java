package org.firstinspires.ftc.teamcode.teamcode.Hardware;
//package org.firstinspires.ftc.robotcontroller.external.samples;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="VuTest", group= "Vision")
public class Vuforia extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection cameraChoice = BACK;
    private static final boolean portrait = false  ;

    private static final String vuKey =
                    "AdzMYbL/////AAABmflzIV+frU0RltL/ML+2uAZXgJiI" +
                    "Werfe92N/AeH7QsWCOQqyKa2G+tUDcgvg8uE8QjHeBZPcpf5hAwlC5qCfvg76eBoaa2b" +
                    "MMZ73hmTiHmr9fj3XmF4LWWZtDC6pWTFrzRAUguhlvgnck6Y4jjM16Px5TqgWYuWnpcxNM" +
                    "HMyOXdnHLlyysyE64PVzoN7hgMXgbi2K8+pmTXvpV2OeLCag8fAj1Tgdm/kKGr0TX86aQsC2" +
                    "RVjToZXr9QyAeyODi4l1KEFmGwxEoteNU8yqNbBGkPXGh/+IIm6/s/KxCJegg8qhxZDgO8110F" +
                    "RzwA5a6EltfxAMmtO0G8BB9SSkApxkcSzpyI0k2LxWof2YZG6x4H";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    private static final float stoneZ = 2.00f * mmPerInch;

    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;
    private static final float bridgeRotZ = 180;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    final float forwardDisp  = 4.0f * mmPerInch;
    final float verticalDisp = 8.0f * mmPerInch;
    final float leftDisp     = 0;

    private void setClimate(VuforiaTrackable track, float dx, float dy,
                            float dz, float firstAng, float secondAng, float thirdAng) {
        track.setLocation(OpenGLMatrix
                .translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ,
                        DEGREES, firstAng, secondAng, thirdAng)));
    }

    @Override
    public void runOpMode() {

        double[] zeroPoint;
        zeroPoint = new double[5];
        zeroPoint[3] = 0;

        //Add webCheck
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = vuKey;

        parameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> skyTrack = new ArrayList<VuforiaTrackable>();
        skyTrack.addAll(targetsSkyStone);

        setClimate(stoneTarget, 0, 0, stoneZ, 90, 0, -90);

        if (cameraChoice == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (portrait) {
            phoneXRotate = 90 ;
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
        while (!isStopRequested()) {

            targetVisible = false;
            for (VuforiaTrackable trackable : skyTrack) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform =
                            ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch,
                        translation.get(2) / mmPerInch);
                zeroPoint[0] = translation.get(0) / mmPerInch;
                zeroPoint[1] = translation.get(1) / mmPerInch;
                zeroPoint[2] = translation.get(2) / mmPerInch;

                //if (zeroPoint[0] )

                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ,
                        DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} =" +
                        " %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle,
                        rotation.thirdAngle);
                zeroPoint[3] = 1;
            }

            else {
                telemetry.addData("Visible Target", "none");
                zeroPoint[0] = 0;
                zeroPoint[1] = 0;
                zeroPoint[2] = 0;
                zeroPoint[3] = 0;
            }

            telemetry.update();
        }

        targetsSkyStone.deactivate();
        //return zeroPoint;
    }
}