package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;

@Autonomous(name ="Troll vuforia", group="Auto Basic")
public class VuTroll extends LinearOpMode {

    public static final String tag = "VuCode";
    public static final String element = "TargetElement";
    public static final String key = "AdzMYbL/////AAABmflzIV+frU0RltL/ML+2uAZXg" +
            "JiIWerfe92N/AeH7QsWCOQqyKa2G+tUDcgvg8uE8QjHeBZPcpf5hAwlC5qCfvg76eB" +
            "oaa2bMMZ73hmTiHmr9fj3XmF4LWWZtDC6pWTFrzRAUguhlvgnck6Y4jjM16Px5TqgWYu" +
            "WnpcxNMHMyOXdnHLlyysyE64PVzoN7hgMXgbi2K8+pmTXvpV2OeLCag8fAj1Tgdm/kKGr" +
            "0TX86aQsC2RVjToZXr9QyAeyODi4l1KEFmGwxEoteNU8yqNbBGkPXGh/+IIm6/s/KxCJe" +
            "gg8qhxZDgO8110FRzwA5a6EltfxAMmtO0G8BB9SSkApxkcSzpyI0k2LxWof2YZG6x4H";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    OpenGLMatrix phoneLocation = new OpenGLMatrix();

    DriveTrain drive = new DriveTrain();

    float PerInch = 25.4f;
    float BotWidth = 18 * PerInch;
    float FTCFieldWidth = (12 * 12 - 2) * PerInch;

    int stonePos = 4;
    int[] shuffle = {0, 0, 0, 0, 0, 0};


    private void bootMatrix(OpenGLMatrix matrix, float dx, float dy, float dz,
                            float firstAngle, float secondAngle, float thirdAngle,
                            VuforiaTrackable track, boolean target) {
        matrix.translation(dx, dy, dz);
        matrix.multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.XZX,
                AngleUnit.DEGREES, firstAngle, secondAngle, thirdAngle));
        if (target) {
            track.setLocation(matrix);
        }
        RobotLog.ii(tag, "Red Target=%s", format(matrix));
    }

    private void maxBoot(OpenGLMatrix matrix, VuforiaTrackable firstTrackable, VuforiaTrackable secondTrackable) {
        OpenGLMatrix redLocation = new OpenGLMatrix();
        bootMatrix(redLocation, -FTCFieldWidth / 2, 0, 0, 90, 90,
                0, firstTrackable, true);

        OpenGLMatrix blueLocation = new OpenGLMatrix();
        bootMatrix(blueLocation, 0, FTCFieldWidth / 2, 0, 90, 0,
                0, secondTrackable, true);

        bootMatrix(matrix, BotWidth / 2, 0, 0, -90, 0,
                0, secondTrackable, false);
    }

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private List<VuforiaTrackable> formTrackables(VuforiaTrackables set) {
        allTrackables.addAll(set);
        return allTrackables;
    }

    int iSpot;

    private int checkPos (int posType) {
        for (int i = 0; i < shuffle.length; i++) {
            if (shuffle[i] == 4) {
                iSpot = i + posType;
                break;
            }
        }
        return shuffle[iSpot];
    }

    private void VuCheck (VuforiaTrackable track, int get, boolean second, int secondGet) {
        if (targetLock(track)) {
            shuffle[get] = stonePos;
            stonePos = 5;
        }
        else {
            //drive.nextCheck();
            if (second) {
                shuffle[secondGet] = stonePos;
                stonePos = 5;
            }
        }
    }

    private boolean broken () {
        if (stonePos == 5) {
            return true;
        }
        else {
            return false;
        }
    }

    private VuforiaTrackable setTarget (VuforiaTrackable trackable, VuforiaTrackables set, String
            targetName, int get) {
        trackable = set.get(get);
        trackable.setName(targetName);
        return trackable;
    }

    private boolean targetLock (VuforiaTrackable track) {
         if (track.getName() == element) {
             return true;
         }
         else {
             return false;
         }
    }

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable redTarget = stonesAndChips.get(0);
        VuforiaTrackable blueTarget = stonesAndChips.get(1);


        parameters.vuforiaLicenseKey = key;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        setTarget (redTarget, stonesAndChips, "RedTarget", 0);
        setTarget (blueTarget, stonesAndChips, "BlueTarget", 1);

        maxBoot(phoneLocation, redTarget, blueTarget);

        ((VuforiaTrackableDefaultListener)redTarget.getListener()).setPhoneInformation(phoneLocation,
                parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocation,
                parameters.cameraDirection);

        formTrackables (stonesAndChips);

        telemetry.addData("Waiting...", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        stonesAndChips.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {

                VuCheck (trackable, 0, false, 0);
                if (broken()) {
                    break;
                }
                VuCheck (trackable, 1, true, 2);
                if (broken()) {
                    break;
                }

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.
                        getListener()).isVisible() ? "Visible" : "Not Visible");

                OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener)trackable.
                        getListener()).getUpdatedRobotLocation();
                if (robotLocation != null) {
                    lastLocation = robotLocation;
                }
            }

            if (lastLocation != null) {
                telemetry.addData("Pos :", format(lastLocation));
            } else {
                telemetry.addData("Pos :", "Unknown");
            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public int getStonePos () {
        return checkPos(0);
    }

    public int getNextPos () {
        return checkPos(3);
    }

}
