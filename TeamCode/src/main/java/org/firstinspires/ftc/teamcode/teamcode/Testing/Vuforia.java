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

public class Vuforia extends LinearOpMode{

    public static final String tag = "VuCode";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    DriveTrain drive = new DriveTrain;

    float PerInch        = 25.4f;
    float BotWidth       = 18 * PerInch;
    float FTCFieldWidth  = (12 * 12 - 2) * PerInch;

    public final int posOne = 1;
    public final int posTwo = 2;
    public final int posThree = 3;

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

    private void maxBoot (OpenGLMatrix matrix, VuforiaTrackable firstTrackable, VuforiaTrackable secondTrackable) {
        OpenGLMatrix redLocation = new OpenGLMatrix();
        bootMatrix(redLocation, - FTCFieldWidth/2, 0, 0, 90, 90,
                0, firstTrackable, true);

        OpenGLMatrix blueLocation = new OpenGLMatrix();
        bootMatrix(blueLocation, 0, FTCFieldWidth/2, 0, 90, 0,
                0, secondTrackable, true);

        bootMatrix(matrix, BotWidth/2, 0, 0, -90, 0,
                0, secondTrackable, false);
    }

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdzMYbL/////AAABmflzIV+frU0RltL/ML+2uAZXg" +
                "JiIWerfe92N/AeH7QsWCOQqyKa2G+tUDcgvg8uE8QjHeBZPcpf5hAwlC5qCfvg76eB" +
                "oaa2bMMZ73hmTiHmr9fj3XmF4LWWZtDC6pWTFrzRAUguhlvgnck6Y4jjM16Px5TqgWYu" +
                "WnpcxNMHMyOXdnHLlyysyE64PVzoN7hgMXgbi2K8+pmTXvpV2OeLCag8fAj1Tgdm/kKGr" +
                "0TX86aQsC2RVjToZXr9QyAeyODi4l1KEFmGwxEoteNU8yqNbBGkPXGh/+IIm6/s/KxCJe" +
                "gg8qhxZDgO8110FRzwA5a6EltfxAMmtO0G8BB9SSkApxkcSzpyI0k2LxWof2YZG6x4H";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("StonesAndChips");
        VuforiaTrackable redTarget = stonesAndChips.get(0);
        redTarget.setName("RedTarget");

        VuforiaTrackable blueTarget  = stonesAndChips.get(1);
        blueTarget.setName("BlueTarget");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(stonesAndChips);

        OpenGLMatrix phoneLocation = new OpenGLMatrix();
        maxBoot(phoneLocation, redTarget, blueTarget);

        ((VuforiaTrackableDefaultListener)redTarget.getListener()).setPhoneInformation(phoneLocation,
                parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocation,
                parameters.cameraDirection);


        telemetry.addData("Waiting...", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        stonesAndChips.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.
                        getListener()).isVisible() ? "Visible" : "Not Visible");

                OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener)trackable.
                        getListener()).getUpdatedRobotLocation();
                if (robotLocation != null) {
                    lastLocation = robotLocation;
                }
            }

            if (lastLocation != null) {
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public int senseSkystone(LinearOpMode opMode) {

        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("StonesAndChips");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(stonesAndChips);
        stonesAndChips.activate();

        int stonePos = 0;

        while (opMode.opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {

                if (trackable.getName() == "TargetElement") {
                    stonePos = posOne;
                    break;
                } else {
                    drive.encoderDrive(opMode, .6,8, -8, -8);
                }
                if (stonePos == posOne) {
                    break;
                } else if (trackable.getName() == "TargetElement") {
                    stonePos = posTwo;
                } else {
                    drive.encoderDrive(opMode, .6,8, -8, -8);
                    stonePos = posThree;
                }

            }
        } return stonePos;
    }

}
