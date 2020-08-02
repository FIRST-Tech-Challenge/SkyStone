package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name = "Vuforia")
public class Vuforia extends LinearOpMode {

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    autoMethodsCopy auto = new autoMethodsCopy(DcMotor.RunMode.RUN_USING_ENCODER, hardwareMap);

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AUw51u3/////AAABmS2SebfPGERUmfixnpbS89g79T2cQLWzcEcMv6u+RTGzrrvHwTVug45aIF3UiYJXKVzy/zhBFDleEJD2gEjPWWDQeYDV9k3htKwbHofAiOwRfivq8h2ZJIGcmUwiNT40UAEeUvQlKZXTcIYTrxiYmN4tAKEjmH5zKoAUfLefScv9gDDMwTYCKVm1M45M2a1VdIu0pMdoaJKo2DRZ3B+D+yZurFO/ymNtyAWME+1eE9PWyulZUmuUw/sDphp13KrdNHNbDUXwbunQN7voVm2HE5fWrFNtX5euVaPy/jedXTiM5KBeosXuemMeppimcTLHFvyhSwOMZMRhPT1Gus487FRWMt479sn2EhexfDCcd0JG";

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException{

        setupVuforia();

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        auto.drive(0.7, 20);

        visionTargets.activate();

        while(opModeIsActive()){
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if(latestLocation != null){
                lastKnownLocation = latestLocation;
            }

            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            telemetry.update();
            idle();
        }
    }

    public void setupVuforia(){

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");

        target = visionTargets.get(0);
        target.setName("Skystone");
        target.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }

}
