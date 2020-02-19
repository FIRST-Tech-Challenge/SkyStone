package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.ConceptVuforiaSkyStoneNavigationWebcam;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@Autonomous(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends LinearOpMode {

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final float mmPerInch        = 25.4f;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final float stoneZ = 2.00f * mmPerInch;

    private enum SkystonePosition {
        R1,
        R2,
        R3,
        B1,
        B2,
        B3,
    }

    private SkystonePosition Skystone = SkystonePosition.B1;

    private static final String VUFORIA_KEY =
            "AU3bRpT/////AAABmXojhhSQw0t1v5y+m3h8AKkWsNNBNdx9pchrAjkbpsDQzCf6qEeBQ52HTxfNpr573118aw6sGlJT04GBul7QkXGdnghzUmFXconiUSC6bn3OOYXXlHsrF9r7czfAWv0Xab7R7ho+K+mS+l2o4oyfxxEu7LttxCIR4J5nDRcIA07LeTVwun2dvnL4xtufk00tOuO7DKQhmDc0iSY8hzi2DgDCYmn7PAvWBmgxUc3pSxbKE4Q8P0nn/S/2WVDiBmOon3mS5LWnNHQqobCmnoaR8zRZYKbPZ2gXE+uVi9XFtWeH63irNEMn0U/u4SwmNHPJfNG2j4Y7Qt2zMO+Vvg9s9ILLUlQRiGRePrl4bEHnORai";

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    public enum positionRedSkystone {
        LEFT,
        CENTER,
        RIGHT,
    }

    public enum positionBlueSkystone {
        LEFT,
        CENTER,
        RIGHT,
    }

    public positionRedSkystone PRS;
    public positionBlueSkystone PBS;

    DriveControl Drive = new DriveControl();
    private ConceptVuforiaSkyStoneNavigationWebcam Webcam = new ConceptVuforiaSkyStoneNavigationWebcam();
    UserControl User = new UserControl();

    private enum SkystonePositionTest {
        R1,
        R2,
        R3,
        B1,
        B2,
        B3,
    }

    private SkystonePositionTest SkystoneTest = SkystonePositionTest.B1;

    @Override
    public void runOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        waitForStart();

        targetsSkyStone.activate();

        // create and initialize sub-assemblies
        Drive.init(this);
        Webcam.init();
        User.init(this);

        boolean AllianceColor;

        /*AllianceColor = User.getRedBlue("Alliance Color");
        if (AllianceColor == true) {
            if (Webcam.PRS == Webcam.PRS.RIGHT) {
                Skystone = SkystonePosition.R1;
            } else if (Webcam.PRS == Webcam.PRS.CENTER) {
                Skystone = SkystonePosition.R2;
            } else {
                Skystone = SkystonePosition.R3;
            }
        } else if (AllianceColor == false) {
            if (Webcam.PBS == Webcam.PBS.LEFT) {
                Skystone = SkystonePosition.B1;
            } else if (Webcam.PBS == Webcam.PBS.CENTER) {
                Skystone = SkystonePosition.B2;
            } else {
                Skystone = SkystonePosition.B3;
            }
        }*/

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();



        while (opModeIsActive()) {



                    // check all the trackable targets to see which one (if any) is visible.
                    targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }
                    //Y and x are reversed, y is side to side, x is up and down
                    // Provide feedback as to where the robot is located (if we know).
                    if (targetVisible) {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                        double yPosition = translation.get(1);
                        //String positionSkystone = "";
                        if (yPosition > 4.5) {
                            PRS = positionRedSkystone.RIGHT;
                            telemetry.addLine("Red Right");
                        } else if (yPosition < 4.5) {
                            PRS = positionRedSkystone.CENTER;
                            telemetry.addLine("Red Center");
                        } else {
                            PRS = positionRedSkystone.LEFT;
                            telemetry.addLine("Red Left");
                        }
                        //Red not working, switching between right and center at y=0 not 4.5 as directed
                        if (yPosition < -3) {
                            PBS = positionBlueSkystone.LEFT;
                            telemetry.addLine("Blue Left");
                        } else if (yPosition > -3) {
                            PBS = positionBlueSkystone.CENTER;
                            telemetry.addLine("Blue Center");
                        } else {
                            PBS = positionBlueSkystone.RIGHT;
                            telemetry.addLine("Blue Right");
                        }
                        //Blue working great
                        /* Blue Left x=-18.7 y=-7.7 z=6.7 LEFT
                         * Blue Center x=-20.9 y=2 z=7.4 LEFT
                         * Blue Right x= y= z= Cannot see right*/
                        /*Red Right x=-17.5 y=-2.4 z=6.3 LEFT (lined up edge of robot, not edge of grabber)
                         * Red Center x=-21.4 y=-9.5 z=5.8 LEFT (lined up edge of robot, not edge of grabber)
                         * Red Left x= y= z= */
                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    } else {
                        telemetry.addData("Visible Target", "none");
                    }
                    telemetry.addData("Red Skystone Position", PRS);
                    telemetry.addData("Blue Skystone Position", PBS);
                    telemetry.update();
                    // Disable Tracking when we are done;
                    targetsSkyStone.deactivate();

                    telemetry.addLine("Initial");
                    telemetry.update();

                    telemetry.addData("Red Skystone Position", Webcam.PRS);
                    if (Webcam.PBS == Webcam.PBS.LEFT)
                        telemetry.addLine("Blue Skystone Left");
                    else if (Webcam.PBS == Webcam.PBS.CENTER)
                        telemetry.addLine("Blue Skystone Center");
                    else
                        telemetry.addLine("Blue Skystone Right");

                    if (Webcam.PRS == Webcam.PRS.RIGHT)
                        telemetry.addLine("Red Skystone Right");
                    else if (Webcam.PRS == Webcam.PRS.CENTER)
                        telemetry.addLine("Red Skystone Center");
                    else
                        telemetry.addLine("Red Skystone Left");
                    // express the rotation of the robot in degrees.
                    Drive.TimeDelay(10);


                    telemetry.addLine("Stop");
                    telemetry.update();
                    break;
            }
        }
    }



