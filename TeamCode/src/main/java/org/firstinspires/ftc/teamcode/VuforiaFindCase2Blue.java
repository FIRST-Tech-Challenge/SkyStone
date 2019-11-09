package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.hardware.DcMotor$RunMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This file illustrates the concept of driving a path based on imu heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C imu with the name "imu"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the imu correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the imu Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the imu.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//@Autonomous(name="AV4VA", group="MoveBot")
// @Disabled
public class VuforiaFindCase2Blue {
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AXGLTO3/////AAABmTK6sC55/UXIrPPDsV2B+BRsDCvfYQT4WRRRqXzke8Pb5l8bDG0wY/10aTdh0xTyIZhrb/JsLpRDhxG2PU9aNrc2D99uWBFFgYc3oO/O3y628047am08g44IwVCv3SmYYtc9x3fK/Rp+kMttG9UA2mWobqfelMbFaIub6aAjJjh+zzY+Em3Bw1hcQ+QlFSSiuG7YrAuQZ1tvFVBOPeHa9cFHCpVNeD7kXyfND69ZWw/ZS5zDyFdgAIaBAO9kpyvmpfGRQwQXJFCnGAQ9qql2ggKYY8EmPw+K6wM3PDYgpELOlhrHBvF5gAHtqkqqYgyv1EI3KD074u3SgdF1l3t6jT98tPvbisXEBbo6pDLl6B/o";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private LinearOpMode opmode;
    private float phoneZRotate = 0;
    private static final float mmPerInch = 25.4f;
    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    // Constants for the center support targets

    private ElapsedTime runtime;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    public int positionCase = 3;
    List<VuforiaTrackable> allTrackables;
    VuforiaTrackable stoneTarget;
    VuforiaTrackables targetsSkyStone;
    public VuforiaFindCase2Blue(LinearOpMode opmode){
        this.opmode = opmode;
        //this.hardwareMap = opmode.hardwareMap;
    }
    public int vuforia() {
    // Wait for the game to start (Display imu value), and reset imu before we move..
    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    // Put a hold after each turn
        boolean targetVisible = false;
        targetsSkyStone.activate();
        runtime = new ElapsedTime();
        while (!targetVisible && runtime.seconds() <= 1) {
            // check all the trackable targets to see which one (if any) is visible.
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    this.opmode.telemetry.addData("Visible Target", trackable.getName());
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
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                this.opmode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                this.opmode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                //this.opmode.telemetry.update();
                this.opmode.telemetry.addData("get0", translation.get(1) / mmPerInch);
                this.opmode.telemetry.update();
//                sleep(40000);
                // if loop to assign cases
                if ((translation.get(1) / mmPerInch <= -3)) { // this used to be -2
                    positionCase = 1;
                } else if ((translation.get(1) / mmPerInch) > 2) { // this used to be 3
                    positionCase = 3;
                } else {
                    positionCase = 2;
                }
//                this.opmode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//                this.opmode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//                this.opmode.telemetry.addData("Position Case", positionCase);
//                this.opmode.telemetry.update();


            } else {
                // When none of the skystone is visible, position case will be 4
                this.opmode.telemetry.addData("Visible Target", "none");
                this.opmode.telemetry.update();
                positionCase = 4;

            }
        }
        return positionCase;
    }
    public void initVuforia(){
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opmode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        this.targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("SkyStone Target");
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        this.allTrackables = new ArrayList<VuforiaTrackable>();
        this.allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }




        this.opmode.telemetry.addData(">", "Robot Ready.");    //
        this.opmode.telemetry.update();
    }
}