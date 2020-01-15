package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Autonomous Mode Usecase 4<BR>
 *
 * Description : Start on wall in Loading Zone, identify and move 1 skystone to Building zone and<BR>
 *                  place stone on Foundation. Pull foundation to edge of Building zone<BR>
 *                  park near wall or near neutral Skybridge<BR>
 *
 * Steps:<BR>
 * <ol>
 * <li>Robot starts on SB5
 * <li>On start, robot opens wrist to front position
 * <li>Lift Arm to Sense Position using Vuforia
 * <li>If Skystone, record Skystone position as SB5, Go to Step 10
 * <emsp><emsp>Else move robot to SB4. Check on Vuforia for Skystone.
 * <emsp>If Skystone, record Skystone position as SB4, Go to Step 10
 * <emsp><emsp>Else move robot to SB3. record skystone position as SB3.
 * <li>Grip and pick the block.
 * <li>Lift Arm to Level 2 Tray Height
 * <li>Slide back to edge of B2,
 * <li>Turn 90 degrees Left
 * <li>Move to B4
 * <li>Turn right, move forward and drop skystnne on foundation
 * <li>Slide 10" right and drop arm to hold foundation
 * <li>Pull foundation to edge of Loading zone
 * <li>Move right till outside of foundation
 * <li>(Alternate usecase) : Move forward to align to parking location near neatural sybridge, else stay
 * to park along the wall.
 * <li>Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
 * </ol>
 * Uses playingAlliance variable to select as 1 for Blue, -1 for Red Alliance<BR>
 * Uses parkingPlaceNearSkyBridge variable false for near wall, true for near NeutralSkybridge<BR>
 */

public class AutoUC4_Skystone_MoveFdn_Park {

    int skystonePosition;

    boolean parkedStatus = false; // Will be true once robot is parked

    ElapsedTime AutonomousTimeOut = new ElapsedTime();

    //*************Vuforia******************
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            "AZME4Mr/////AAABmY+MAyxxT0IileR7JBqaAPsxN2XNYlaGBtEjYaHOlVVTqPQf7NH9eIrosYKKQHPGEXLtJUsdMwZ9e3EXBfy6arulcLPvdpW9bqAB2F2MJJXo35lLA096l/t/LQTi+etVso0Xc5RYkTVSIP3YABp1TeOaF8lCSpjVhPIVW3l/c/XlrnEMPhJk9IgqMEp4P/ifqAqMMMUAIKPEqIrXIv79TvAfdIJig46gfQGaQl5tFHr3nmvMbh/LhFrh5AWAy3B/93cCkOszmYkdHxZStbNB5lMdkTnf3sCnYbQY4jviorfhYrAkqHWH6vNOB9lUt8dOSeHsDtlk33e/6xQgOCNYFN80anYMp82JNDBFX3oyGliV";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private double x_translate = 0;
    private double y_translate = 0;
    double stoneTostone = 8;

    VuforiaTrackables targetsSkyStone;
    VuforiaTrackable stoneTarget;
    //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    int cameraMonitorViewId;

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT  =  1.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    //*************Vuforia******************


    public boolean AutoUC4_Skystone_MoveFdn_Park_Method(
            LinearOpMode callingOpMode,
            int playingAlliance,
            boolean parkingPlaceNearSkyBridge,
            Chassis autoUCChassis,
            Arm autoUCArm,
            Intake autoUCIntake
    ) {

        // Robot starts on SB5

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        //Lift Arm to AboveFoundation level
        autoUCArm.moveArm_aboveFoundationLevel();
        callingOpMode.telemetry.update();

        // Move by distance X forward near SB5 : 6 inches to skystone
        double robotToNearSkystone = 24; // was 27
        autoUCChassis.runFwdBackLeftRight(robotToNearSkystone,0,0.25, callingOpMode);

        callingOpMode.sleep(200);

        callingOpMode.telemetry.addData("before viewforia in loop","");
        callingOpMode.telemetry.update();

        skystonePosition = 5;
        vuforiaFindSkystone(callingOpMode);

        callingOpMode.telemetry.addData("after viewforia: Target visible: ",targetVisible);
        callingOpMode.telemetry.update();
        /*
        if (targetVisible){
            /if(Math.abs(x_translate)>1){
                runFwdBackLeftRight(Math.abs(x_translate), (int) x_translate/Math.abs(x_translate),0.1);
            }
        }
        sleep(500);*/
        /*if (targetVisible){
            runFwdBackLeftRight(y_translate, 0,0.1);
        }*/

        //If target is not visible move to next block
        if(!targetVisible) {
            autoUCChassis.runFwdBackLeftRight(stoneTostone, playingAlliance, 0.1, callingOpMode);
            skystonePosition = 4;
        }

        if(!targetVisible && (skystonePosition == 4)){
                vuforiaFindSkystone(callingOpMode);
        }

        /*if (targetVisible && (skystonePosition == 4)){
            runFwdBackLeftRight(Math.abs(x_translate), x_translate/Math.abs(x_translate),0.1);
        }*/
        /*sleep(500);
        if (targetVisible){
            runFwdBackLeftRight(y_translate, 0,0.1);
        }*/

        //If target is not visible move to next block
        if(!targetVisible && (skystonePosition == 4)) {
            autoUCChassis.runFwdBackLeftRight(stoneTostone, playingAlliance, 0.1, callingOpMode);
            skystonePosition = 3;
        }

        // Drop Arm
        autoUCArm.moveArm_groundLevel();
        autoUCArm.turnArmBrakeModeOn();
        callingOpMode.sleep(250);

        // Move forward 5 inches
        autoUCChassis.runFwdBackLeftRight(5, 0, 0.1, callingOpMode);
        callingOpMode.sleep(250);

        //Grip the block
        autoUCIntake.closeGrip();
        callingOpMode.sleep(250);

        // Slide back to edge of B2, 10 inches
        autoUCChassis.runFwdBackLeftRight(-9,0,0.1, callingOpMode); // distance was 8

        callingOpMode.sleep(200);
        // Turn 90 degrees Left
        autoUCChassis.turnby90degree(playingAlliance*(-1),0.2, callingOpMode); // was 0.1
        callingOpMode.sleep(250);


       //Move forward till Chassis bumber limit switch is pressed.
        double expectedMaxDistanceToFoundation = 86 + (5 - skystonePosition) * stoneTostone; // was 40 --> 70
        autoUCChassis.runFwdBackLeftRight(expectedMaxDistanceToFoundation, 0, 0.5, callingOpMode) ;

        //Lift Arm
        autoUCArm.moveArm_aboveFoundationLevel();

        // Turn 90 degrees right
        autoUCChassis.turnby90degree(playingAlliance,0.1, callingOpMode); // was 0.1
        callingOpMode.sleep(250);

        //Move forward till Chassis bumber limit switch is pressed
        autoUCChassis.runFwdTill_frontleftChassisTouchSensor_Pressed(18, 0.25, callingOpMode);

        // Drop block
        autoUCIntake.openGrip();
        callingOpMode.sleep(250);

        //Move right 10 inches to hook the base outside of the stone
        //autoUCChassis.runFwdBackLeftRight(5, playingAlliance*(-1), 0.2, callingOpMode);

        // Drop Arm
        //autoUCChassis.runFwdBackLeftRight(2,0,0.2, callingOpMode);
        autoUCArm.moveArm_onFoundationLevel();

        // Hook Foundation
        autoUCChassis.moveHook_holdFoundation();

        // Strafe Left
        autoUCChassis.runFwdBackLeftRight(2,-playingAlliance,0.2, callingOpMode);
        callingOpMode.sleep(250);
        autoUCArm.moveArm_onFoundationLevel();

        // Move Back
        autoUCChassis.runFwdBackLeftRight(-40,0,0.2, callingOpMode);

        //Lift Arm to Above foundation level and release hook
        autoUCArm.moveArm_aboveFoundationLevel();
        //autoUCArm.moveArm_detectSkystoneLevel();
        autoUCChassis.moveHook_Released();
        callingOpMode.sleep(250);

        //Push forward to move foundation to end of line
        /*double foundationtoEdgeofBuildingSite = 1;
        autoUCChassis.runFwdBackLeftRight(foundationtoEdgeofBuildingSite,0,0.1, callingOpMode);
        callingOpMode.sleep(100);
        */
        //Move back till wall is hit
        //autoUCChassis.runFwdBackLeftRight(-4,0,0.25, callingOpMode);
        //callingOpMode.sleep(100);

        //Move out of foundation area
        autoUCChassis.runFwdBackLeftRight(35, playingAlliance, 0.35, callingOpMode);

        //Move Arm to ground Level
        autoUCArm.turnArmBrakeModeOn();
        callingOpMode.sleep(200);


        //Optional : Move to park near skybridge Neutral
        if (parkingPlaceNearSkyBridge){
            autoUCChassis.runFwdBackLeftRight(25,0,0.35, callingOpMode);
        }

        //Turn by 90 degrees to point arm forward
        autoUCChassis.turnby90degree(-playingAlliance,0.2, callingOpMode);
        callingOpMode.sleep(200);

        //Park near wall
        //Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
        if (playingAlliance == 1) {
            //Blue Alliance
            autoUCChassis.runTill_ChassisRightColorSensorIsBlue(-15, 0, 0.2, callingOpMode);
        } else {
            //Red Alliance
            autoUCChassis.runTill_ChassisRightColorSensorIsRed(-15, 0, 0.2, callingOpMode);
        }

        //Reached Parking position
        return parkedStatus = true;

        //End of Usecase : Should be parked at this time.
    }

    /**
     * Method to Initiatlize Vuforia
     * @param hardwareMap to pass the phone camera
     */
    public void vuforiaInit(HardwareMap hardwareMap){
        //*************Vuforia******************
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        /* For convenience, gather together all the trackable objects in one easily-iterable collection
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        */

        VectorF translation;

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /*  Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        */
        //*************Vuforia************************

    }

    /**
     * Method to activate Vuforia and identify skystone
     * @param callingOpMode to use for telemetry updates
     */
    void vuforiaFindSkystone(LinearOpMode callingOpMode){
        AutonomousTimeOut.reset();
        AutonomousTimeOut.startTime();
        targetVisible = false;

        targetsSkyStone.activate();
        //Turn Camera flash on
        CameraDevice.getInstance().setFlashTorchMode(true);

        while(AutonomousTimeOut.milliseconds()<500 && !targetVisible) {
            //**************Vuforia********************
            // check all the trackable targets to see which one (if any) is visible.
            /*/TESTCOMMENT for (VuforiaTrackable trackable : allTrackables) {
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
                }*/
            //TESTCOMMENT ADD START
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                callingOpMode.telemetry.addData("Visible Target", stoneTarget.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            //TESTCOMMENT ADD END

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                callingOpMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                callingOpMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                x_translate = translation.get(0)/mmPerInch;
                y_translate = translation.get(1) / mmPerInch;

            } /*else {
                callingOpMode.telemetry.addData("Visible Target", "none");
            }*/
            callingOpMode.telemetry.update();

        }
        //Turn Camera flash on
        CameraDevice.getInstance().setFlashTorchMode(false);

        targetsSkyStone.deactivate();
        //***********Vuforia**************

    }
}
