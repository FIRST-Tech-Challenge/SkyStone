/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.algorithms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * <p>
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class VuforiaAutoNav {

    /**
     * TODO: add automatic alliance detection
     */
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ATUUQRD/////AAABmWvkn/HpKkiTkmH+kqcdQa87+E5nnizSTMHex9sTxsSbub3m/AzfdamdYGP7pwr/6Ea3A5aHYC35fc9Nw8wFLofmMHwKHSwnm6wC/kS6oEspjXxlk7p3YKgHpe9iWIuvYVHDI211sVIxCg+wd8DvtdoFulhQ+dLLSajTNryZpsKgOJRHKnq4KREOb3jticHQpvTWDrM3O3yya3F5KEOBUr5ekhLxz06M7VpmIeuCc6FTw3RxRQ6qtqKfXxCzCK0ziyyDyMlBCie0WH1gvI1kKhk3modRIaJfaTcAw54REWyTfIhhV3A4Nyp/99j1FYonm94fu/gvOemiGDI1WWotAOSYqxnLmru7vN7kSzlsKits";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target (currently unused)
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    //TODO: check if the phone has a z rotation, and, if it does, update this value
    private ArrayList<Stone> Stones = new ArrayList<>();
    private HardwareMap hardwareMap;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private int[] skyStonePositions = {-1, -1};


    /*
     * For coordinate system: origin is on floor at center of field
     * x axis is perp. to y axis; building zone is positive and loading zone is negative
     * y axis runs through bridges; blue side is positive and red side is negative
     * z axis goes through center of field; up is positive
     * no clue yet which direction is zero degrees and which direction is positive degrees
     *
     *
     */

    private float robotX = 0;
    private float robotY = 0;
    private float robotZ = 0;
    private float robotAngle = 0;
    private int alliance = 0;

    /**
     * gets the alliance
     *
     * @return alliance (1 is blue, -1 is red)
     */
    public int getAlliance() {
        return alliance;
    }

    /**
     * sets the alliance
     *
     * @param alliance Indicates alliance (1 is blue, -1 is red)
     */
    public void setAlliance(int alliance) {
        this.alliance = alliance;
    }

    /**
     * gets the hardware map
     *
     * @return hardware map
     */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    /**
     * sets the hardware map
     *
     * @param hardwareMap hardware map
     */
    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * list of vuforia trackables
     *
     * @return vuforia trackables
     */
    public List<VuforiaTrackable> getAllTrackables() {
        return allTrackables;
    }

    /**
     * sets vuforia trackables
     *
     * @param allTrackables vuforia trackables
     */
    public void setAllTrackables(List<VuforiaTrackable> allTrackables) {
        this.allTrackables = allTrackables;
    }

    public float getRobotX() {
        return robotX;
    }

    public void setRobotX(float robotX) {
        this.robotX = robotX;
    }

    public float getRobotY() {
        return robotY;
    }

    public void setRobotY(float robotY) {
        this.robotY = robotY;
    }

    public float getRobotZ() {
        return robotZ;
    }

    public void setRobotZ(float robotZ) {
        this.robotZ = robotZ;
    }

    public void setRobotPosition(float robotX, float robotY, float robotZ) {
        this.setRobotX(robotX);
        this.setRobotY(robotY);
        this.setRobotZ(robotZ);
    }

    public float getRobotAngle() {
        return robotAngle;
    }

    public void setRobotAngle(float robotAngle) {
        this.robotAngle = robotAngle;
    }

    public ArrayList<Stone> getStones() {
        return Stones;
    }

    public void setStones(ArrayList<Stone> stones) {
        Stones = stones;
    }

    public Stone getStone(int index) {
        return Stones.get(index);
    }

    public void setStone(int index, Stone stone) {
        Stones.set(index, stone);
    }

    public void addStone(int kind, float top, float bottom, float left, float right) {
        Stone stone = new Stone(kind, top, bottom, left, right);
        this.getStones().add(stone);
    }

    public void addStone(float top, float bottom, float left, float right) {
        Stone stone = new Stone(top, bottom, left, right);
        this.getStones().add(stone);
    }

    public void addStone(Stone stone) {
        this.getStones().add(stone);
    }

    public void clearStones() {
        this.getStones().clear();
    }

    public int[] getSkyStonePositions() {
        return skyStonePositions;
    }

    public void setSkyStonePositions(int[] skyStonePositions) {
        this.skyStonePositions = skyStonePositions;
    }

    /**
     * Gets the relative position of one of the sky stones
     *
     * @param index the stone to return (0 for left one, 1 for right one)
     * @return the sky stone position (they are indexed from 0 to 5 from left to right when looking away from the field wall)
     */
    public int getSkyStonePosition(int index) {
        return skyStonePositions[index];
    }

    public void setSkyStonePosition(int index, int skyStonePosition) {
        this.skyStonePositions[index] = skyStonePosition;
    }

    /**
     * gets the X coordinate of the center of a stone
     *
     * @param stoneNum Indicates stone number (they are indexed from 0 to 5 from left to right when looking away from the field wall)
     * @param alliance Indicates alliance (1 is blue, -1 is red)
     * @return stone center x
     */
    public float getStoneCenterX(int stoneNum, int alliance) {
        if (stoneNum < 0 || stoneNum > 5) {
            return 0;
        } else {
            if (alliance == 1) {
                return 28 + 8 * stoneNum;
            } else if (alliance == -1) {
                return 28 + 8 * (5 - stoneNum);
            } else {
                return 0;
            }
        }
    }

    /**
     * gets the Y coordinate of the center of a stone
     *
     * @param stoneNum Indicates stone number (they are indexed from 0 to 5 from left to right when looking away from the field wall)
     * @param alliance Indicates alliance (1 is blue, -1 is red)
     * @return stone center y
     */
    public float getStoneCenterY(int stoneNum, int alliance) {
        if (alliance == 1) {
            return 22;
        } else if (alliance == -1) {
            return -22;
        } else {
            return 0;
        }
    }

    /**
     * gets the X coordinate of the center of a sky stone stone
     *
     * @param index    Indicates sky stone number
     * @param alliance Indicates alliance (1 is blue, -1 is red)
     * @return stone center x
     */
    public float getSkyStoneCenterX(int index, int alliance) {
        int stoneNum = this.getSkyStonePosition(index);
        if (stoneNum < 0 || stoneNum > 5) {
            return 0;
        } else {
            if (alliance == 1) {
                return 28 + 8 * stoneNum;
            } else if (alliance == -1) {
                return 28 + 8 * (5 - stoneNum);
            } else {
                return 0;
            }
        }
    }

    /**
     * gets the Y coordinate of the center of a sky stone
     *
     * @param index    Indicates sky stone number
     * @param alliance Indicates alliance (1 is blue, -1 is red)
     * @return stone center y
     */
    public float getSkyStoneCenterY(int index, int alliance) {
        if (alliance == 1) {
            return 22;
        } else if (alliance == -1) {
            return -22;
        } else {
            return 0;
        }
    }

    /**
     * gets the X coordinate of the center of a stone
     *
     * @param stoneNum Indicates stone number (they are indexed from 0 to 5 from left to right when looking away from the field wall)
     * @return stone center x
     */
    public float getStoneCenterX(int stoneNum) {
        return this.getStoneCenterX(stoneNum, this.getAlliance());
    }

    /**
     * gets the Y coordinate of the center of a stone
     *
     * @param stoneNum Indicates stone number (they are indexed from 0 to 5 from left to right when looking away from the field wall)
     * @return stone center y
     */
    public float getStoneCenterY(int stoneNum) {
        return this.getStoneCenterY(stoneNum, this.getAlliance());
    }

    /**
     * gets the X coordinate of the center of a sky stone stone
     *
     * @param index Indicates sky stone number
     * @return stone center x
     */
    public float getSkyStoneCenterX(int index) {
        return this.getSkyStoneCenterX(index, this.getAlliance());
    }

    /**
     * gets the Y coordinate of the center of a sky stone
     *
     * @param index Indicates sky stone number
     * @return stone center y
     */
    public float getSkyStoneCenterY(int index) {
        return this.getSkyStoneCenterX(index, this.getAlliance());
    }

    /**
     * Because this class is not an opmode, the opmode using it needs to feed in its hardware map.
     *
     * @param hardwareMap
     */
    public VuforiaAutoNav(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * This method initializes the webcam and navigation system and should always be used first.
     */
    public void initView() {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        // skystones have a variable position, so we can't use them.

//        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
//        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
//        stoneTarget.setLocation(OpenGLMatrix
//                .translation(0, 0, stoneZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        // TODO: update with actual camera placement
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

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        // Disable Tracking when we are done;
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

    }

    /**
     * This method updates the robot's position, angle, and field of view.
     */
    public void updateView() {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
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
            this.setRobotPosition(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            this.setRobotAngle(rotation.thirdAngle);
        }
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                ArrayList<Stone> stoneView = new ArrayList<>();
                // step through the list of recognitions and display boundary info.
                this.clearStones();
                for (Recognition recognition : updatedRecognitions) {
                    int kind = 0;
                    if (recognition.getLabel().equals("Skystone")) {
                        kind = 1;
                    }
                    Stone newStone = new Stone(kind, recognition.getTop(), recognition.getBottom(), recognition.getLeft(), recognition.getRight());
                    stoneView.add(newStone);
                }
                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    float currentPosition = stoneView.get(0).getCenterX();
                    int currentStone = 0;
                    for (int j = 1; j < updatedRecognitions.size() - i - 1; j++) {
                        if (stoneView.get(j).getCenterX() < currentPosition) {
                            currentPosition = stoneView.get(j).getCenterX();
                            currentStone = j;
                        }
                    }
                    this.addStone(stoneView.get(currentStone));
                    stoneView.remove(currentStone);
                }

                if (updatedRecognitions.size() == 6 && (this.getSkyStonePosition(0) == -1 || this.getSkyStonePosition(1) == -1)) {
                    for (int i = 0; i < 6; i++) {
                        if (this.getStone(i).getKind() == 1) {
                            if (this.getSkyStonePosition(0) == -1) {
                                this.setSkyStonePosition(0, i);
                            } else {
                                this.setSkyStonePosition(1, i);
                            }
                        }
                    }
                }
            }

        }
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


}
