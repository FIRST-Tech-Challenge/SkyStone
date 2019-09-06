package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by Ethan on 10/23/2016.
 */
public abstract class OmniAutoClass extends LinearOpMode {

    // ========== VUFORIA TRACKING VARIABLES  ==========
    VuforiaTrackables           targetsRoverRuckus = null;
    // Navigation data is only valid if targetFound == true;
    private static final int    SAMPLE_RIGHTSIDE = 1;        // Assess sampling field from the RIGHT side (CENTER & RIGHT visible)
    private static final int    SAMPLE_LEFTSIDE  = 2;        // Assess sampling field from the LEFT side  (LEFT & CENTER visible)
    public static final int    POSITION_UNKNOWN = -1;
    public static final int    POSITION_LEFT    = 1;
    public static final int    POSITION_CENTER  = 2;
    public static final int    POSITION_RIGHT   = 3;
    private boolean teamNumber1   = false;  // default to Postion #2 (RIGHT side, facing DEPOT)
    private ElapsedTime timer;


    private VuforiaLocalizer vuforia;
    // ========== TENSORFLOW VARIABLES  ==========
    private TFObjectDetector tfod = null;
    final String LABEL_GOLD_MINERAL = "Gold Mineral";
    final String LABEL_SILVER_MINERAL = "Silver Mineral";
    final int    PHONE_DISPLAY_MIDPOINT = 360;   // assumes rotated 1280x720 resolution (=720/2)
    // variables related to sampling left two or right two positions:
    private char  sampleLeftLefType,  sampleRightLefType;  // Type: G=Gold, S=Silver, ?=Unknown
    private char  sampleLeftCenType,  sampleRightCenType;
    private char  sampleLeftRigType,  sampleRightRigType;
    private int   sampleLeftCenX,     sampleRightCenX;     // X pixel value
    private int   sampleLeftLefX,     sampleRightRigX;
    private int   sampleLeftCenY,     sampleRightCenY;     // Y pixel value
    private int   sampleLeftLefY,     sampleRightRigY;
    private float sampleLeftCenC,     sampleRightCenC;     // Confidence levels
    private float sampleLeftLefC,     sampleRightRigC;
    // variables related to LEFT/CENTER/RIGHT position assignment:
    private char  mineralLefType, mineralCenType, mineralRigType; // Type
    private int   mineralLefX,    mineralCenX,    mineralRigX;    // X pixel
    private int   mineralLefY,    mineralCenY,    mineralRigY;    // Y pixel
    private float mineralLefC,    mineralCenC,    mineralRigC;    // Confidence

    private List<Recognition> leftDetections = new ArrayList<Recognition>();
    private List<Recognition> centerDetections = new ArrayList<Recognition>();
    private List<Recognition> rightDetections = new ArrayList<Recognition>();

    public static float mmPerInch = OmniAutoClass.MM_PER_INCH;
    public static float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    public static float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


    HardwareOmnibot robot = new HardwareOmnibot();

    // Default to 4" wheels
    private static double myWheelSize = 4.0;
    // Default to 40:1 motors
    private static double myMotorRatio = 40.0;

    // 20:1 motor = 560
    // 40:1 motor = 1120
    // 60:1 motor = 1680
    private static final double encoderClicksPerRev = 28;
    private static double clicksPerInch = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize);

    public static final float MM_PER_INCH = 25.4f;

    /**
     *
     */
    public void deployArm() {
        robot.setExtenderMotorPower(0.0, true);
        robot.setRotatorMotorPower(0.0);
        robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extender.setTargetPosition(1100);
        robot.rotator1.setTargetPosition(-3000);
        robot.rotator2.setTargetPosition(-3000);
        robot.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rotator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setExtenderMotorPower(1.0, true);
        robot.setRotatorMotorPower(1.0);
    }

    /**
     * @param newWheelSize  - The size of the wheels, used to calculate encoder clicks per inch
     * @param newMotorRatio - The motor gearbox ratio, used to calculate encoder clicks per inch
     */
    public void setupRobotParameters(double newWheelSize, double newMotorRatio) {
        robot.init(hardwareMap);
        timer = new ElapsedTime();

        robot.resetDriveEncoders();
        robot.setInputShaping(false);
        myWheelSize = newWheelSize;
        myMotorRatio = newMotorRatio;

        clicksPerInch = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize);

        // Initialize Vuforia
        initVuforia();
        // Activate Vuforia (this takes a few seconds)
        activateTracking();
        // Initialize TensorFlow
        // NOTE: TensorFlow uses the camera frames from VuforiaLocalizer, so vuforia must be created first.
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Activate Tensor Flow Object Detection
        if (tfod != null) {
            tfod.activate();
        }
    }

    /**
     * @param position    - The current encoder position
     * @param destination - The desired encoder position
     * @param speed       - The speed of travel used to get direction
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean reachedClickPosition(int position, int destination, double speed, boolean reverseEncoders) {
        boolean result = false;

        if (reverseEncoders) {
            if (speed < 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        } else {
            if (speed > 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        }
        return result;
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle) {
        double xPower = 0.0;
        double yPower = 0.0;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 1;
        double gyroReading = robot.readIMU();
        deltaAngle = deltaAngle(headingAngle, gyroReading);

        if (Math.abs(deltaAngle) > SAME_ANGLE) {
            if (deltaAngle > 0.0) {
                rotateSpeed = -rotateSpeed;
            }
        } else {
            rotateSpeed = 0.0;
        }

        xPower = speed * Math.cos(Math.toRadians(driveAngle));
        yPower = speed * Math.sin(Math.toRadians(driveAngle));
        robot.drive(xPower, yPower, rotateSpeed, -90.0);
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeadingForTime(double speed, double rotateSpeed, double driveAngle, double headingAngle, int driveTime) {
        int sleepTime = 0;
        final int DELTA_SLEEP = 20;

        while (!isStopRequested() && sleepTime < driveTime) {
            driveAtHeading(speed, rotateSpeed, driveAngle, headingAngle);
            sleep(DELTA_SLEEP);
            sleepTime += DELTA_SLEEP;
        }
        robot.setAllDriveZero();
    }

    /**
     * @param speed        - The maximum speed for the robot
     * @param rotateSpeed  - The maximum rotational speed for the robot
     * @param driveAngle   - The angle to drive at
     * @param headingAngle - The angle to maintain heading for
     * @param distance     - The distance to travel
     * @param maxTime      - The time to wait before giving up
     */
    private void driveDistanceAtAngleOnHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle, double distance, int maxTime, boolean reverseEncoders) {
        //temporary fix until we understand
        double correctedDistance = distance / 1.75;

        int sleepTime = 0;
        final int deltaSleep = 50;
        int position = robot.leftMotorFore.getCurrentPosition();
        int finalEncoderValue;
        double gyroReading = robot.readIMU();

        if (reverseEncoders) {
            if (speed < 0) {
                finalEncoderValue = position + (int) (correctedDistance * clicksPerInch);
            } else {
                finalEncoderValue = position - (int) (correctedDistance * clicksPerInch);
            }
        } else {
            if (speed < 0) {
                finalEncoderValue = position - (int) (correctedDistance * clicksPerInch);
            } else {
                finalEncoderValue = position + (int) (correctedDistance * clicksPerInch);
            }
        }

        while ((!reachedClickPosition(position, finalEncoderValue, speed, reverseEncoders) && (sleepTime < maxTime))) {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + finalEncoderValue;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Setting Power: ", speed);
            telemetry.addData("Sleep Time: ", sleepTime);

            // Since this is a driveForward function, the Y Axis is the only important axis
            driveAtHeading(speed, rotateSpeed, driveAngle, gyroReading);
            updateTelemetry(telemetry);

            sleep(deltaSleep);
            sleepTime += deltaSleep;
            position = robot.leftMotorFore.getCurrentPosition();
            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        robot.setAllDriveZero();
    }

    /**
     * @param speed    - Max speed to run robot
     * @param distance - Desired distance to travel in inches
     * @param maxTime  - The time allowed before exiting without completing
     */
    public void driveDistanceForwardOnHeading(double speed, double distance, int maxTime, boolean reverseEncoders) {
        driveDistanceAtAngleOnHeading(speed, 0.1, 90.0, robot.readIMU(), distance, maxTime, reverseEncoders);
    }

    /**
     * @param speed    - Max speed to run robot
     * @param distance - Desired distance to travel in inches
     * @param maxTime  - The time allowed before exiting without completing
     */
    public void driveDistanceSidewaysOnHeading(double speed, double distance, int maxTime, boolean reverseEncoders) {
        driveDistanceAtAngleOnHeading(speed, 0.1, 0.0, robot.readIMU(), distance, maxTime, reverseEncoders);
        robot.setAllDriveZero();
    }

    /**
     * @param destinationAngle - The target angle to reach, between 0.0 and 360.0
     * @param gyroReading      - The current angle of the robot
     * @return The minumum angle to travel to get to the destination angle
     */
    private double deltaAngle(double destinationAngle, double gyroReading) {
        double result = 0.0;
        double leftResult = 0.0;
        double rightResult = 0.0;

        if (gyroReading > destinationAngle) {
            leftResult = gyroReading - destinationAngle;
            rightResult = 360.0 - gyroReading + destinationAngle;
        } else {
            leftResult = gyroReading + 360.0 - destinationAngle;
            rightResult = destinationAngle - gyroReading;
        }

        if (leftResult < rightResult) {
            result = -leftResult;
        } else {
            result = rightResult;
        }

        return result;
    }

    /**
     * @param speed       - The maximum speed to rotate the robot
     * @param targetAngle - The 0-360 degree angle to rotate the robot to
     * @param maxTime     - The time to allow before quiting
     */
    public void rotateRobotToAngle(double speed, double targetAngle, int maxTime) {
        double gyroReading = robot.readIMU();
        int sleepTime = 0;
        final int deltaSleep = 50;
        double angleRemaining = 0.0;
        final double SAME_ANGLE = 1.0;
        double rotateSpeed = 0.0;

        angleRemaining = deltaAngle(targetAngle, gyroReading);
        while ((Math.abs(angleRemaining) > SAME_ANGLE && (sleepTime < maxTime) && (!isStopRequested()))) {
            telemetry.addData("Current Angle: ", gyroReading);
            telemetry.addData("Destination Angle: ", targetAngle);
            telemetry.addData("Sleep Time: ", sleepTime);
            telemetry.addData("Delta Angle: ", angleRemaining);

            rotateSpeed = controlledRotationAngle(angleRemaining, speed);
            if (angleRemaining > 0.0) {
                // Positive angle, need to rotate right
                rotateSpeed = -rotateSpeed;
            }
            telemetry.addData("Rotate Speed: ", rotateSpeed);
            robot.drive(0.0, 0.0, rotateSpeed, 0.0);
            updateTelemetry(telemetry);

            sleep(deltaSleep);
            sleepTime += deltaSleep;
            gyroReading = robot.readIMU();
            angleRemaining = deltaAngle(targetAngle, gyroReading);
        }

        robot.setAllDriveZero();
    }

    /**
     * @param speed   - Max speed to rotate
     * @param angle   - Angle in degrees to rotate.  Positive right, negative left
     * @param maxTime - Timeout to quit trying
     */
    public void rotateRobot(double speed, double angle, int maxTime) {
        double gyroReading = robot.readIMU();
        double targetAngle = gyroReading + angle;

        // We won't do circles with this function, just minimum rotation.
        // Get the destination gyro angle
        while (targetAngle > 360.0) {
            targetAngle -= 360.0;
        }
        while (targetAngle < 0.0) {
            targetAngle += 360.0;
        }

        rotateRobotToAngle(speed, targetAngle, maxTime);
    }

    /**
     *
     */
    public void rotateRobotForTime(double speed, int maxTime) {
        int elaspedTime = 0;
        while(opModeIsActive() && elaspedTime < maxTime) {
            robot.drive(0.0,0.0,speed,0.0);
            elaspedTime += 50;
            sleep(50);
        }
        robot.setAllDriveZero();
    }

    /**
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed           - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledDeceleration(double distanceToTravelMm, double maxSpeed) {
        final double superFastDistance = 400.0;
        final double fastDistance = 200.0;
        final double mediumDistance = 100.0;
        final double fastDivider = 1.4;
        final double mediumDivider = 2.0;
        final double slowDivider = 3.0;

        double result = 0.0;

        if (distanceToTravelMm > superFastDistance) {
            result = maxSpeed;
        } else if (distanceToTravelMm > fastDistance) {
            result = maxSpeed / fastDivider;
        } else if (distanceToTravelMm > mediumDistance) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed           - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationMm(double distanceToTravelMm, double maxSpeed) {
        final double fastDistance = 75.0;
        final double mediumDistance = 50.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 4.0;

        double result = 0.0;

        if (distanceToTravelMm > fastDistance) {
            result = maxSpeed;
        } else if (distanceToTravelMm > mediumDistance) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     * @param angleToTravel - How far we are traveling in degrees
     * @param maxSpeed      - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationAngle(double angleToTravel, double maxSpeed) {
        final double fastAngle = 30.0;
        final double mediumAngle = 15.0;
        final double mediumDivider = 1.5;
        final double slowDivider = 3.0;
        double angleToTravelAbs = Math.abs(angleToTravel);

        double result = 0.0;

        if (angleToTravelAbs > fastAngle) {
            result = maxSpeed;
        } else if (angleToTravelAbs > mediumAngle) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }


    // The value of speed should be signed such that it drives AWAY from the wall.
    // The function will reverse the sign to drrive TOWARDS the wall if it determines
    // it needs to.
    public void driveDistanceFromWall(double speed, double distance, int maxTime, boolean reverseEncoders, boolean forwards, boolean blue) {
        int elaspedTime = 0;
//        double rangeValue = robot.readRangeSensor();
        double rangeValue = 0;
        double delta =  distance - rangeValue;
        double addedAngle = 0;
        double driveSign;

        // This will reverse the speed if we are going towards the wall
        if(speed > 0) {
            driveSign = 1.0;
        }
        else {
            driveSign = -1.0;
        }

        if (forwards) {
            addedAngle = 90;
        }

        while(Math.abs(delta) > 1 && elaspedTime < maxTime && opModeIsActive()) {
            if(Math.abs(delta) < 10) {
                if(delta < 0) {
                    // Drive towards the wall
                    speed = driveSign * 0.05 * -1.0;
                }
                else {
                    speed = driveSign * 0.05;
                }
            }
            else if(Math.abs(delta) < 35) {
                if(delta < 0) {
                    // Drive towards the wall
                    speed = driveSign * 0.1 * -1.0;
                }
                else {
                    speed = driveSign * 0.1;
                }
            }
            driveAtHeading(speed, 0, 180 + addedAngle, 0);
            sleep(50);
            elaspedTime += 50;

//            rangeValue = robot.readRangeSensor();
            rangeValue = 0.0;
            delta =  distance - rangeValue;
        }

        robot.setAllDriveZero();
    }

    public void endAuto() {
        // Shut down Tensor Flow Object Detection
        if( tfod != null ) {
            tfod.shutdown();
        }
        while (!isStopRequested()) {
            sleep(100);
        }
    }

    public void Land() {
        timer.reset();
        robot.startLifterUp();
        while((robot.lifter.isBusy()) && (timer.milliseconds() < 5000.0) && (!isStopRequested()));
        robot.stopLifter();

        // Drive away from hook
        driveAtHeadingForTime(0.2, 0.05, 0, 0, 150);
        driveAtHeadingForTime(0.2, 0.05, -90, 0, 300);
        driveAtHeadingForTime(0.2, 0.05, 0, 0, 100);

        robot.startLifterDown();
    }

    // Input: samplePosition 1 = left
    //                       2 = center - currently unused
    //                       3 = right
    // Output: -1 = unknown
    //         1 = left
    //         2 = center
    //         3 = right
    public int sampleElements(int samplePosition) {
        int detectedPosition = POSITION_UNKNOWN;
        String leftElement = "";
        double leftConfidence = 0.0;
        String centerElement = "";
        double centerConfidence = 0.0;
        String rightElement = "";
        double rightConfidence = 0.0;
        Recognition leftDetectedElement = null;
        Recognition centerDetectedElement = null;

        // Reset our detection results
        leftDetections.clear();
        centerDetections.clear();
        rightDetections.clear();

        // Abort if Tensor Flow Object Detection initialization failed
        if( tfod != null ) {
            // getUpdatedRecognitions() will return null if no new information is available yet
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // Filter results to those in the two windows where the elements should be.
                windowDetectedResults(samplePosition, updatedRecognitions);
                // Narrow down the detected lists to the highest confidence result.
                if(samplePosition == POSITION_LEFT) {
                    for(Recognition element : leftDetections) {
                        if(element.getConfidence() > leftConfidence) {
                            leftDetectedElement = element;
                            leftElement = element.getLabel();
                            leftConfidence = element.getConfidence();
                        }
                    }
                    for(Recognition element : centerDetections) {
                        if(element.getConfidence() > centerConfidence) {
                            centerDetectedElement = element;
                            centerElement = element.getLabel();
                            centerConfidence = element.getConfidence();
                        }
                    }
                    if(leftElement.equals(LABEL_GOLD_MINERAL)) {
                        if(centerElement.equals(LABEL_GOLD_MINERAL)) {
                            if(leftDetectedElement.getConfidence() > centerDetectedElement.getConfidence()) {
                                detectedPosition = POSITION_LEFT;
                            } else {
                                detectedPosition = POSITION_CENTER;
                            }
                        } else {
                            detectedPosition = POSITION_LEFT;
                        }
                    } else if(centerElement.equals(LABEL_GOLD_MINERAL)) {
                        detectedPosition = POSITION_CENTER;
                    } else if(leftElement.equals(LABEL_SILVER_MINERAL) && centerElement.equals(LABEL_SILVER_MINERAL)){
                        detectedPosition = POSITION_RIGHT;
                    } else {
                        detectedPosition = POSITION_UNKNOWN;
                    }
                } else if( samplePosition == POSITION_RIGHT) {
                    for(Recognition element : centerDetections) {
                        if(element.getConfidence() > centerConfidence) {
                            centerElement = element.getLabel();
                            centerConfidence = element.getConfidence();
                        }
                    }
                    for(Recognition element : rightDetections) {
                        if(element.getConfidence() > rightConfidence) {
                            rightElement = element.getLabel();
                            rightConfidence = element.getConfidence();
                        }
                    }
                    if(centerElement.equals(LABEL_GOLD_MINERAL)) {
                        detectedPosition = POSITION_CENTER;
                    } else if(rightElement.equals(LABEL_GOLD_MINERAL)) {
                        detectedPosition = POSITION_RIGHT;
                    } else if(centerElement.equals(LABEL_SILVER_MINERAL) && rightElement.equals(LABEL_SILVER_MINERAL)){
                        detectedPosition = POSITION_LEFT;
                    } else {
                        detectedPosition = POSITION_UNKNOWN;
                    }
                }
            }
        }
        // Print out for debug
        if(leftDetectedElement != null) {
            telemetry.addData("Left Detected Element: ", leftDetectedElement.getLabel());
            telemetry.addData("Left Top: ", leftDetectedElement.getRight());
            telemetry.addData("Left Left: ", leftDetectedElement.getTop());
            telemetry.addData("Left Detected Confidence: ", leftDetectedElement.getConfidence());
        }
        if(centerDetectedElement != null) {
            telemetry.addData("Center Detected Element: ", centerDetectedElement.getLabel());
            telemetry.addData("Center Top: ", centerDetectedElement.getRight());
            telemetry.addData("Center Left: ", centerDetectedElement.getTop());
            telemetry.addData("Center Detected Confidence: ", centerDetectedElement.getConfidence());
        }
        telemetry.addData("Detected Position: ", detectedPosition);
        updateTelemetry(telemetry);
//        sleep(5000);
        // Shut down Tensor Flow Object Detection
        if( tfod != null ) {
            tfod.shutdown();
            tfod = null;
        }
        return detectedPosition;
    }

    public void windowDetectedResults(int samplePosition, List<Recognition> detectedElements) {
        // Since phone is locked in portrait mode, top is left and -left is top
        // left11, top11 top left corner element on left search box
        // left12, top12 bottom right corner element on left search box
        // left21, top21 top left corner element on right search box
        // left22, top22 bottom right corner element on right search box
        int left11, top11, left12, top12, left21, top21, left22, top22;

        // The tensorflow resolution seems to be about 800x480
        if(samplePosition == POSITION_LEFT) {
            left11 = 0;
            top11 = 240;
            left12 = 400;
            top12 = 0;
            left21 = 400;
            top21 = 240;
            left22 = 900;
            top22 = 0;
        } else if(samplePosition == POSITION_RIGHT) {
            left11 = 0;
            top11 = 540;
            left12 = 400;
            top12 = 0;
            left21 = 400;
            top21 = 540;
            left22 = 900;
            top22 = 0;
        } else {
            // POSITION_CENTER - currently unused.
            left11 = 0;
            top11 = 540;
            left12 = 400;
            top12 = 0;
            left21 = 400;
            top21 = 540;
            left22 = 900;
            top22 = 0;
        }
        for(Recognition element : detectedElements) {
            // Since the phone is in portrait mode, getting right will be the top of the cube
            // and top will be the left of the cube.
            int elementLeft = (int) element.getTop();
            int elementTop = (int) element.getRight();

            // Bin the results
            if(elementLeft > left11 && elementLeft < left12 &&
                    elementTop < top11 && elementTop > top12) {

                if(samplePosition == POSITION_LEFT) {
                    leftDetections.add(element);
                } else if (samplePosition == POSITION_RIGHT) {
                    centerDetections.add(element);
                }
            } else if(elementLeft > left21 && elementLeft < left22 &&
                    elementTop < top21 && elementTop > top22) {
                if(samplePosition == POSITION_LEFT) {
                    centerDetections.add(element);
                } else if (samplePosition == POSITION_RIGHT) {
                    rightDetections.add(element);
                }
            }
        }
    }

    public int sampleElements() {
        int gold_position = POSITION_UNKNOWN;
        int sampled_position = SAMPLE_LEFTSIDE;  // which side was gold_position detected?
        // Reset all the sampling parameters
        initializeSampling();
        // Sample the LEFT and CENTER particles (from relative far back in case viewing angle is off)
        sampled_position = SAMPLE_LEFTSIDE;
        gold_position = identifyGoldCube( sampled_position );
        // If we couldn't tell, assume the CENTER position (1 in 3 chance of being right)
        if (gold_position == POSITION_UNKNOWN ) {
            gold_position = POSITION_CENTER;
        }
        return gold_position;
    }

    void sampleDepot(int position) {
        if(position == POSITION_LEFT) {
            rotateRobotToAngle(0.2, 90.0, 5000);
            driveAtHeadingForTime(0.2, 0.05, 270, 90, 400);
            driveAtHeadingForTime(0.2, 0.05, 0, 90, 350);
            driveAtHeadingForTime(0.2, 0.05, 270, 90, 900);
            rotateRobotToAngle(0.2, 45.0, 5000);
            driveAtHeadingForTime(0.2, 0.05, 225, 45, 500);
            driveAtHeadingForTime(0.2, 0.05, 315, 45, 200);

            robot.setLeftCollectorPower(-1.0);
            robot.setRightCollectorPower(-1.0);
            sleep(500);
            robot.setLeftCollectorPower(0.0);
            robot.setRightCollectorPower(0.0);
            deployArm();

            driveAtHeadingForTime(0.2, 0.1, 43, 43, 1900);
        } else if(position == POSITION_CENTER) {
            rotateRobotToAngle(0.2, 90.0, 5000);
            driveAtHeadingForTime(0.2, 0.05, 270, 90, 400);
            driveAtHeadingForTime(0.2, 0.05, 180, 90, 300);
            driveAtHeadingForTime(0.2, 0.05, 270, 90, 1150);
            rotateRobotToAngle(0.2, 45.0, 5000);
            driveAtHeadingForTime(0.2, 0.05, 315, 45, 450);

            robot.setLeftCollectorPower(-1.0);
            robot.setRightCollectorPower(-1.0);
            sleep(500);
            robot.setLeftCollectorPower(0.0);
            robot.setRightCollectorPower(0.0);
            deployArm();

            driveAtHeadingForTime(0.2, 0.05, 43, 43, 1900);
        } else {
            rotateRobotToAngle(0.3, 90.0, 5000);
            driveAtHeadingForTime(0.2, 0.05, 270, 90, 400);
            driveAtHeadingForTime(0.2, 0.05, 180, 90, 900);
            driveAtHeadingForTime(0.2, 0.05, 270, 90, 750);
            rotateRobotToAngle(0.3, 135.0, 5000);
            driveAtHeadingForTime(0.2, 0.05, 315, 135, 600);
            driveAtHeadingForTime(0.2, 0.05, 135, 135, 150);

            rotateRobotToAngle(0.3, 45.0, 5000);
            driveAtHeadingForTime(0.2, 0.05, 45, 45, 150);
            driveAtHeadingForTime(0.2, 0.05, 315, 45, 550);

            robot.setLeftCollectorPower(-1.0);
            robot.setRightCollectorPower(-1.0);
            sleep(500);
            robot.setLeftCollectorPower(0.0);
            robot.setRightCollectorPower(0.0);
            deployArm();

            driveAtHeadingForTime(0.4, 0.05, 43, 43, 1000);
        }
    }

    void sampleCrater(int position) {
        double slowMove = 0.2;
        double fastMove = 0.4;
        double rotateSpeed = 0.3;
        if(position == POSITION_LEFT) {
            // Lining Up
            rotateRobotToAngle(rotateSpeed, 90.0, 5000);
            driveAtHeadingForTime(slowMove, 0.05, 270, 90, 400);
            driveAtHeadingForTime(slowMove, 0.05, 0, 90, 350);
            driveAtHeadingForTime(slowMove, 0.05, 270, 90, 350);
            driveAtHeadingForTime(slowMove, 0.05, 90, 90, 200);

            // Go to wall
            driveAtHeadingForTime(slowMove, 0.05, 0, 90, 800);
            rotateRobotToAngle(rotateSpeed, 225.0, 5000);
            driveAtHeadingForTime(slowMove, 0.05, 315, 225, 520);
            driveAtHeadingForTime(fastMove, 0.1, 43, 227, 725);

            robot.setLeftCollectorPower(-1.0);
            robot.setRightCollectorPower(-1.0);
            sleep(500);
            robot.setLeftCollectorPower(0.0);
            robot.setRightCollectorPower(0.0);
            deployArm();

            driveAtHeadingForTime(fastMove, 0.1, 227, 227, 1000);
        } else if(position == POSITION_CENTER) {
            // Lining Up
            rotateRobotToAngle(rotateSpeed, 90.0, 5000);
            driveAtHeadingForTime(slowMove, 0.05, 270, 90, 400);
            driveAtHeadingForTime(slowMove, 0.05, 180, 90, 350);
            driveAtHeadingForTime(slowMove, 0.05, 270, 90, 350);
            driveAtHeadingForTime(slowMove, 0.05, 90, 90, 350);

            // Go to wall
            driveAtHeadingForTime(fastMove, 0.05, 0, 90, 650);
            rotateRobotToAngle(rotateSpeed, 225.0, 5000);
            driveAtHeadingForTime(slowMove, 0.05, 315, 225, 600);
            driveAtHeadingForTime(fastMove, 0.1, 43, 227, 650);

            robot.setLeftCollectorPower(-1.0);
            robot.setRightCollectorPower(-1.0);
            sleep(500);
            robot.setLeftCollectorPower(0.0);
            robot.setRightCollectorPower(0.0);
            deployArm();

            driveAtHeadingForTime(fastMove, 0.1, 227, 227, 950);
        } else {
            // Lining Up
            rotateRobotToAngle(rotateSpeed, 90.0, 5000);
            driveAtHeadingForTime(slowMove, 0.05, 270, 90, 400);
            driveAtHeadingForTime(slowMove, 0.05, 180, 90, 850);

            driveAtHeadingForTime(slowMove, 0.05, 270, 90, 350);
            driveAtHeadingForTime(slowMove, 0.05, 90, 90, 350);

            // Go to wall
            driveAtHeadingForTime(fastMove, 0.05, 0, 90, 950);
            rotateRobotToAngle(rotateSpeed, 225.0, 5000);
            driveAtHeadingForTime(slowMove, 0.05, 315, 225, 550);
            driveAtHeadingForTime(fastMove, 0.1, 43, 227, 600);

            robot.setLeftCollectorPower(-1.0);
            robot.setRightCollectorPower(-1.0);
            sleep(500);
            robot.setLeftCollectorPower(0.0);
            robot.setRightCollectorPower(0.0);
            deployArm();

            driveAtHeadingForTime(fastMove, 0.1, 227, 227, 900);
        }
    }

    void placeMarkerDepot() {
        driveAtHeadingForTime(0.2, 0.05, -90, 0, 500);
        rotateRobotToAngle(0.2, 90, 5000);
        driveAtHeadingForTime(0.2, 0.05, 270, 90, 1700);
        robot.setLeftCollectorPower(1.0);
        robot.setRightCollectorPower(1.0);
        sleep(500);
        robot.setLeftCollectorPower(0.0);
        robot.setRightCollectorPower(0.0);
        rotateRobotToAngle(0.2, 50, 5000);
        driveAtHeadingForTime(0.2, 0.05, 0, 50, 300);
        driveAtHeadingForTime(0.2, 0.05, 40, 50, 2500);
    }

    void placeMarkerCrater() {
        driveAtHeadingForTime(0.2, 0.05, -90, 0, 600);
//        driveAtHeadingForTime(0.5, 0.1, 90, 0, 200);
        driveAtHeadingForTime(0.2, 0.05, 0, 0, 1600);
        rotateRobotToAngle(0.2, 210, 5000);
        driveAtHeadingForTime(0.2, 0.05, 40, 220, 1200);
        robot.setLeftCollectorPower(1.0);
        robot.setRightCollectorPower(1.0);
        sleep(500);
        robot.setLeftCollectorPower(0.0);
        robot.setRightCollectorPower(0.0);
        driveAtHeadingForTime(0.2, 0.05, 220, 230, 2200);
    }
    /**
     * Rotate the robot clockwise without regard to shortest rotation
     *
     * @param maxSpeed      - Maximum speed to spin
     * @param rotationAngle - The angle to rotate the robot
     * @param maxTime       - How long to attempt to spin before giving up
     */
    public void spinClockwise(double maxSpeed, double rotationAngle, int maxTime) {
        double currentAngle = 0.0;
        double lastAngle = currentAngle;
        double angleTraveled = 0.0;
        int elapsedTime = 0;
        double spinRate = 0.0;
        final double SAME_ANGLE = 1.0;
        final int DELTA_SLEEP = 10;
        double deltaAngle = 0.0;

        while ((elapsedTime < maxTime) && (Math.abs(rotationAngle - angleTraveled) > SAME_ANGLE)) {
            currentAngle = robot.readIMU();
            if (currentAngle >= lastAngle) {
                angleTraveled += currentAngle - lastAngle;
            } else {
                angleTraveled += 360.0 + currentAngle - lastAngle;
            }
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.addData("Last Angle: ", lastAngle);

            deltaAngle = angleTraveled - rotationAngle;
            telemetry.addData("Delta Angle: ", deltaAngle);
            spinRate = controlledRotationAngle(deltaAngle, maxSpeed);
            // We went too far, come back
            if (deltaAngle < 0.0) {
                robot.drive(0.0, 0.0, -spinRate, 0.0);
                telemetry.addData("Spin Rate: ", -spinRate);
            } else {
                robot.drive(0.0, 0.0, spinRate, 0.0);
                telemetry.addData("Spin Rate: ", spinRate);
            }
            if (isStopRequested()) {
                break;
            }
            sleep(DELTA_SLEEP);
            elapsedTime += DELTA_SLEEP;

            telemetry.addData("Elapsed Time: ", elapsedTime);
            telemetry.update();
        }
        robot.setAllDriveZero();
    }

    /**
     * Rotate the robot without regard to shortest rotation angle
     *
     * @param maxSpeed      - Maximum speed to spin, negative is CCW
     * @param rotationAngle - The angle to rotate the robot, negative is CCW
     * @param maxTime       - How long to attempt to spin before giving up
     */
    public void spinRobot(double maxSpeed, double rotationAngle, int maxTime) {
        double currentAngle = robot.readIMU();
        double lastAngle = currentAngle;
        double angleTraveled = 0.0;
        int elapsedTime = 0;
        double spinRate = 0.0;
        final double SAME_ANGLE = 1.0;
        final int DELTA_SLEEP = 10;
        double deltaAngle = 0.0;

        double normalizedAngle = 0.0;
        double normalizedSpeed = 0.0;

        // Final version will be speed with a positive angle value.
        // Speed will dictate CW or CCW
        if (rotationAngle < 0.0) {
            normalizedSpeed = -maxSpeed;
            normalizedAngle = -rotationAngle;
        } else {
            normalizedSpeed = maxSpeed;
            normalizedAngle = rotationAngle;
        }

        while ((elapsedTime < maxTime) && (Math.abs(normalizedAngle - angleTraveled) > SAME_ANGLE)
                && (!isStopRequested())) {
            currentAngle = robot.readIMU();
            if (normalizedSpeed < 0.0) {
                if (currentAngle >= lastAngle) {
                    angleTraveled += currentAngle - lastAngle;
                } else {
                    angleTraveled += 360.0 + currentAngle - lastAngle;
                }
            } else {
                if (currentAngle <= lastAngle) {
                    angleTraveled += lastAngle - currentAngle;
                } else {
                    angleTraveled += 360.0 + lastAngle - currentAngle;
                }
            }
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.addData("Last Angle: ", lastAngle);
            lastAngle = currentAngle;

            deltaAngle = normalizedAngle - angleTraveled;
            telemetry.addData("Delta Angle: ", deltaAngle);
            spinRate = controlledRotationAngle(deltaAngle, normalizedSpeed);

            if (normalizedSpeed < 0.0) {
                // CCW
                // We went too far, come back
                if (deltaAngle < 0.0) {
                    robot.drive(0.0, 0.0, -spinRate, 0.0);
                    telemetry.addData("Spin Rate: ", spinRate);
                } else {
                    robot.drive(0.0, 0.0, spinRate, 0.0);
                    telemetry.addData("Spin Rate: ", -spinRate);
                }
            } else {
                // CW
                // We went too far, come back
                if (deltaAngle < 0.0) {
                    robot.drive(0.0, 0.0, -spinRate, 0.0);
                    telemetry.addData("Spin Rate: ", -spinRate);
                } else {
                    robot.drive(0.0, 0.0, spinRate, 0.0);
                    telemetry.addData("Spin Rate: ", spinRate);
                }
            }
            sleep(DELTA_SLEEP);
            elapsedTime += DELTA_SLEEP;

            telemetry.addData("Destination Angle: ", normalizedAngle);
            telemetry.addData("Traveled Angle: ", angleTraveled);
            telemetry.addData("Elapsed Time: ", elapsedTime);
            telemetry.update();
        }
        robot.setAllDriveZero();
    }

    /*--------------------------------------------------------------------------------------------*/
    private void initVuforia() {
        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        float mmPerInch        = 25.4f;
        float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
        float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);  // Use this line to see camera display
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();                             // OR... Use this line to improve performance

        parameters.vuforiaLicenseKey = "AQTDKlf/////AAAAGQMApYs8y0xSld/ybTpELU0IcfIRlGXWhNor7tRATP50ohGS0vvyd7UqNyqo0jMZBqr+vsZFAL0jhYI1gapwBSqDLxEyaZiFjGHnODarQ/FUt4EBrX93AwXhIIBrPAhqng8chus9sszS2Yb71AhhlUlLTzSrhZT4dhpOmlc0pjIwv2aBefeS4crF7TWw4VyqrxJwXXOU1fCfOH6H+UuIHLHoaPXmYuWJb+I8DGncabEPlmgyUD2d5m2xdYnQaEASkKPyYleWKoZtfdmG7aL8mu95JPazqpjz1jHbjjBZ3bObK7dHN7eJ8cFZhg1m+AOLyqkBOeb1lEWcBpAX3VS3vQKqje0tAmm8uBFum0OAbVCl";
        parameters.cameraDirection = BACK;  // FRONT is control sise; BACK is high resolution camera

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

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
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, AxesOrder.XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, AxesOrder.XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, AxesOrder.XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, AxesOrder.XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 180 mm rear the middle of the robot, and 170 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 180;  // eg: Camera is 180 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 170;  // eg: Camera is 170 mm above floor
        final int CAMERA_LEFT_DISPLACEMENT     = 0;    // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        -90, 0, 0 ) );

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    } // initVuforia

    /*--------------------------------------------------------------------------------------------*/
    public void activateTracking() {
        // Start tracking any of the defined Vuforia images
        if (targetsRoverRuckus != null)
            targetsRoverRuckus.activate();;
    } // activateTracking

    /*--------------------------------------------------------------------------------------------*/
    // Initialize the Tensor Flow Object Detection engine
    // The variables "tfod" and "vuforia" are defined external to this code.
    private void initTfod() {
        final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        // Specifying an ID for a "camera monitor view" tells TensorFlow to display an overlay on the
        // Robot Controller screeen showing TensorFlow activity (including bounding boxes for detected objects).
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        /*      TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(); // camera monitor view disabled */
        tfodParameters.useObjectTracker  = false; // DISABLE object tracker (the default is ENABLED)
        tfodParameters.minimumConfidence = 0.40;  // be more selective when identifying objects? (default is 40% confident)
        // Construct a TensorFlow object using the parameters listed above (assumes "vuforia" already initialized)
        tfod = ClassFactory.getInstance().createTFObjectDetector( tfodParameters, vuforia );
        // Load the TensorFlow model data for the Gold and Silver Minerals
        tfod.loadModelFromAsset( TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL );
    } // initTfod

    /*--------------------------------------------------------------------------------------------*/
    // In case we sample from more than one location (or double-sample for our alliance partner),
    // we DYNAMICALLY initialize the sampling variables here instead of STATICALLY initializing
    // when they're declared.
    private void initializeSampling() {
        sampleLeftCenType='?';   sampleRightCenType='?';
        sampleLeftLefType='?';   sampleRightLefType='?';
        sampleLeftRigType='?';   sampleRightRigType='?';
        sampleLeftCenX=0;        sampleRightCenX=0;
        sampleLeftLefX=0;        sampleRightRigX=0;
        sampleLeftCenY=0;        sampleRightCenY=0;
        sampleLeftLefY=0;        sampleRightRigY=0;
        sampleLeftCenC=0.0f;     sampleRightCenC=0.0f;
        sampleLeftLefC=0.0f;     sampleRightRigC=0.0f;

        mineralLefType='?';  mineralCenType='?';  mineralRigType='?';
        mineralLefX=0;       mineralCenX=0;       mineralRigX=0;
        mineralLefY=0;       mineralCenY=0;       mineralRigY=0;
        mineralLefC=0.0f;    mineralCenC=0.0f;    mineralRigC=0.0f;
        return;
    } // initializeSampling

    /*--------------------------------------------------------------------------------------------*/
    // Use Tensor Flow Object Detection engine to sample the gold mineral.
    // NOTES:
    // 1) The reported values of LEFT & TOP depend on both the phone orientation (portrait/landscape)
    // as well as whether SCREEN ROTATION is enabled (or LOCKED to portrait mode even when horizontal).
    // We have SCREEN ROTATION disabled (ie, always in PORTRAIT mode even though phone mounted sideways).
    // This gives us a larger display on the screen for development/debugging, but means we have to
    // reverse the TOP and LEFT results from TensorFlow.
    // 2) Depending on the sampling position (closer than 36"), it's possible not all 3 particles are
    // visible.  When 24" away, only 2 of the 3 are visible and the two possible viewpoint positions are:
    //   SAMPLE_RIGHTSIDE --> only CENTER & RIGHT particles visible
    //   SAMPLE_LEFTSIDE  --> only LEFT & CENTER particles visible
    // If both visible positions are detected as SILVER, then the 3rd (non-visible) position is
    // inferred to be the GOLD particle.
    //
    // Returns:  POSITION_LEFT, POSITION_CENTER, POSITION_RIGHT, or POSITION_UNKNOWN
    private int identifyGoldCube( int viewPoint ) {
        int   gold_position = POSITION_UNKNOWN;  // initialize result to UNKNOWN
        float gold_confidence = 0.0f;
        int   y_cutoff = (teamNumber1)? 1280:1200;  // ignore background particles behind the sampling field
        // Abort if Tensor Flow Object Detection initialization failed
        if( tfod != null ) {
            int loop;
            int particles_detected;
            // Loop 3 times, in case detection is slow to recognize
            for( loop=0; loop<3; loop++ ) {
                // Wait 0.75 sec for next acquisition attempt
                sleep( 750 );
                // getUpdatedRecognitions() will return null if no new information is available yet
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    particles_detected = updatedRecognitions.size();
                    if (particles_detected > 0) {
                        for (Recognition recognition : updatedRecognitions) {
                            // WHERE was the particle detected? (query TOP & LEFT corners of the detection bounding box)
                            int xPixel = (int) recognition.getTop();  // see Note 1 above about TOP vs LEFT
                            int yPixel = (int) recognition.getLeft();
                            float conf = recognition.getConfidence();
                            boolean isGold = (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) ? true : false;
                            // Display the objection detection info
                            telemetry.addData(((isGold) ? "GOLD " : "SILVER "), "x=%d y=%d (%.3f)", xPixel, yPixel, conf);
                            // Assign LEFT/CENTER/RIGHT based on X-pixel value (and observation position).  Since we center the
                            // robot on the left-two and right-two, the midpoint of the phone display is the dividing point.
                            if( viewPoint == SAMPLE_LEFTSIDE ) {
                                if (xPixel < PHONE_DISPLAY_MIDPOINT ) {
                                    if (conf > sampleLeftLefC) {
                                        sampleLeftLefC = conf;
                                        sampleLeftLefX = xPixel;
                                        sampleLeftLefY = yPixel;
                                        sampleLeftLefType = (isGold) ? 'G' : 'S';
                                    }
                                } // LEFT
                                else {
                                    if (conf > sampleLeftCenC) {
                                        sampleLeftCenC = conf;
                                        sampleLeftCenX = xPixel;
                                        sampleLeftCenY = yPixel;
                                        sampleLeftCenType = (isGold) ? 'G' : 'S';
                                    }
                                } // CENTER
                            } // SAMPLE_LEFTSIDE
                            else {
                                if (xPixel < PHONE_DISPLAY_MIDPOINT ) {
                                    if (conf > sampleRightCenC) {
                                        sampleRightCenC = conf;
                                        sampleRightCenX = xPixel;
                                        sampleRightCenY = yPixel;
                                        sampleRightCenType = (isGold) ? 'G' : 'S';
                                    }
                                } // CENTER
                                else {
                                    if (conf > sampleRightRigC) {
                                        sampleRightRigC = conf;
                                        sampleRightRigX = xPixel;
                                        sampleRightRigY = yPixel;
                                        sampleRightRigType = (isGold) ? 'G' : 'S';
                                    }
                                } // RIGHT
                            } // SAMPLE_RIGHTSIDE
                        } // for(3 loops)

                        // Now that all detections have been processed, check for "inferred" facts.
                        // 1) If both LEFT/CENTER are high-confidence silver, then RIGHT is very likely gold.
                        // 2) Likewise, if both CENTER/RIGHT are silver then LEFT is very likely the gold.
                        // We could also infer identify of the 3rd mineral as SILVER, but don't care.
                        if ((sampleLeftLefType == 'S') && (sampleLeftLefC >= 0.5) &&
                                (sampleLeftCenType == 'S') && (sampleLeftCenC >= 0.5)) {
                            sampleLeftRigType = 'G';
                            mineralRigType = 'G';
                            mineralRigC = (sampleLeftLefC < sampleLeftCenC) ? sampleLeftLefC : sampleLeftCenC;
                        }
                        if ((sampleRightCenType == 'S') && (sampleRightCenC >= 0.5) &&
                                (sampleRightRigType == 'S') && (sampleRightRigC >= 0.5)) {
                            sampleRightLefType = 'G';
                            mineralLefType = 'G';
                            mineralLefC = (sampleRightCenC < sampleRightRigC) ? sampleRightCenC : sampleRightRigC;
                        }

                        // Now make a final decision about the LEFT/CENTER/RIGHT minerals:
                        // (are these results higher confidence than what we started with?)
                        if (sampleLeftLefC > mineralLefC) {   // left sample vs. inferred left
                            mineralLefC = sampleLeftLefC;
                            mineralLefType = sampleLeftLefType;
                            mineralLefX = sampleLeftLefX;
                            mineralLefY = sampleLeftLefY;
                        }
                        if (sampleRightRigC > mineralRigC) {  // right sample vs. inferred right
                            mineralRigC = sampleRightRigC;
                            mineralRigType = sampleRightRigType;
                            mineralRigX = sampleRightRigX;
                            mineralRigY = sampleRightRigY;
                        }
                        if (sampleLeftCenC > sampleRightCenC) { // both center samples:
                            mineralCenC = sampleLeftCenC;       // (left had higher confidence)
                            mineralCenType = sampleLeftCenType;
                            mineralCenX = sampleLeftCenX;
                            mineralCenY = sampleLeftCenY;
                        } else {
                            mineralCenC = sampleRightCenC;      // (right had higher confidence)
                            mineralCenType = sampleRightCenType;
                            mineralCenX = sampleRightCenX;
                            mineralCenY = sampleRightCenY;
                        }  // use the highest-confidence center sample

                        // Now make a decision about the GOLD location (8 total permutations):
                        // Start with the simple 4 cases (no gold, or only 1 gold detected)
                        if ((mineralLefType != 'G') && (mineralCenType != 'G') && (mineralRigType != 'G')) {
                            gold_position   = POSITION_UNKNOWN;
                            gold_confidence = 0.0f;  // we're 100% sure we DON'T know where it is :-)
                        }
                        else if ((mineralLefType == 'G') && (mineralCenType != 'G') && (mineralRigType != 'G')) {
                            gold_position = POSITION_LEFT;
                            gold_confidence = mineralLefC;
                        }
                        else if ((mineralLefType != 'G') && (mineralCenType == 'G') && (mineralRigType != 'G')) {
                            gold_position = POSITION_CENTER;
                            gold_confidence = mineralCenC;
                        }
                        else if ((mineralLefType != 'G') && (mineralCenType != 'G') && (mineralRigType == 'G')) {
                            gold_position = POSITION_RIGHT;
                            gold_confidence = mineralRigC;
                        }
                        // Now handle any complex cases where more than 1 gold is detected (use the highest-confidence location)
                        else if ((mineralLefType == 'G') && (mineralCenType == 'G') && (mineralRigType == 'G')) {
                            if (mineralLefC > mineralCenC) {
                                gold_position   = (mineralLefC >= mineralRigC)? POSITION_LEFT : POSITION_RIGHT;
                                gold_confidence = (mineralLefC >= mineralRigC)?   mineralLefC : mineralRigC;
                            } else {
                                gold_position   = (mineralCenC >= mineralRigC)? POSITION_CENTER : POSITION_RIGHT;
                                gold_confidence = (mineralCenC >= mineralRigC)?   mineralCenC : mineralRigC;
                            }
                        } // GGG
                        else if ((mineralLefType == 'G') && (mineralCenType == 'G') && (mineralRigType != 'G')) {
                            gold_position   = (mineralLefC > mineralCenC)? POSITION_LEFT : POSITION_CENTER;
                            gold_confidence = (mineralLefC > mineralCenC)?   mineralLefC : mineralCenC;
                        }
                        else if ((mineralLefType != 'G') && (mineralCenType == 'G') && (mineralRigType == 'G')) {
                            gold_position   = (mineralCenC > mineralRigC)? POSITION_CENTER : POSITION_RIGHT;
                            gold_confidence = (mineralCenC > mineralRigC)?     mineralCenC : mineralRigC;
                        }
                        else if ((mineralLefType == 'G') && (mineralCenType != 'G') && (mineralRigType == 'G')) {
                            gold_position   = (mineralLefC > mineralRigC)? POSITION_LEFT : POSITION_RIGHT;
                            gold_confidence = (mineralLefC > mineralRigC)?   mineralLefC : mineralRigC;
                        }
                        else { // should never reach here
                            gold_position   = POSITION_UNKNOWN;
                            gold_confidence = 0.0f;
                        }

                        // Finally, if this is the 1st sample then we must achieve 80% confidence
                        // (or else we trigger the 2nd measurement to ensure it's not a bad detection)
                        if( (viewPoint == SAMPLE_LEFTSIDE) && (gold_confidence < 0.70f) ) {
                            gold_position   = POSITION_UNKNOWN;
                            gold_confidence = 0.0f;
                        }
                    } // particles_detected > 0
                    updateSamplingTelemetry();
                    if (gold_position == POSITION_LEFT)
                        telemetry.addData("Gold =", "LEFT");
                    else if (gold_position == POSITION_CENTER)
                        telemetry.addData("Gold =", "CENTER");
                    else if (gold_position == POSITION_RIGHT)
                        telemetry.addData("Gold =", "RIGHT");
                    else
                        telemetry.addData("Gold =", "UNKNOWN");
                    telemetry.update();
                } // updatedRecognitions != null
            } // for()
        } // tfod != null
             sleep(7000);  // DEBUG (give time to review results)
        return gold_position;
    } // identifyGoldCube
    /*--------------------------------------------------------------------------------------------*/
    // Visually display the results of the TensorFlow sampling from left and right sides.
    // Unknown information is displayed as "?".
    private void updateSamplingTelemetry() {
        // We always have SAMPLE1 data when we call this method (so display it)
        telemetry.addData("#1","%c (%.3f)    %c (%.3f)    %c (inf)",
                sampleLeftLefType, sampleLeftLefC,
                sampleLeftCenType, sampleLeftCenC,
                sampleLeftRigType );
        telemetry.addData("  ","x=%-3d,y=%-3d  x=%-3d,y=%-3d",
                sampleLeftLefX, sampleLeftLefY, sampleLeftCenX, sampleLeftCenY );
        // Only display SAMPLE2 data if it's been collected:
        if( (sampleRightLefType != '?') || (sampleRightCenType != '?') || (sampleRightRigType != '?')  ) {
            telemetry.addData("#2","%c (inf)    %c (%.3f)    %c (%.3f)",
                    sampleRightLefType,
                    sampleRightCenType, sampleRightCenC,
                    sampleRightRigType, sampleRightRigC );
            telemetry.addData("  ", "            x=%-3d,y=%-3d  x=%-3d,y=%-3d",
                    sampleRightCenX, sampleRightCenY, sampleRightRigX, sampleRightRigY);
        }
        // Display what we know so far for a result:
        telemetry.addData("==>","%c (%.3f)  %c (%.3f)  %c (%.3f)",
                mineralLefType, mineralLefC,
                mineralCenType, mineralCenC,
                mineralRigType, mineralRigC );
        telemetry.addData(" ","x=%-3d,y=%-3d  x=%-3d,y=%-3d  x=%-3d,y=%-3d",
                mineralLefX, mineralLefY, mineralCenX, mineralCenY, mineralRigX, mineralRigY );
        return;
    } // updateSamplingTelemetry
}