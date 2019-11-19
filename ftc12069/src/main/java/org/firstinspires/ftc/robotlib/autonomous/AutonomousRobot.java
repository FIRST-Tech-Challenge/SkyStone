package org.firstinspires.ftc.robotlib.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotlib.information.Area;
import org.firstinspires.ftc.robotlib.information.OrientationInfo;
import org.firstinspires.ftc.robotlib.navigation.Point3D;
import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;
import org.firstinspires.ftc.robotlib.state.Alliance;
import org.firstinspires.ftc.robotlib.navigation.Point;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class AutonomousRobot {
    private String vuforiaKey;
    public MecanumHardwareMap hardware;
    private Alliance alliance;
    private Telemetry telemetry;

    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private OpenGLMatrix lastRobotLocation = null;
    private long lastRobotLocationTime;
    private OpenGLMatrix lastRobotLocationFromSkystone;

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;  // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private static final boolean PHONE_IS_PORTRAIT = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    public VuforiaTrackables trackables;
    public List<VuforiaTrackable> trackablesList;
    public List<VuforiaTrackable> visibleTrackables;

    // Constants for Areas
    private final Area blueBridge = new Area(new Point(2, 72), new Point(-2, 24));
    private final Area redBridge = new Area(new Point(2, -24), new Point(-2, -72));
    private final Area buildingZone = new Area(new Point(72, 72), new Point(10, -72));
    private final Area loadingZone = new Area(new Point(-72, 72), new Point(-10, -72));

    /**
     * Creates an instance of an autonomous robot manager
     * @param hwMap FTC hardware map
     * @param vuforiaKey Vuforia Key
     * @param alliance Alliance Enum
     * @param telemetry Logging
     */
    public AutonomousRobot(HardwareMap hwMap, String vuforiaKey, Alliance alliance, Telemetry telemetry) {
        this.hardware = new MecanumHardwareMap(hwMap);
        this.vuforiaKey = vuforiaKey;
        this.alliance = alliance;
        this.telemetry = telemetry;
    }

    /**
     * Initializes the robot (specifically Vuforia)
     */
    public void init() {
        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardware.internalHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.internalHardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //parameters.cameraName = hardware.webcamName;
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = vuforiaKey;
        parameters.cameraDirection = CAMERA_CHOICE;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        trackables = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = trackables.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = trackables.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = trackables.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = trackables.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = trackables.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = trackables.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = trackables.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = trackables.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = trackables.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = trackables.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = trackables.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = trackables.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = trackables.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        trackablesList = new ArrayList<>(trackables);
        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
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
            phoneXRotate = 90 ;
        }
        final float CAMERA_FORWARD_DISPLACEMENT  = 0f * mmPerInch;   // eg: Camera is 0 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0f * mmPerInch;   // eg: Camera is 6.625 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : trackablesList) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        visibleTrackables = new ArrayList<>();
    }

    /**
     * Scans the area for Vuforia trackables and saves them in a list
     * If a trackable is found then the robot will update its position using the trackable
     * Use this in the game loop to get the most up-to-date information from Vuforia
     */
    public void scan() {
        visibleTrackables.clear();
        for (VuforiaTrackable trackable : trackablesList) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                visibleTrackables.add(trackable);

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    if (trackable.getName().equals("Stone Target")) lastRobotLocationFromSkystone = robotLocationTransform;
                    else {
                        lastRobotLocation = robotLocationTransform;
                        lastRobotLocationTime = System.currentTimeMillis();
                    }
                }
            }
        }
    }

    /**
     * Checks if the robot knows a location (might not be updated)
     */
    public boolean isLocationKnown() {
        return lastRobotLocation != null;
    }

    /**
     * Checks if there is any visible trackables
     */
    public boolean isTrackableVisible() {
        return visibleTrackables.size() > 0;
    }

    /**
     * Stringifies visible targets
     * @return string separated by a comma
     */
    public String stringifyVisibleTargets() {
        StringBuilder stringTargets = new StringBuilder();
        for (VuforiaTrackable vuforiaTrackable : visibleTrackables) {
            stringTargets.append(vuforiaTrackable.getName()).append(", ");
        }
        return stringTargets.toString();
    }

    /**
     * Retrieves the current position of the robot (only works if there is a visible trackable)
     * @return current position on the FTC field
     */
    public Point3D getPosition() {
        if (lastRobotLocation == null) return null;
        //if (!isTrackableVisible()) return null;
        // express position (translation) of robot in inches.
        VectorF translation = lastRobotLocation.getTranslation();
        return new Point3D(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
    }

    /**
     * Retrieves the current position of the robot (only works if there is a visible Skystone)
     * @return current position relative to the Skystone
     */
    public Point3D getPositionFromSkystone() {
        if (lastRobotLocationFromSkystone == null) return null;
        // express position (translation) of robot in inches.
        VectorF translation = lastRobotLocationFromSkystone.getTranslation();
        return new Point3D(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
    }

    /**
     * Retrieves the orientation of the robot in 2D space (heading)
     * @return current heading
     */
    public double getOrientation2D() {
        return Orientation.getOrientation(lastRobotLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
    }

    /**
     * Retrieves the orientation of the robot in 3D space
     * @return current orientation
     */
    public Orientation getOrientation() {
        // imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        return Orientation.getOrientation(lastRobotLocation, EXTRINSIC, XYZ, DEGREES);
    }

    /**
     * Turns the robot by an angle
     * This blocks the current thread
     * @param angle angle to turn to
     * @param velocity rotation speed (between 0 and 1)
     */
    public void turn(double angle, double velocity) {
        double initialOrientation = this.getOrientation2D();
        double[] rotationValues = hardware.drivetrain.getWheelRotationValues(velocity);

        while (this.getOrientation2D() - initialOrientation < angle) {
            hardware.drivetrain.setMotorPowers(rotationValues[0], rotationValues[1], rotationValues[2], rotationValues[3]);
        }
    }

    /**
     * Checks if a trackable with a certain name is visible
     * @param name trackable name identifier
     * @return Trackable information if visible (else null)
     */
    public VuforiaTrackable getVisibleTrackable(String name) {
        for (VuforiaTrackable trackable : trackablesList) {
            if (trackable.getName().equals(name)) return trackable;
        }
        return null;
    }

    /**
     * Calculates the course for the robot to arrive a point in a 2D space
     * @param object Point3D
     * @return required course to arrive at a point
     */
    public double getCourseFromRobot(Point object) {
        Point3D robotPosition = this.getPosition();
        return Math.atan2(robotPosition.y - object.y, robotPosition.x - object.x);
    }

    /**
     * Calculates the course between a set "robot" point and an object
     * @param robot point of reference
     * @param object object to get course to
     * @return required course to arrive at a point
     */
    public double getCourse(Point robot, Point object) {
        return Math.atan2(robot.y - object.y, robot.x - object.x);
    }

    /**
     * Calculates the course for the robot to arrive a point in a 3D space
     * @param object Point3D
     * @return required course to arrive at a point
     */
    public double getCourseFromRobot3D(Point3D object) {
        Point3D robotPosition = this.getPosition();
        double robotMagnitude = Math.sqrt(Math.pow(robotPosition.x, 2) + Math.pow(robotPosition.y, 2) + Math.pow(robotPosition.z, 2));
        double objectMagnitude = Math.sqrt(Math.pow(object.x, 2) + Math.pow(object.y, 2) + Math.pow(object.z, 2));

        return Math.acos(robotPosition.multiply(object)/(robotMagnitude - objectMagnitude));
    }

    /**
     * Calculates the distance between a point on the field and the robot
     * @param object Point3D
     * @return distance between point and robot center
     */
    public double getDistanceFromRobot(Point object) {
        return this.getPosition().distance(object);
    }

    /**
     * Calculates the distance between a point on the field and a supposed robot point
     * @param robot supposed robot point
     * @param object object point
     * @return distance between point and "robot"
     */
    public double getDistance(Point robot, Point object) {
        return this.getPosition().distance(object);
    }

    /**
     * Automatically moves the robot under the bridge
     */
    public void parkUnderBridge() {
        Point3D robotPosition = this.getPosition();
        //TODO Account for robot size
        if (alliance == Alliance.BLUE) {
            this.moveToPoint(new Point(robotPosition.x, blueBridge.getCornerPoint2().y), 1);
            this.moveToPoint(new Point(blueBridge.getMiddleX(), robotPosition.y), 1);
        } else if (alliance == Alliance.RED) {
            this.moveToPoint(new Point(robotPosition.x, redBridge.getCornerPoint2().y), 1);
            this.moveToPoint(new Point(redBridge.getMiddleX(), robotPosition.y), 1);
        }
    }

    /**
     * Moves the robot using a course, velocity, rotation, and distance
     * This adjusts some values, not including rotation
     * @param course Angle (in degrees) of movement
     * @param velocity New velocity (-1 to 1)
     * @param rotation Double (between -1 and 1) -1 - clockwise 0 - no rotation 1 - counterclockwise
     * @param distance Distance (in inches) to execute this movement
     */
    public void simpleMove(double course, double velocity, double rotation, double distance) {
        hardware.drivetrain.setCourse(course * Math.PI / 180);
        hardware.drivetrain.setRotation(rotation);
        hardware.drivetrain.setVelocity(velocity);
        hardware.drivetrain.setTargetPosition(distance * hardware.motorTicksPerInch);
        hardware.drivetrain.position();
    }

    /**
     * Moves the robot using a course, velocity, rotation, and distance
     * @param course Angle (in degrees) of movement
     * @param velocity New velocity (-1 to 1)
     * @param orientationInfo Angle (in degrees) of orientation relative to current orientation
     * @param distance Distance (in inches) to execute this movement
     */
    public void move(double course, double velocity, OrientationInfo orientationInfo, double distance) {
        hardware.drivetrain.setCourse(course * Math.PI / 180);
        hardware.drivetrain.setVelocity(velocity);
        hardware.drivetrain.setTargetPosition(distance * hardware.motorTicksPerInch);

        double initialOrientation = this.getOrientation2D();

        while (hardware.drivetrain.isPositioning()) {
            if (this.getOrientation2D() - initialOrientation < orientationInfo.angle) hardware.drivetrain.setRotation(orientationInfo.rotation);
        }

        // In case the robot did not finish turning by the time it reached its destination
        if (this.getOrientation2D() - initialOrientation < orientationInfo.angle)
            this.turn(orientationInfo.angle - (this.getOrientation2D() - initialOrientation), orientationInfo.rotation);
    }

    /**
     * Moves the robot to a point
     * @param point point to move to
     * @param velocity New velocity
     */
    public void moveToPoint(Point point, double velocity) {
        this.simpleMove(this.getCourseFromRobot(point), velocity, 0, this.getDistanceFromRobot(point));
    }

    /**
     * Moves the robot to a point
     * @param robot supposed robot point
     * @param point point to move to
     * @param velocity New velocity
     */
    public void moveToPoint(Point robot, Point point, double velocity) {
        this.simpleMove(this.getCourse(robot, point), velocity, 0, this.getDistance(robot, point));
    }
}