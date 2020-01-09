package org.firstinspires.ftc.robotlib.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotlib.Constants;
import org.firstinspires.ftc.robotlib.information.LocationInfo;
import org.firstinspires.ftc.robotlib.information.OrientationInfo;
import org.firstinspires.ftc.robotlib.navigation.Area;
import org.firstinspires.ftc.robotlib.navigation.Point;
import org.firstinspires.ftc.robotlib.navigation.Point3D;
import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;
import org.firstinspires.ftc.robotlib.robot.OdometricalMecanumHardwareMap;
import org.firstinspires.ftc.robotlib.state.Alliance;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Class for easily controlling an autonomous robot.
 * Vuforia is handled internally, providing only important information.
 */
public class AutonomousRobot {
    public OdometricalMecanumHardwareMap hardware;
    private Alliance alliance;
    private Telemetry telemetry;

    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private LocationInfo locationInfo;
    private ElapsedTime elapsedTime;

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

    // Constants for Areas/Points on the field
    private final Area blueBridge = new Area(new Point(2, 72), new Point(-2, 24));
    private final Area redBridge = new Area(new Point(2, -24), new Point(-2, -72));
    public final Area buildingZone = new Area(new Point(72, 72), new Point(10, -72));
    public final Area loadingZone = new Area(new Point(-72, 72), new Point(-10, -72));

    // Foundation Points
    private final Point blueFoundationMoveLocation = new Point(42, 24);
    private final Point redFoundationMoveLocation = blueFoundationMoveLocation.opponentPoint();
    private final Point blueFoundationFinalLocation = new Point(42, 67);
    private final Point redFoundationFinalLocation = blueFoundationFinalLocation.opponentPoint();

    // Loading Points
    private final Point blueLoadingScanLocation = new Point(loadingZone.getMiddleX(), 40);
    private final Point redLoadingScanLocation = blueLoadingScanLocation.opponentPoint();

    /**
     * Creates an instance of an autonomous robot manager.
     * @param hwMap FTC hardware map
     * @param alliance Alliance Enum
     * @param telemetry Logging
     */
    public AutonomousRobot(HardwareMap hwMap, Alliance alliance, Telemetry telemetry) {
        this.hardware = new OdometricalMecanumHardwareMap(hwMap);
        this.alliance = alliance;
        this.telemetry = telemetry;
        this.elapsedTime = new ElapsedTime();
        this.locationInfo = new LocationInfo();
    }

    /**
     * Creates an instance of an autonomous robot manager.
     * @param hwMap FTC hardware map
     * @param alliance Alliance Enum
     * @param telemetry Logging
     * @param elapsedTime Game Timer
     */
    public AutonomousRobot(HardwareMap hwMap, Alliance alliance, Telemetry telemetry, ElapsedTime elapsedTime) {
        this(hwMap, alliance, telemetry);
        this.elapsedTime = elapsedTime;
    }

    /**
     * Initializes the robot (specifically Vuforia).
     */
    public void init() {
        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardware.internalHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.internalHardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = hardware.webcamName;
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
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
        final float CAMERA_FORWARD_DISPLACEMENT  = -10f * mmPerInch;   // eg: Camera is 0 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 9.75f * mmPerInch;   // eg: Camera is 6.625 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0f * mmPerInch;     // eg: Camera is ON the robot's center line

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
     * Scans the area for Vuforia trackables and saves them in a list.
     * If a trackable is found then the robot will update its position using the trackable.
     * Use this in the game loop to get the most up-to-date information from Vuforia.
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
                    if (trackable.getName().equals("Stone Target")) locationInfo.setLastRobotLocationFromSkystone(robotLocationTransform);
                    else locationInfo.setLastRobotLocation(robotLocationTransform);
                }
            }
        }
    }

    /**
     * Scans for x seconds.
     * @param waitTime seconds to scan
     */
    public void scanWait(int waitTime) {
        double initialTime = System.currentTimeMillis();
        while (!this.isTrackableVisible() && System.currentTimeMillis() < initialTime + (waitTime * 1000)) this.scan();
    }

    /**
     * Waits for x seconds.
     * This should be used for servo movements
     * @param waitTime seconds to wait
     */
    public void wait(double waitTime) {
        double initialTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < initialTime + (waitTime * 1000));
    }

    /**
     * Checks if the robot knows its approximate location.
     */
    public boolean isLocationKnown() {
        return locationInfo.getRobotLocation() != null;
    }

    /**
     * Checks if there is any visible trackables.
     */
    public boolean isTrackableVisible() {
        return visibleTrackables.size() > 0;
    }

    /**
     * Checks if there is any visible Skystones.
     */
    public boolean isSkystoneVisible() {
        return this.getVisibleTrackable("Stone Target") != null;
    }

    /**
     * Retrieves a tracked Skystone.
     */
    public VuforiaTrackable getTrackedSkystone() {
        return this.getVisibleTrackable("Stone Target");
    }

    /**
     * Stringifies visible targets.
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
     * Retrieves the last certain position of the robot (only works if there is a visible trackable).
     * @return current position on the FTC field
     */
    public Point3D getLastPosition() {
        return locationInfo.getLastRobotLocation();
    }

    /**
     * Retrieves the possible position of the robot (based off of tracking and motor encoders).
     * @return current position on the FTC field
     */
    public Point3D getPosition() {
        return locationInfo.getRobotLocation();
    }

    /**
     * Retrieves the current position of the robot (only works if there is a visible Skystone).
     * @return current position relative to the Skystone
     */
    public Point3D getPositionFromSkystone() {
        return locationInfo.getLastRobotLocationFromSkystone();
    }

    /**
     * Retrieves the orientation of the robot in a 2D space (heading/yaw).
     * @return current heading in degrees
     */
    public double getOrientation2D() {
        return this.getOrientation().thirdAngle;
    }

    /**
     * Retrieves the orientation of the robot in a 3D space.
     * @return current orientation in degrees
     */
    public Orientation getOrientation() {
        return Orientation.getOrientation(locationInfo.getRobotLocationMatrix(), EXTRINSIC, AxesOrder.XYZ, DEGREES);
    }

    /**
     * Retrieves the orientation of the robot according to the IMU.
     * The orientation is based off the robot (NOT the FTC field).
     * @return current orientation according to the IMU (in degrees)
     */
    public Orientation getOrientationIMU() {
        return hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
    }

    /**
     * Retrieves the orientation of the robot in a 2D space (heading/yaw) according to the IMU.
     * @return current heading in degrees
     */
    public double getOrientation2DIMU() {
        return AngleUnit.DEGREES.normalize(this.getOrientationIMU().firstAngle);
    }

    /**
     * Turns the robot by an angle.
     * This blocks the current thread.
     * @param angle angle to turn to in degrees
     */
    public void turn(double angle) {
        telemetry.addData("TURN EXECUTING", "Angle: %.2f degrees", angle);
        telemetry.update();

        hardware.drivetrain.setVelocity(0.1);
        hardware.drivetrain.setTargetHeading(Math.toRadians(angle));
        hardware.drivetrain.rotate();
        //if (angle - 90 == 90) turn(90);
    }

    /**
     * Checks if a trackable with a certain name is visible.
     * @param name trackable name identifier
     * @return Trackable information if visible (else null)
     */
    public VuforiaTrackable getVisibleTrackable(String name) {
        for (VuforiaTrackable trackable : visibleTrackables) {
            if (trackable.getName().equals(name)) return trackable;
        }
        return null;
    }

    /**
     * Calculates the course for the robot to arrive a point in a 2D space.
     * @param object Point3D
     * @return required course to arrive at a point in radians
     */
    public double getCourseFromRobot(Point object) {
        return this.getCourse(this.getPosition(), object);
    }

    /**
     * Calculates the course between a "robot" point and an object.
     * @param robot point of reference
     * @param object object to get course to
     * @return required course to arrive at a point in radians
     */
    public double getCourse(Point robot, Point object) {
        double c = -Math.atan2(object.y - robot.y, object.x - robot.x) - Math.PI / 2;
        if (c < -Math.PI) return Math.abs(c) - Math.PI;
        return c;

        //-Math.atan2(object.y - robot.y, object.x - robot.x);
        /*if (!Helpers.isTriangle(robot.calculateTriangleSides(object)) {
            double c = Math.atan2(object.y - robot.y, object.x - robot.x) - Math.PI / 2;
            if (c == Math.PI || c == -Math.PI) c = 0;
            return c;
        }

        return Math.atan2(object.y - robot.y, object.x - robot.x);*/
    }

    /**
     * Calculates the course for the robot to arrive a point in a 3D space.
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
     * Calculates the distance between a point on the field and the robot.
     * @param object Point3D
     * @return distance between point and robot center
     */
    public double getDistanceFromRobot(Point object) {
        return this.getPosition().distance(object);
    }

    /**
     * Calculates the distance between a point on the field and a supposed robot point.
     * @param robot supposed robot point
     * @param object object point
     * @return distance between the point and "robot"
     */
    public double getDistance(Point robot, Point object) {
        double distanceFromRobotCenter = robot.distance(object);
        return distanceFromRobotCenter - 9; // this is because the distance is calculated from the center of the robot. The camera is ~9 inches in front
    }

    /**
     * Automatically moves the robot under the bridge.
     */
    public void parkUnderBridge() {
        Point3D robotPosition = this.getPosition();
        if (alliance == Alliance.BLUE) {
            this.moveToPoint(new Point(robotPosition.x, blueBridge.getCornerPoint2().y + 18), 1);
            this.moveToPoint(new Point(blueBridge.getMiddleX(), robotPosition.y), 1);
        } else if (alliance == Alliance.RED) {
            this.moveToPoint(new Point(robotPosition.x, redBridge.getCornerPoint2().y - 18), 1);
            this.moveToPoint(new Point(redBridge.getMiddleX(), robotPosition.y), 1);
        }
    }

    /**
     * Automatically repositions the foundation to the Building Site.
     * This assumes the foundation is in its initial position.
     */
    public void repositionFoundation() {
        Point3D robotPosition = this.getPosition();
        if (alliance == Alliance.BLUE) {
            this.moveToPoint(blueFoundationMoveLocation, 0.5, new OrientationInfo(90, 0.7));
            // @TODO This is where we would move the servos
            this.moveToPoint(blueFoundationFinalLocation, 0.5);
        } else if (alliance == Alliance.RED) {
            this.moveToPoint(redFoundationMoveLocation, 0.5, new OrientationInfo(-90, 0.7));
            // This is where we would move the servos
            this.moveToPoint(redFoundationFinalLocation, 0.5);
        }
    }

    /**
     * Attempts to use a possible robot position to move to the scanning location for Skystone pickup.
     */
    public void gotoLoadingZone() {
        if (alliance == Alliance.BLUE) this.moveToPoint(blueLoadingScanLocation, 0.3, new OrientationInfo(180, 0.7));
        else if (alliance == Alliance.RED) this.moveToPoint(redLoadingScanLocation, 0.3, new OrientationInfo(-180, 0.7));
    }

    /**
     * Moves the robot using a course, velocity, rotation, and distance.
     * This adjusts some values, not including rotation.
     * @param course Angle (in degrees) of movement
     * @param velocity New velocity (-1 to 1)
     * @param rotation Double (between -1 and 1) -1 - clockwise 0 - no rotation 1 - counterclockwise
     * @param distance Distance (in inches) to execute this movement
     */
    public void simpleMove(double course, double velocity, double rotation, double distance) {
        hardware.drivetrain.setCourse(Math.toRadians(course));
        hardware.drivetrain.setRotation(rotation);
        hardware.drivetrain.setVelocity(velocity);
        hardware.drivetrain.setTargetPosition(distance * hardware.motorTicksPerInch);
        hardware.drivetrain.position();
    }

    /**
     * Moves the robot using a course, velocity, rotation, and distance.
     * @param course Angle (in degrees) of movement
     * @param velocity New velocity (-1 to 1)
     * @param orientationInfo Angle (in degrees) of orientation relative to current orientation
     * @param distance Distance (in inches) to execute this movement
     */
    public void move(double course, double velocity, OrientationInfo orientationInfo, double distance) {
        telemetry.addData("MOVE EXECUTING", "Course: %.2f degrees\nVelocity: %.2f\nOrientation: %s\nDistance: %.2f inches",
                course, velocity,
                orientationInfo != null ? "Angle: " + orientationInfo.angle + ", Rotation: " + orientationInfo.rotation : "Not Available",
                distance
        );
        telemetry.update();

        hardware.drivetrain.setCourse(Math.toRadians(course));
        hardware.drivetrain.setVelocity(velocity);
        hardware.drivetrain.setTargetPosition(distance * hardware.motorTicksPerInch);

        double startMillis = elapsedTime.milliseconds();
        ArrayList<Double> xVelocities = new ArrayList<>();
        ArrayList<Double> yVelocities = new ArrayList<>();

        if (orientationInfo != null) {
            double initialOrientation = this.getOrientation2DIMU();

            while (hardware.drivetrain.isPositioning()) {
                if (this.getOrientation2DIMU() - initialOrientation < orientationInfo.angle)
                    hardware.drivetrain.setRotation(orientationInfo.rotation);
            }

            // In case the robot did not finish turning by the time it reached its destination
            if (this.getOrientation2D() - initialOrientation < orientationInfo.angle)
                this.turn(orientationInfo.angle - (this.getOrientation2D() - initialOrientation));
        } else {
            while (hardware.drivetrain.isPositioning()) {
                Velocity gyroVelocity = hardware.imu.getVelocity();
                xVelocities.add(gyroVelocity.xVeloc);
                yVelocities.add(gyroVelocity.yVeloc);
            }
        }

        if (this.isLocationKnown()) {
            locationInfo.translateRobotLocation(this.getOrientation2D(), distance);

            /*
                REV IMU Version
                locationInfo.translateRobotLocation(Helpers.averageArrayList(xVelocities) * (elapsedTime.milliseconds() - startMillis), Helpers.averageArrayList(yVelocities) * (elapsedTime.milliseconds() - startMillis), 0.0);
             */
        }
    }

    /**
     * Moves the robot to a point.
     * @param point point to move to
     * @param velocity New velocity
     * @see #move(double, double, OrientationInfo, double)
     */
    public void moveToPoint(Point point, double velocity) {
        this.moveToPoint(point, velocity, null);
    }

    /**
     * Moves the robot to a point with orientation info.
     * @param point point to move to
     * @param velocity New velocity
     * @param orientationInfo The orientation to be at when moving to the point
     */
    public void moveToPoint(Point point, double velocity, OrientationInfo orientationInfo) {
        this.move(this.getCourseFromRobot(point), velocity, orientationInfo, this.getDistanceFromRobot(point));
    }

    /**
     * Moves the robot to a point.
     * @param robot supposed robot point
     * @param point point to move to
     * @param velocity New velocity
     */
    public void moveToPoint(Point robot, Point point, double velocity) {
        this.move(this.getCourse(robot, point), velocity, null, this.getDistance(robot, point));
    }

    /**
     * Corrects the course depending on the Alliance.
     * @param course The course assuming the movement is based off the Blue Alliance
     * @return New course depending on alliance
     */
    public double correctMovement(double course) {
        if (alliance == Alliance.RED) return -course;
        return course;
    }

    /**
     * Prints general telemetry for debugging.
     */
    public void printTelemetry() {
        telemetry.addData("Visible Target(s)", this.stringifyVisibleTargets());

        if (this.isLocationKnown()) {
            // express position (translation) of robot in inches.
            Point3D position = this.getPosition();
            telemetry.addData("Position (inch)", "{X, Y, Z} = %.1f, %.1f, %.1f", position.x, position.y, position.z);

            // express the orientation of the robot in degrees.
            Orientation orientation = this.getOrientation();
            telemetry.addData("Orientation (deg)", "{Heading, Roll, Pitch} = %.0f, %.0f, %.0f", orientation.thirdAngle, orientation.firstAngle, orientation.secondAngle);
        } else {
            telemetry.addData("Visible Target", "None");
        }
        telemetry.update();
    }
}