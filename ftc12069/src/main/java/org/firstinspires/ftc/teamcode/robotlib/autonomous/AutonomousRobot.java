import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@ -29,11 +30,18 @@ import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocaliz
public class AutonomousRobot {
    private String vuforiaKey;
    public MecanumHardwareMap hardware;
<<<<<<< HEAD:ftc12069/src/main/java/org/firstinspires/ftc/teamcode/robotlib/autonomous/AutonomousRobot.java
    private Object alliance;
=======
    private Alliance alliance;
    private Telemetry telemetry;
>>>>>>> 52de9d979852bc2205f48d20d61b58a3d3f12e19:ftc12069/src/main/java/org/firstinspires/ftc/robotlib/autonomous/AutonomousRobot.java

    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private OpenGLMatrix lastLocation = null;
    private OpenGLMatrix lastRobotLocation = null;
    private long lastRobotLocationTime;
    private OpenGLMatrix lastRobotLocationFromSkystone;

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;
@ -72,11 +80,17 @@ public class AutonomousRobot {
     * @param hwMap FTC hardware map
     * @param vuforiaKey Vuforia Key
     * @param alliance Alliance Enum
     * @param telemetry Logging
     */
<<<<<<< HEAD:ftc12069/src/main/java/org/firstinspires/ftc/teamcode/robotlib/autonomous/AutonomousRobot.java
    public AutonomousRobot(HardwareMap hwMap, String vuforiaKey, Object alliance) {
=======
    public AutonomousRobot(HardwareMap hwMap, String vuforiaKey, Alliance alliance, Telemetry telemetry) {
>>>>>>> 52de9d979852bc2205f48d20d61b58a3d3f12e19:ftc12069/src/main/java/org/firstinspires/ftc/robotlib/autonomous/AutonomousRobot.java
        this.hardware = new MecanumHardwareMap(hwMap);
        this.vuforiaKey = vuforiaKey;
        this.alliance = alliance;
        this.telemetry = telemetry;
    }

    /**
@ -89,7 +103,7 @@ public class AutonomousRobot {
         */
        int cameraMonitorViewId = hardware.internalHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.internalHardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = hardware.webcamName;
        //parameters.cameraName = hardware.webcamName;
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = vuforiaKey;
@ -224,6 +238,8 @@ public class AutonomousRobot {
        for (VuforiaTrackable trackable : trackablesList) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        visibleTrackables = new ArrayList<>();
    }

    /**
@ -232,8 +248,8 @@ public class AutonomousRobot {
     * Use this in the game loop to get the most up-to-date information from Vuforia
     */
    public void scan() {
        visibleTrackables.clear();
        for (VuforiaTrackable trackable : trackablesList) {
            visibleTrackables.clear();
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                visibleTrackables.add(trackable);

@ -241,16 +257,27 @@ public class AutonomousRobot {
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
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
     * Checks if there is any visible targets
     * Checks if the robot knows a location (might not be updated)
     */
    public boolean isTargetVisible() {
    public boolean isLocationKnown() {
        return lastRobotLocation != null;
    }

    /**
     * Checks if there is any visible trackables
     */
    public boolean isTrackableVisible() {
        return visibleTrackables.size() > 0;
    }

@ -271,9 +298,21 @@ public class AutonomousRobot {
     * @return current position on the FTC field
     */
    public Point3D getPosition() {
        if (!isTargetVisible()) return null;
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
        VectorF translation = lastLocation.getTranslation();
        VectorF translation = lastRobotLocationFromSkystone.getTranslation();
        return new Point3D(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
    }

@ -282,7 +321,7 @@ public class AutonomousRobot {
     * @return current heading
     */
    public double getOrientation2D() {
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
        return Orientation.getOrientation(lastRobotLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
    }

    /**
@ -291,7 +330,7 @@ public class AutonomousRobot {
     */
    public Orientation getOrientation() {
        // imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        return Orientation.getOrientation(lastRobotLocation, EXTRINSIC, XYZ, DEGREES);
    }

    /**
@ -331,6 +370,16 @@ public class AutonomousRobot {
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
@ -353,6 +402,16 @@ public class AutonomousRobot {
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
@ -415,4 +474,14 @@ public class AutonomousRobot {
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