import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
<<<<<<< HEAD:ftc12069/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/MecanumAutonomous.java
import org.firstinspires.ftc.teamcode.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robotlib.information.OrientationInfo;
import org.firstinspires.ftc.teamcode.robotlib.navigation.Point3D;
import org.firstinspires.ftc.teamcode.robotlib.state.Alliance;
=======
import org.firstinspires.ftc.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.robotlib.navigation.Point3D;
import org.firstinspires.ftc.robotlib.state.Alliance;
>>>>>>> 52de9d979852bc2205f48d20d61b58a3d3f12e19:ftc12069/src/main/java/org/firstinspires/ftc/opmodes/auto/MecanumAutonomous.java

public class MecanumAutonomous {
    private Alliance alliance;
@ -57,13 +63,19 @@ public class MecanumAutonomous {
        this.alliance = alliance;
    }

    /**
     * Ran before the game starts
     */
    void init() {
        // Initialize robot
        telemetry.addData("Status", "Initialized");
        robot = new AutonomousRobot(this.hardwareMap, VUFORIA_KEY, alliance);
        robot = new AutonomousRobot(this.hardwareMap, VUFORIA_KEY, alliance, telemetry);
        robot.init();
    }

    /**
     * Ran after the game starts and before the game loop begins
     */
    void start() {
        elapsedTime.reset();

@ -71,43 +83,60 @@ public class MecanumAutonomous {
        robot.trackables.activate();
    }

    /**
     * Ran once the game has ended
     */
    void end() {
        // Disable Tracking when we are done
        robot.trackables.deactivate();
    }

    /**
     * Game Loop Method (runs until stop is requested)
     * @return true - keep looping | false - stop looping
     */
    boolean loop() {
        if (elapsedTime.seconds() > 25) {
        /*if (elapsedTime.seconds() > 25) {
            robot.parkUnderBridge();
            return false;
        }
        }*/

        robot.scan();

        telemetry.addData("Elapsed Time", elapsedTime.seconds() + " seconds");
        // Provide feedback as to where the robot is located (if we know).
        if (robot.isTargetVisible()) {
            // express position (translation) of robot in inches.
            Point3D position = robot.getPosition();
            telemetry.addData("Position (inch)", "{X, Y, Z} = %.1f, %.1f, %.1f", position.x, position.y, position.z);

            // express the orientation of the robot in degrees.
            Orientation orientation = robot.getOrientation();
            telemetry.addData("Orientation (deg)", "{Heading, Roll, Pitch} = %.0f, %.0f, %.0f", orientation.thirdAngle, orientation.firstAngle, orientation.secondAngle);
        if (robot.isTrackableVisible()) {
            if (robot.isLocationKnown()) {
                // express position (translation) of robot in inches.
                Point3D position = robot.getPosition();
                telemetry.addData("Position (inch)", "{X, Y, Z} = %.1f, %.1f, %.1f", position.x, position.y, position.z);

                // express the orientation of the robot in degrees.
                Orientation orientation = robot.getOrientation();
                telemetry.addData("Orientation (deg)", "{Heading, Roll, Pitch} = %.0f, %.0f, %.0f", orientation.thirdAngle, orientation.firstAngle, orientation.secondAngle);
            }

            telemetry.addData("Visible Target(s)", robot.stringifyVisibleTargets());

            // move to stone if visible
            VuforiaTrackable trackedStone = robot.getVisibleTrackable("Stone Target");
            if (trackedStone != null) {
                Point3D positionFromSkystone = robot.getPositionFromSkystone();
                Point3D stonePoint3D = new Point3D(trackedStone.getLocation());
                robot.simpleMove(robot.getCourseFromRobot(stonePoint3D), 1, 0, robot.getDistanceFromRobot(stonePoint3D));
                robot.turn(90, 0.5);
                robot.move(robot.getCourseFromRobot(stonePoint3D), 1, new OrientationInfo(145, 0.3), robot.getDistanceFromRobot(stonePoint3D));
                telemetry.addData("Position relative to Skystone", positionFromSkystone);
                robot.simpleMove(robot.getCourse(positionFromSkystone, stonePoint3D), 1, 0, robot.getDistance(positionFromSkystone, stonePoint3D));
            }
        } else {
            telemetry.addData("Visible Target", "None");
        }

        /*
            Example Moving/Turning
            robot.simpleMove(robot.getCourseFromRobot(stonePoint3D), 1, 0, robot.getDistanceFromRobot(stonePoint3D));
            robot.turn(90, 0.5);
            robot.move(robot.getCourseFromRobot(stonePoint3D), 1, new OrientationInfo(145, 0.3), robot.getDistanceFromRobot(stonePoint3D));
         */

        telemetry.update();
        return true;
    }