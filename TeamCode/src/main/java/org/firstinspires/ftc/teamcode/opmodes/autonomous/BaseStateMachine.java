package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

public abstract class BaseStateMachine extends BaseOpMode {
    public enum State {
        STATE_INITIAL,
        STATE_FIND_SKYSTONE,
        STATE_DELIVER_STONE,
        STATE_GRAB_STONE,
        STATE_FIND_STONE,
        STATE_PARK_AT_LINE,
        STATE_DEPOSIT_STONE,
        STATE_DRAG_FOUNDATION,
        STATE_RETURN,
        GRAB_STONE_DEAD_RECKONING,
        LOGGING
    }

    public enum Team {
        RED, BLUE
    }

    private final static String TAG = "BaseStateMachine";
    private ColorSensor colorSensor;
    protected State mCurrentState;    // Current State Machine State.
    protected ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    protected DistanceSensor distanceFront;
    private Team currentTeam;
    VuforiaTrackable skystone;

    public void init(Team team) {
        super.init();
        this.msStuckDetectInit = 20000;
        this.msStuckDetectInitLoop = 20000;
        // TODO: Get webcame choice for competition
        super.setCamera(CameraChoice.PHONE_BACK);
//        rearPerimeter = vuforia.targetsSkyStone.get(team == Team.RED ? 12 : 11);
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        this.msStuckDetectLoop = 30000;
        newState(State.LOGGING);
        skystone = vuforia.targetsSkyStone.get(0);
        currentTeam = team;
    }

    private DriveSystem.Direction direction;
    @Override
    public void loop() {
        switch (mCurrentState) {
            case LOGGING:
                telemetry.addData("DistanceFront", distanceFront.getDistance(DistanceUnit.INCH));
                telemetry.addData("Color Blue", colorSensor.blue());
                telemetry.addData("Color Red", colorSensor.red());
                telemetry.addData("Color Green", colorSensor.green());
                telemetry.addData("Color Alpha", colorSensor.alpha());
                telemetry.addData("Color Hue", colorSensor.argb());
                telemetry.update();
                break;
            case STATE_INITIAL:
                // Initialize
                // Drive 0.5m (1 tile) to the left
                direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                while (!driveSystem.driveToPosition(400, direction, 0.8) && !isStopRequested()) {}
                newState(State.STATE_FIND_SKYSTONE);
                mStateTime.reset();
                break;

            case STATE_FIND_SKYSTONE:
                // Strafe towards line
                // Identify SkyStone
                telemetry.addData("State", "STATE_FIND_SKYSTONE");
                if (mStateTime.seconds() > 5) {
                    // TODO: Unable to detect stone after 10 seconds. Use dead reckoning
                    // TODO: Make new state for this. Currently just set to log
                    newState(State.GRAB_STONE_DEAD_RECKONING);
                    break;
                }
                Log.d(TAG, mCurrentState.toString());
                if (vuforia.isTargetVisible(skystone)) {
                    Log.d(TAG, "Got here");
                    newState(State.STATE_GRAB_STONE);
                    break;
                }
                telemetry.update();
                break;
            case STATE_GRAB_STONE:
                Log.d(TAG, mCurrentState.toString());
                // Grab the stone and slurp it into the machine
                telemetry.addData("State", "STATE_GRAB_STONE");
                telemetry.update();

                if (vuforia.isTargetVisible(skystone)) {
                    Log.d(TAG, "inside grab stone loop");
                    VectorF translation = vuforia.getRobotPosition();
                    // Strafe to align with skystone
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.FORWARD : DriveSystem.Direction.BACKWARD;
                    while (!driveSystem.driveToPosition((int) translation.get(1), direction, 0.5) && !isStopRequested()) {}

                    // Drive up to the skystone
                    double distance = distanceFront.getDistance(DistanceUnit.MM);
                    distance -= 10;
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                    while (!driveSystem.driveToPosition((int) distance, direction, 0.8) && !isStopRequested()) {};

                    // Offset from skystone
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.FORWARD : DriveSystem.Direction.BACKWARD;
                    while (!driveSystem.driveToPosition(900, direction, 0.5) && !isStopRequested()) {}
                    // Shove into the other stones
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                    while (!driveSystem.driveToPosition(1400, direction, 0.5) && !isStopRequested()) {}
                    // Drive into skystone
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.BACKWARD : DriveSystem.Direction.FORWARD;
                    while (!driveSystem.driveToPosition(500, direction, 0.5) && !isStopRequested()) {}

                    // Move away with skystone (prepare for next state)
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.LEFT : DriveSystem.Direction.RIGHT;
                    while (!driveSystem.driveToPosition(1400, DriveSystem.Direction.RIGHT, 0.8) && !isStopRequested()) {};
                    double heading = driveSystem.imuSystem.getHeading();
                    // I think it is getting stuck here. The purpose is to align the robot with the
                    // audience such it moves straight
                    while (!driveSystem.turn(-heading, 0.5) && !isStopRequested()) {};
                }

                telemetry.update();
                newState(State.STATE_PARK_AT_LINE);
                break;
            case GRAB_STONE_DEAD_RECKONING:
                telemetry.addData("State", "GRAB_STONE_DEAD_RECKONING");

                // Drive up to the skystone
                double distance = distanceFront.getDistance(DistanceUnit.MM);
                distance -= 10;
                direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                while (!driveSystem.driveToPosition((int) distance, direction, 0.8) && !isStopRequested()) {};

                // Offset from skystone
                direction = currentTeam == Team.RED ? DriveSystem.Direction.FORWARD : DriveSystem.Direction.BACKWARD;
                while (!driveSystem.driveToPosition(900, direction, 0.5) && !isStopRequested()) {}
                // Shove into the other stones
                direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                while (!driveSystem.driveToPosition(1400, direction, 0.5) && !isStopRequested()) {}
                // Drive into skystone
                direction = currentTeam == Team.RED ? DriveSystem.Direction.BACKWARD : DriveSystem.Direction.FORWARD;
                while (!driveSystem.driveToPosition(500, direction, 0.5) && !isStopRequested()) {}

                // Move away with skystone (prepare for next state)
                direction = currentTeam == Team.RED ? DriveSystem.Direction.LEFT : DriveSystem.Direction.RIGHT;
                while (!driveSystem.driveToPosition(1400, DriveSystem.Direction.RIGHT, 0.8) && !isStopRequested()) {};
                double heading = driveSystem.imuSystem.getHeading();
                // I think it is getting stuck here. The purpose is to align the robot with the
                // audience such it moves straight
                while (!driveSystem.turn(-heading, 0.5) && !isStopRequested()) {};
                telemetry.update();
                newState(State.STATE_PARK_AT_LINE);
                break;

            case STATE_DELIVER_STONE:
                telemetry.addData("State", "STATE_DELIVER_STONE");
                // TODO: Use distance sensor to detect distance from wall
                while (!driveSystem.driveToPosition(4000, DriveSystem.Direction.BACKWARD, 1.0)  && !isStopRequested()) {};
                newState(State.LOGGING);
                telemetry.update();
                break;

            case STATE_FIND_STONE:
                // Find a stone using TensorFlow
                newState(State.STATE_GRAB_STONE);
                break;

            case STATE_PARK_AT_LINE:
                // Find the line
                // Park
                break;

            case STATE_DEPOSIT_STONE:
                // Put stone down on foundation
                newState(State.STATE_RETURN);
                break;

            case STATE_DRAG_FOUNDATION:
                // Drag foundation out of box
                newState(State.STATE_PARK_AT_LINE);
                break;

            case STATE_RETURN:
                // Go back to block repository
                newState(State.STATE_FIND_SKYSTONE);
                break;
        }
    }

    public void newState(State newState) {
        // Restarts the state clock as well as the state
        mStateTime.reset();
        mCurrentState = newState;
    }

}

