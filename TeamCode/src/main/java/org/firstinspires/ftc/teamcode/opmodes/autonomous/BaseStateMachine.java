package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IntakeSystem;
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
        EJECT_STONE,
        LOGGING
    }

    public enum Team {
        RED, BLUE
    }

    private final static String TAG = "BaseStateMachine";
    private ColorSensor colorSensor;
    protected State mCurrentState;    // Current State Machine State.
    protected ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    protected DistanceSensor distanceSide;
    private Team currentTeam;
    VuforiaTrackable skystone;

    public void init(Team team) {
        super.init();
        this.msStuckDetectInit = 15000;
        this.msStuckDetectInitLoop = 15000;
        // TODO: Get webcam choice for competition
//        rearPerimeter = vuforia.targetsSkyStone.get(team == Team.RED ? 12 : 11);
        if (currentTeam == Team.RED) {
            distanceSide = hardwareMap.get(DistanceSensor.class, "FRONTRIGHTLIDAR");
//            super.setCamera(CameraChoice.WEBCAM1);
        } else {
            distanceSide = hardwareMap.get(DistanceSensor.class, "FRONTLEFTLIDAR");
//            super.setCamera(CameraChoice.WEBCAM2);
        }
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        this.msStuckDetectLoop = 30000;
        newState(State.STATE_INITIAL);
//        skystone = vuforia.targetsSkyStone.get(0);
        currentTeam = team;
    }

    private DriveSystem.Direction direction;
    @Override
    public void loop() {
        switch (mCurrentState) {
            case LOGGING:
                telemetry.addData("DistanceFront", distanceSide.getDistance(DistanceUnit.INCH));
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
                if (mStateTime.seconds() > 2) {
                    // TODO: Unable to detect stone after 2 seconds. Use dead reckoning
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
                    while (!driveSystem.driveToPosition((int) translation.get(1), DriveSystem.Direction.FORWARD, 0.5) && !isStopRequested()) {}

                    // Drive up to the skystone
                    double distance = distanceSide.getDistance(DistanceUnit.MM);
                    distance -= 10;
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                    while (!driveSystem.driveToPosition((int) distance, direction, 0.8) && !isStopRequested()) {};
                    // Offset from skystone
                    while (!driveSystem.driveToPosition(700, DriveSystem.Direction.FORWARD, 0.5) && !isStopRequested()) {}
                    // Shove into the other stones
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                    while (!driveSystem.driveToPosition(1525, direction, 0.5) && !isStopRequested()) {}
                    // Drive into skystone
                    while (!driveSystem.driveToPosition(500, DriveSystem.Direction.BACKWARD, 0.3) && !isStopRequested()) {
                        intakeSystem.suck();
                    }
                    intakeSystem.stop();
                    // Move away with skystone (prepare for next state)
                    direction = currentTeam == Team.RED ? DriveSystem.Direction.LEFT : DriveSystem.Direction.RIGHT;
                    while (!driveSystem.driveToPosition(1200, direction, 0.8) && !isStopRequested()) {};
                    double heading = driveSystem.imuSystem.getHeading();
                    // I think it is getting stuck here. The purpose is to align the robot with the
                    // audience such it moves straight
                    while (!driveSystem.turn(-heading + 190, 0.9) && !isStopRequested()) {};
                    telemetry.update();
                    newState(State.STATE_DELIVER_STONE);
                }

                telemetry.update();
                newState(State.STATE_DELIVER_STONE);
                break;
            case GRAB_STONE_DEAD_RECKONING:
                telemetry.addData("State", "GRAB_STONE_DEAD_RECKONING");

                // Drive up to the skystone
                double distance = distanceSide.getDistance(DistanceUnit.MM);
                distance -= 10;
                direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                while (!driveSystem.driveToPosition((int) distance, direction, 0.8) && !isStopRequested()) {};
                // Offset from skystone
                while (!driveSystem.driveToPosition(700, DriveSystem.Direction.FORWARD, 0.5) && !isStopRequested()) {}
                // Shove into the other stones
                direction = currentTeam == Team.RED ? DriveSystem.Direction.RIGHT : DriveSystem.Direction.LEFT;
                while (!driveSystem.driveToPosition(1525, direction, 0.5) && !isStopRequested()) {}
                // Drive into skystone
                while (!driveSystem.driveToPosition(500, DriveSystem.Direction.BACKWARD, 0.3) && !isStopRequested()) {
                    intakeSystem.suck();
                }
                intakeSystem.stop();
                // Move away with skystone (prepare for next state)
                direction = currentTeam == Team.RED ? DriveSystem.Direction.LEFT : DriveSystem.Direction.RIGHT;
                while (!driveSystem.driveToPosition(1200, direction, 0.8) && !isStopRequested()) {};
                double heading = driveSystem.imuSystem.getHeading();
                // I think it is getting stuck here. The purpose is to align the robot with the
                // audience such it moves straight
                while (!driveSystem.turn(-heading + 190, 0.9) && !isStopRequested()) {};
                telemetry.update();
                newState(State.STATE_DELIVER_STONE);
                break;

            case STATE_DELIVER_STONE:
                telemetry.addData("State", "STATE_DELIVER_STONE");
                while (!driveSystem.driveToPosition(2200, DriveSystem.Direction.BACKWARD, 1.0)  && !isStopRequested()) {};
                intakeSystem.unsuck();
                newState(State.EJECT_STONE);
                telemetry.update();
                break;

            case EJECT_STONE:
                if (mStateTime.milliseconds() >= 1000) {
                    intakeSystem.stop();
                    newState(State.STATE_PARK_AT_LINE);
                } else {
                    intakeSystem.stop();
                }
                break;

            case STATE_FIND_STONE:
                // Find a stone using TensorFlow
                newState(State.STATE_GRAB_STONE);
                break;

            case STATE_PARK_AT_LINE:
                // Find the line
                if (currentTeam == Team.BLUE) {
                    if (colorSensor.blue() > colorSensor.red() * 1.5) {
                        driveSystem.setMotorPower(0.0);
                        newState(State.LOGGING);
                        break;
                    }
                } else if (currentTeam == Team.RED) {
                    if (colorSensor.red() > colorSensor.blue() * 1.5) {
                        driveSystem.setMotorPower(0.0);
                        newState(State.LOGGING);
                        break;
                    }
                }
                driveSystem.drive(0.0f, 0.0f, -0.2f, 0.0f);
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

