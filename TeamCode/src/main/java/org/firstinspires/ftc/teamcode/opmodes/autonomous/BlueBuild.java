package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseStateMachine;

@Autonomous(name = "BlueBuild", group="Autonomous")
public class BlueBuild extends BaseStateMachine {
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
    }

    public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private Vuforia vuforia;
    private VuforiaTrackable skystone;
    private DriveSystem driveSystem;
    private static final float mmPerInch = 25.4f;

    protected State mCurrentState;    // Current State Machine State.

    @Override
    public void init() {
        super.init();
        skystone = vuforia.targetsSkyStone.get(0);
        this.msStuckDetectLoop = 15000;
        newState(State.STATE_INITIAL);
    }


    @Override
    public void loop() {
        switch (mCurrentState) {
            case STATE_INITIAL:
                // Initialize
                newState(State.STATE_FIND_SKYSTONE);
                break;

            case STATE_FIND_SKYSTONE:
                // Strafe towards line
                // Identify SkyStone
                // If we can't see it after 3 feet, start driving  until we do
                telemetry.addData("State", "STATE_FIND_SKYSTONE");

                if (!vuforia.isTargetVisible(skystone)) {
                    driveSystem.setMotorPower(0.2);
                } else {
                    driveSystem.setMotorPower(0);
                    newState(State.STATE_GRAB_STONE);
                }
                telemetry.update();

                break;

            case STATE_GRAB_STONE:
                // Grab the stone and slurp it into the machine
                telemetry.addData("State", "STATE_GRAB_STONE");

                Orientation rotation = vuforia.getRobotHeading();
                driveSystem.turn(rotation.thirdAngle - 90, 0.8);
                if (vuforia.isTargetVisible(skystone)) {
                    VectorF translation = vuforia.getRobotPosition();
                   // driveSystem.driveToPositionInches(translation.get(1) / mmPerInch, DriveSystem.Direction.FORWARD, 0.5);
                }
                driveSystem.turn(-90, 1.0);
              //  driveSystem.driveToPositionInches(24, DriveSystem.Direction.FORWARD, 1.0);
                telemetry.update();

                newState(State.STATE_DELIVER_STONE);
                break;

            case STATE_DELIVER_STONE:
                telemetry.addData("State", "STATE_DELIVER_STONE");

                // Drive with stone to the foundation
                // Go under bridge
                telemetry.update();
//                    newState(State.STATE_DEPOSIT_STONE);
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

    // Stop
    @Override
    public void stop() {
    }

    public void newState(State newState) {
        // Restarts the state clock as well as the state
        mStateTime.reset();
        mCurrentState = newState;
    }

}

