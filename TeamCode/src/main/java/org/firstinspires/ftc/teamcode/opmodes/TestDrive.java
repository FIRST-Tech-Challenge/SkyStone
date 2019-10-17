package org.firstinspires.ftc.teamcode.opmodes;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;

@Autonomous(name = "TestDrive", group="Autonomous")
public class TestDrive extends BaseStateMachine {
    public enum State {
        STATE_INITIAL,
        STATE_RIGHT,
        STATE_LEFT,
        STATE_FORWARD,
        STATE_BACKWARD,
        STATE_FINISHED,
    }

    public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.

    public static final String TAG = "Test Drive";

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private Vuforia vuforia;
    private VuforiaTrackable skystone;
    private DriveSystem driveSystem;
    private static final float mmPerInch = 25.4f;

    protected State mCurrentState;    // Current State Machine State.

    @Override
    public void init() {
        super.init();
        driveSystem = super.driveSystem;
        mCurrentState = State.STATE_INITIAL;
    }


    @Override
    public void loop() {
        switch (mCurrentState) {
            case STATE_INITIAL:
                // Initialize
                newState(State.STATE_FORWARD);
                break;

            case STATE_FORWARD:
                if(driveSystem.driveToPositionInches(48, DriveSystem.Direction.FORWARD, .5)){
                    newState(State.STATE_BACKWARD);
                }
                break;
            case STATE_BACKWARD:
                if(driveSystem.driveToPositionInches(48, DriveSystem.Direction.BACKWARD, .5)){
                    newState(State.STATE_RIGHT);
                }
                break;
            case STATE_RIGHT:
                if(driveSystem.driveToPositionInches(48, DriveSystem.Direction.RIGHT, .5)){
                    newState(State.STATE_LEFT);
                }
                break;
            case STATE_LEFT:
                if(driveSystem.driveToPositionInches(48, DriveSystem.Direction.LEFT, .5)){
                    newState(State.STATE_FINISHED);
                }
                break;
            case STATE_FINISHED:

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

