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
                newState(State.STATE_FIND_SKYSTONE);
                break;

            case STATE_FIND_SKYSTONE:
                // Strafe towards line
                // Identify SkyStone
                // If we can't see it after 3 feet, start driving  until we do
                if(driveSystem.driveToPositionInches(48, DriveSystem.Direction.RIGHT, .5)){
                    Log.d(TAG,"exiting loop");
                    newState(State.STATE_RETURN);
                }

                break;

            case STATE_RETURN:
                Log.d(TAG,"Target reached");
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

