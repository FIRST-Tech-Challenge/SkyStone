package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@Autonomous(name = "TestDrive", group="Autonomous")
public class TestDrive extends BaseOpMode {
    public enum State {
        STATE_INITIAL,
        STATE_RIGHT,
        STATE_LEFT,
        STATE_FORWARD,
        STATE_BACKWARD,
        STATE_FINISHED,
    }

    public static final String TAG = "Test Drive";


    private static final float mmPerInch = 25.4f;

    protected State mCurrentState;    // Current State Machine State.

    @Override
    public void init() {
        super.init();
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
                if(driveSystem.driveToPosition(100, DriveSystem.Direction.FORWARD, .5)){
                    newState(State.STATE_BACKWARD);
                }
                break;
            case STATE_BACKWARD:
                if(driveSystem.driveToPosition(100, DriveSystem.Direction.BACKWARD, .5)){
                    newState(State.STATE_RIGHT);
                }
                break;
            case STATE_RIGHT:
                if(driveSystem.driveToPosition(100, DriveSystem.Direction.RIGHT, .5)){
                    newState(State.STATE_LEFT);
                }
                break;
            case STATE_LEFT:
                if(driveSystem.driveToPosition(100, DriveSystem.Direction.LEFT, .5)){
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
        mCurrentState = newState;
    }

}

