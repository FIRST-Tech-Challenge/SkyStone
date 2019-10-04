package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;


public class BaseStateMachine extends OpMode {
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

        public ElapsedTime elapsedTime;   // Time into round.

        protected State mCurrentState;    // Current State Machine State.

        protected Vuforia.CameraChoice currentCamera;

        protected Vuforia vuforia;

        VuforiaTrackable stoneTarget;

        private DriveSystem driveSystem;

        @Override
        public void init() {
            elapsedTime = new ElapsedTime();
            currentCamera = Vuforia.CameraChoice.PHONE_BACK;
            vuforia = new Vuforia(hardwareMap, currentCamera);
            mCurrentState = State.STATE_FIND_SKYSTONE;
            VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
            stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
        }


        // @Override
        public void init_loop() {
        }

        @Override
        public void start() {

        }

        @Override
        public void loop() {
            switch (mCurrentState) {
                case STATE_INITIAL:
                    // Initialize
                    break;

                case STATE_FIND_SKYSTONE:
                    // Strafe towards line
                    // Identify SkyStone
                    // If we can't see it after 3 feet, start driving  until we do
                    newState(State.STATE_GRAB_STONE);
                    break;

                case STATE_GRAB_STONE:
                    // Grab the stone and slurp it into the machine
                    newState(State.STATE_DELIVER_STONE);
                    break;

                case STATE_DELIVER_STONE:
                    // Drive with stone to the foundation
                    // Go under bridge
                    newState(State.STATE_DEPOSIT_STONE);
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

