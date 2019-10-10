package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;

@Autonomous(name = "VuforiaTest", group = "Concept")
public class VuforiaTestStateMachine extends BaseStateMachine {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        switch (mCurrentState) {
            case STATE_INITIAL:
                // Initialize
                break;

            case STATE_FIND_SKYSTONE:
                if(vuforia.isTargetVisible(stoneTarget)){
                    newState(State.STATE_DELIVER_STONE);
                }
                break;
            case DRIVE_TO_FOUNDATION_TARGET:
                // Engage drive
                driveSystem.driveToPositionInches(12, DriveSystem.Direction.FORWARD, 1);

                // Switch camera on another thread
                if(currentCamera == Vuforia.CameraChoice.PHONE_BACK){
                    Thread t = new Thread(new ChangeCamera());
                }
                newState(State.STATE_DELIVER_STONE);
                break;
            case STATE_CAMERA_SWITCHED:
                if (!currentCamera.equals(Vuforia.CameraChoice.PHONE_FRONT)) {
                    newState(State.STATE_CAMERA_SWITCHED);
                } else {
                    newState(State.STATE_DELIVER_STONE);
                    wallTarget = targetsSkyStone.get(8);
                    wallTarget.setName("Wall Target");
                }
                break;
            case STATE_DELIVER_STONE:

        }
    }

    public class ChangeCamera implements Runnable{
        public void run(){
            currentCamera = Vuforia.CameraChoice.PHONE_FRONT;
            vuforia.close();
            vuforia = new Vuforia(hardwareMap, currentCamera);
        }

    }
}

