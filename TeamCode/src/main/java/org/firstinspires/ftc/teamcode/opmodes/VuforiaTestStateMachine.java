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
                telemetry.addLine("State: Initial");
                telemetry.update();

                newState(State.STATE_CAMERA_SWITCHED);
                break;

            case STATE_FIND_SKYSTONE:
                telemetry.addLine("State: Find_Skystone");
                telemetry.update();

                if(vuforia.isTargetVisible(stoneTarget)){
                    newState(State.STATE_DELIVER_STONE);
                }
                break;
            case DRIVE_TO_FOUNDATION_TARGET:
                telemetry.addLine("State: Drive_To_Foundation_Target");
                telemetry.update();

                driveSystem.driveToPositionInches(24, DriveSystem.Direction.FORWARD, 1);
                break;

            case STATE_CAMERA_SWITCHED:
                telemetry.addLine("State: Camera_Switched");
                telemetry.update();

                // Switch camera on another thread
                Thread t = new Thread(new ChangeCamera());
                t.start();
                newState(State.DRIVE_TO_FOUNDATION_TARGET);

                //ELIMINATED FOR TESTING PURPOSES
                /*if (!currentCamera.equals(Vuforia.CameraChoice.PHONE_FRONT)) {
                    newState(State.STATE_CAMERA_SWITCHED);
                } else {
                    Thread t = new Thread(new ChangeCamera());
                    t.start();

                    wallTarget = targetsSkyStone.get(8);
                    wallTarget.setName("Wall Target");
                }*/
                break;
            case STATE_DELIVER_STONE:
                telemetry.addLine("State: Deliver_Stone");
                telemetry.update();

                break;
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

