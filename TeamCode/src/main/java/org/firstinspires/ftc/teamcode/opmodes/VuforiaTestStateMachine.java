package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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

            case STATE_DELIVER_STONE:
                // Engage drive

                // Switch camera
                if(currentCamera == Vuforia.CameraChoice.PHONE_BACK){
                    changeCamera();
                }

        }

    }

    // Stop
    @Override
    public void stop() {
    }

    public void changeCamera(){
        vuforia.close();
        currentCamera = Vuforia.CameraChoice.PHONE_FRONT;
        vuforia = new Vuforia(hardwareMap, currentCamera);
    }

public void newState(State newState) {
    mCurrentState = newState;
}



}

