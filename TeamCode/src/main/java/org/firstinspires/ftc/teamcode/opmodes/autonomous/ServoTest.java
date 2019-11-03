package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.LatchSystem;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@Autonomous(name = "ServoTest", group="Autonomous")
public class ServoTest extends BaseStateMachine {

    public enum State {
        STATE_INITAL,
        STATE_LATCH,
        STATE_UNLATCH,
        STATE_REST
    }

    protected State mCurrentState;

    public void init() {
        super.init();
        newState(State.STATE_INITAL);
    }

    public void newState(State newState) {
        mCurrentState = newState;
    }

    @Override
    public void loop() {
        switch(mCurrentState) {
            case STATE_INITAL:
                latchSystem.unlatch();
                sleep(1000);
                newState(State.STATE_LATCH);
                break;
            case STATE_LATCH:
                latchSystem.latch();
                sleep(1000);
                newState(State.STATE_UNLATCH);
                break;
            case STATE_UNLATCH:
                latchSystem.unlatch();
                sleep(1000);
                newState(State.STATE_REST);
                break;
            case STATE_REST:
                sleep(1000);
                break;
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
