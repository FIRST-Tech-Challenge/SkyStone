package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.State;

/**
 * 2019.10.11
 * Autonomous class!
 * Created by Athena Z.
 */

public class Autonomous extends OpMode {

    private enum AutoState {INITIAL, NEXT_STEP, FINAL}

    AutoState state = AutoState.INITIAL;

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        if (state == AutoState.INITIAL) {

        } else if (state == AutoState.NEXT_STEP) {

        } else if (state == AutoState.FINAL) {

        }
        switch (state) {
            case INITIAL:
                break;
            case NEXT_STEP:
                break;
            case FINAL:
                break;
        }
    }
}
