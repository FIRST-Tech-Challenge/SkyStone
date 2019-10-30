package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueStateMachine", group="Autonomous")
public class BlueStateMachine extends BaseStateMachine {

    public void init() {
        super.init(Team.BLUE);
    }
}

