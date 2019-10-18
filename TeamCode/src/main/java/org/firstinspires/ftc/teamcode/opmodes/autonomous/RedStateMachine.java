package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedStateMachine", group="Autonomous")
public class RedStateMachine extends BaseStateMachine {

    public void init() {
        super.init(Team.RED);
    }
}

