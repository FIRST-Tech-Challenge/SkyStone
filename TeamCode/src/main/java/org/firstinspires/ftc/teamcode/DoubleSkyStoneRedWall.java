package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Double SkyStone Red Wall", group = "Exercises")

public class DoubleSkyStoneRedWall extends DoubleSkyStoneBlueWall {
    protected TensorFlowBot robot = new TensorFlowBot(this);

    int direction_forward, direction_backward;

    @Override
    protected void setDirection(){
        direction_forward = robot.DIRECTION_BACKWARD;
        direction_backward = robot.DIRECTION_FORWARD;
    }
}
