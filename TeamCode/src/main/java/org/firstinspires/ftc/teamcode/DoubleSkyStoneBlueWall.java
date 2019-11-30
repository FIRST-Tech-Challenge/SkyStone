package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Double SkyStone Blue Wall", group = "Exercises")

public class DoubleSkyStoneBlueWall extends DoubleSkyStoneRedWall {
    protected TensorFlowBot robot = new TensorFlowBot(this);

    int direction_forward, direction_backward;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        direction_forward = robot.DIRECTION_BACKWARD;
        direction_backward = robot.DIRECTION_FORWARD;
        super.runOpMode();
    }
}





