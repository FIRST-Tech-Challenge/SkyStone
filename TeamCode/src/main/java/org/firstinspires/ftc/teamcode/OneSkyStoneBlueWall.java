package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="One SkyStone Blue Wall", group="Exercises")
public class OneSkyStoneBlueWall extends OneSkyStoneRedWall {
    @Override
    protected void setDirection(){
        direction_forward = robot.DIRECTION_FORWARD;
        direction_backward = robot.DIRECTION_BACKWARD;
    }
}