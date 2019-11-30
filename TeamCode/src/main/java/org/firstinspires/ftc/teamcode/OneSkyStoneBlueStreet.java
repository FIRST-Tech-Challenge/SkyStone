package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="One SkyStone Blue Street", group="Exercises")
public class OneSkyStoneBlueStreet extends OneSkyStoneRedStreet {
    @Override
    protected void setDirection(){
        direction_forward = robot.DIRECTION_FORWARD;
        direction_backward = robot.DIRECTION_BACKWARD;
    }
}
