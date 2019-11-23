package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Tryhard Blue Wall", group="Exercises")
public class TryhardBlueWall extends TryhardRedWall{
    @Override
    public void runOpMode() {
        direction_forward = robot.DIRECTION_BACKWARD;
        direction_backward = robot.DIRECTION_FORWARD;
        super.runOpMode();
    }
}
