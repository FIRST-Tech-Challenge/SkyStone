package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="auto1a", group="Game Controller")
public class auto1a extends Autonomy
{
    public auto1a() {
        parkBridgeSide = true;
        execCollectStones = true;
        execMoveFoundation = true;
    }
}
