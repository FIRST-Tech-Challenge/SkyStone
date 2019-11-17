package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ClawTest;
import org.firstinspires.ftc.teamcode.autoOp.MoveTest;

@Autonomous(name="Move Test", group="ZZTesting")
public class TylerMoveTest extends MoveTest {
    public TylerMoveTest() {
        super(ChassisConfig.forTileRunner());
    }
}