package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ClawTest;
import org.firstinspires.ftc.teamcode.autoOp.OniChan;

@Autonomous(name="ClawTest", group="clawtest")
public class TylerClawTest extends ClawTest {
    public TylerClawTest() {
        super(ChassisConfig.forTileRunner());
    }
}