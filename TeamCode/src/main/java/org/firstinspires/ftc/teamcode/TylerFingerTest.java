package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ClawTest;
import org.firstinspires.ftc.teamcode.autoOp.FingerTest;

@Autonomous(name="FingerTest", group="ZZTesting")
public class TylerFingerTest extends FingerTest {
    public TylerFingerTest() {
        super(ChassisConfig.forTileRunner());
    }
}