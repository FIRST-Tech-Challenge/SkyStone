package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.FivePoints;
import org.firstinspires.ftc.teamcode.autoOp.FivePoints2;
import org.firstinspires.ftc.teamcode.autoOp.OniChan;
import org.firstinspires.ftc.teamcode.autoOp.SafetyPatrol;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="JustParkIt", group="AAAAAAAAAAAAAAAAAAAAAA")
public class TylerFive2 extends FivePoints2 {
    public TylerFive2() {
        super(ChassisConfig.forTileRunner());
    }
}