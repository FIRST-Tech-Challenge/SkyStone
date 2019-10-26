package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.FivePoints;
import org.firstinspires.ftc.teamcode.autoOp.OniChan;
import org.firstinspires.ftc.teamcode.autoOp.SafetyPatrol;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="JustParkItMiddle", group="AAAAAAAAAAAAAAAAAAAAAA")
public class TylerFivePoints extends FivePoints {
    public TylerFivePoints() {
        super(ChassisConfig.forTileRunner());
    }
}