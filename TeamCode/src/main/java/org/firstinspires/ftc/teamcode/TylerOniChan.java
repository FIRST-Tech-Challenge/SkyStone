package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.OniChan;
import org.firstinspires.ftc.teamcode.autoOp.SafetyPatrol;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Turn", group="AAAAAAAAAAAAAAAAAAAAAA")
public class TylerOniChan extends OniChan {
    public TylerOniChan() {
        super(ChassisConfig.forTileRunner());
    }
}