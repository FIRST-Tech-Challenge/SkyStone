package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongHagrids1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Long Hagrid 1", group="AAAAAAAAAAAAAAAAAAAAAA")
public class TylerLongHagrids1 extends LongHagrids1 {
    public TylerLongHagrids1() {
        super(ChassisConfig.forTileRunner());
    }
}