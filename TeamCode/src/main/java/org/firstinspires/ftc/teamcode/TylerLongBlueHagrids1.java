package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongBlueHagrids1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Long Blue Hagrids 1", group="bluehagrids")
public class TylerLongBlueHagrids1 extends LongBlueHagrids1 {
    public TylerLongBlueHagrids1() {
        super(ChassisConfig.forTileRunner());
    }
}