package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongBlueHagrids2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Long Blue Hagrids 2", group="bluehagrids")
public class TylerLongBlueHagrids2 extends LongBlueHagrids2 {
    public TylerLongBlueHagrids2() {
        super(ChassisConfig.forTileRunner());
    }
}