package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongHagrids2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Long Blue Hagrid 2", group="bluehagrids")
public class TylerLongHagrids2 extends LongHagrids2 {
    public TylerLongHagrids2() {
        super(ChassisConfig.forTileRunner());
    }
}