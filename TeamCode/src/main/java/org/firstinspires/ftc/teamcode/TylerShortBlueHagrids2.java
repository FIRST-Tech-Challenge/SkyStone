package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortBlueHagrids2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Short Blue Hagrids 2", group="bluehagrids")
public class TylerShortBlueHagrids2 extends ShortBlueHagrids2 {
    public TylerShortBlueHagrids2() {
        super(ChassisConfig.forTileRunner());
    }
}