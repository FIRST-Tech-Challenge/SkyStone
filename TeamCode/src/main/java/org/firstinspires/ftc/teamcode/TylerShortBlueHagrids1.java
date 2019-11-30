package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortBlueHagrids1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Short Blue Hagrids 1", group="bluehagrids")
public class TylerShortBlueHagrids1 extends ShortBlueHagrids1 {
    public TylerShortBlueHagrids1() {
        super(ChassisConfig.forTileRunner());
    }
}