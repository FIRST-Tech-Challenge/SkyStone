package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortHagrids1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Short Blue Hagrids 1", group="bluehagrids")
public class TylerShortHagrids1 extends ShortHagrids1 {
    public TylerShortHagrids1() {
        super(ChassisConfig.forTileRunner());
    }
}