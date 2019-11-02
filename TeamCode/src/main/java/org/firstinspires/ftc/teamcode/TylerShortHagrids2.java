package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortHagrids2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Short Blue Hagrids 2", group="bluehagrids")
public class TylerShortHagrids2 extends ShortHagrids2 {
    public TylerShortHagrids2() {
        super(ChassisConfig.forTileRunner());
    }
}