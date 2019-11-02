package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortRedHagrids1;
import org.firstinspires.ftc.teamcode.autoOp.ShortRedHagrids2;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Short Red Hagrids 1", group="redhagrids")
public class TylerShortRedHagrids1 extends ShortRedHagrids1 {
    public TylerShortRedHagrids1() {
        super(ChassisConfig.forTileRunner());
    }
}
