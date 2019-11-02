package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortRedBeanis2;
import org.firstinspires.ftc.teamcode.autoOp.ShortRedHagrids2;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Short Red Hagrids 2", group="redhagrids")
public class TylerShortRedHagrids2 extends ShortRedHagrids2 {
    public TylerShortRedHagrids2() {
        super(ChassisConfig.forTileRunner());
    }
}
