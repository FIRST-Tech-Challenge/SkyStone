package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongFancyBeanis2;
import org.firstinspires.ftc.teamcode.autoOp.ShortFancyBeanis2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//Beanis = Square
//Hagrids = Slash

@Disabled(/*name="ShortFancyBeanis", group="fancy"*/)
public class TylerShortFancyBeanis2 extends ShortFancyBeanis2 {
    public TylerShortFancyBeanis2() {
        super(ChassisConfig.forTileRunner());
    }
}
