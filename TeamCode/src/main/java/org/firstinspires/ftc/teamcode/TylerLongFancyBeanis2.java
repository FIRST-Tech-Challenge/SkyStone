package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongFancyBeanis2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="LongFancyBeanis", group="fancy")
public class TylerLongFancyBeanis2 extends LongFancyBeanis2 {
    public TylerLongFancyBeanis2() {
        super(ChassisConfig.forTileRunner());
    }
}
