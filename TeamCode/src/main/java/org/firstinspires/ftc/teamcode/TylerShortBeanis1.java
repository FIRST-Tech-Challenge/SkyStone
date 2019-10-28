package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortBeanis1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="JustParkItMiddle", group="a")
public class TylerShortBeanis1 extends ShortBeanis1 {
    public TylerShortBeanis1() {
        super(ChassisConfig.forTileRunner());
    }
}
