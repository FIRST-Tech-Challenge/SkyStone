package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortRedBeanis1;
import org.firstinspires.ftc.teamcode.autoOp.ShortRedBeanis2;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Short Blue Beanis 2", group="redbean")
public class TylerShortRedBeanis2 extends ShortRedBeanis2 {
    public TylerShortRedBeanis2() {
        super(ChassisConfig.forTileRunner());
    }
}
