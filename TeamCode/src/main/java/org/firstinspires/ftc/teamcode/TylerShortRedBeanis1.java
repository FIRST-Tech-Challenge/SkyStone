package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortRedBeanis1;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Short Red Beanis 1", group="redbean")
public class TylerShortRedBeanis1 extends ShortRedBeanis1 {
    public TylerShortRedBeanis1() {
        super(ChassisConfig.forTileRunner());
    }
}
