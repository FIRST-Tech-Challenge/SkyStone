package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongRedBeanis1;
import org.firstinspires.ftc.teamcode.autoOp.LongRedBeanis2;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Long Red Beanis 2", group="redbean")
public class TylerLongRedBeanis2 extends LongRedBeanis2 {
    public TylerLongRedBeanis2() {
        super(ChassisConfig.forTileRunner());
    }
}