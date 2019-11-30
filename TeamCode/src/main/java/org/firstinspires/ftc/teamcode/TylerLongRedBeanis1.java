package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongRedBeanis1;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Long Red Beanis 1", group="redbean")
public class TylerLongRedBeanis1 extends LongRedBeanis1 {
    public TylerLongRedBeanis1() {
        super(ChassisConfig.forTileRunner());
    }
}