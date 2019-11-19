package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.ShortBlueBeanis1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Short Blue Beanis 1", group="bluebeanis")
public class TylerShortBlueBeanis1 extends ShortBlueBeanis1 {
    public TylerShortBlueBeanis1() {
        super(ChassisConfig.forTileRunner());
    }
}
