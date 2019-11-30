package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongBlueBeanis1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Long Blue Beanis 1", group="bluebeanis")
public class TylerLongBlueBeanis1 extends LongBlueBeanis1 {
    public TylerLongBlueBeanis1() {
        super(ChassisConfig.forTileRunner());
    }
}