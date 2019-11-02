package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongRedBeanis2;
import org.firstinspires.ftc.teamcode.autoOp.LongRedHagrids2;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Long Red Hagrids 2", group="redhagrids")
public class TylerLongRedHagrids2 extends LongRedHagrids2 {
    public TylerLongRedHagrids2() {
        super(ChassisConfig.forTileRunner());
    }
}