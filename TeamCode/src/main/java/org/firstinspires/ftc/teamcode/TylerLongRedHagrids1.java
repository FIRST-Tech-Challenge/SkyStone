package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongRedHagrids1;
import org.firstinspires.ftc.teamcode.autoOp.LongRedHagrids2;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Long Red Hagrids 1", group="redhagrids")
public class TylerLongRedHagrids1 extends LongRedHagrids1 {
    public TylerLongRedHagrids1() {
        super(ChassisConfig.forTileRunner());
    }
}