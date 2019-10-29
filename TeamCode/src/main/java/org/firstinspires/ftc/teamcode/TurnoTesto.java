package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.LongBeanis1;
import org.firstinspires.ftc.teamcode.autoOp.TurnTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Beanis = Square
//Hagrids = Slash

@Autonomous(name="Turn Test", group="a")
public class TurnoTesto extends TurnTest {
    public TurnoTesto() {
        super(ChassisConfig.forTileRunner());
    }
}