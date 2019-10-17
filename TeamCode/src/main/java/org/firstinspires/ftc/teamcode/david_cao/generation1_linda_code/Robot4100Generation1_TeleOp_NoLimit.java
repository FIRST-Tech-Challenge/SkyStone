package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "4100Gen1TeleOp_NOLIMIT_dcao",group = "4100")
public class Robot4100Generation1_TeleOp_NoLimit extends Robot4100Generation1_TeleOp {
    @Override
    public void hardwareInitialize(){
        super.hardwareInitialize();
        super.getRobotCore().getLinearSlide().setMinPos(-1000);
        super.getRobotCore().getLinearSlide().setMaxPos(1000);
    }
}
