package org.firstinspires.ftc.teamcode.RoverRuckus.Deployers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckus.BaseTeleOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mappings.TandemMapping;

@TeleOp(name="Sparky Tandem Depo")
public class TandemTeleOpDepo extends BaseTeleOp {
    @Override
    public void runOpMode() {
        this.controller = new TandemMapping(gamepad1, gamepad2);
        TandemMapping.MAX_MOVE_SPEED = 1.0;
        this.fieldCentric = false;
        this.hangOnCrater = false;
        this.timing = true;
        super.runOpMode();
    }
}
