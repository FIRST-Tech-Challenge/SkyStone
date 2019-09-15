package org.firstinspires.ftc.teamcode.RoverRuckus.Deployers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckus.BaseTeleOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mappings.TandemMapping;
@TeleOp(name="Sparky Tandem Crater")
public class TandemTeleOpCrater extends BaseTeleOp {
    @Override
    public void runOpMode() {
        this.controller = new TandemMapping(gamepad1, gamepad2);
        TandemMapping.MAX_MOVE_SPEED = 0.8;
        this.fieldCentric = false;
        this.hangOnCrater = true;
        this.timing = true;
        super.runOpMode();
    }
}
