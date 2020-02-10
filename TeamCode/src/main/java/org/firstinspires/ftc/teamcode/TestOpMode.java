package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;
import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.GuidanceSubSystem;

public class TestOpMode extends OpMode {
    Robot bot;

    @Override
    public void init() {
            bot = new Robot(this);
            bot.StartIMUSubSystem();
            bot.StartGuidanceSubSystem();
    }

    @Override
    public void loop() {
        bot.Update();
        GuidanceSubSystem.targetAngle = 90;

    }


}
