package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;

public class TestOpMode extends OpMode {
    Robot bot;
    @Override
    public void init() {
            bot = new Robot(this);
            bot.StartIMUSubSystem();
    }

    @Override
    public void loop() {
        bot.Update();
    }
}
