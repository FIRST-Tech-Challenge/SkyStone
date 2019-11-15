package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.sound.BasicSound;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@TeleOp(name="Sound Test", group="Test")
public class SoundTest extends OpMode
{
    private BasicSound basicSound;
    private ToggleBoolean playSound;

    @Override
    public void init()
    {
        basicSound = new BasicSound("police_siren", this.hardwareMap);
        playSound = new ToggleBoolean();
    }

    @Override
    public void loop()
    {
        playSound.input(gamepad1.x);

        if (playSound.output())
        {
            basicSound.playSound();
        }

        telemetry.addData("PlaySound", playSound.output());
        telemetry.addData("Gamepad X", gamepad1.x);
        telemetry.update();
    }
}
