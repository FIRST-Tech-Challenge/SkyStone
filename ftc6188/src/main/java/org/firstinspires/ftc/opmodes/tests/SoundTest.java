package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.sound.BasicSound;
import org.firstinspires.ftc.robotlib.state.Button;

@Disabled
@TeleOp(name="Sound Test", group="Test")
public class SoundTest extends OpMode
{
    private BasicSound basicSound;
    private Button playSound;

    @Override
    public void init()
    {
        basicSound = new BasicSound("police_siren", this.hardwareMap);
        playSound = new Button();
    }

    @Override
    public void loop()
    {
        playSound.input(gamepad1.x);

        if (playSound.onPress()) { basicSound.toggleSound(); }

        telemetry.addData("PlaySoundButton", playSound.isPressed());
        telemetry.addData("Gamepad X", gamepad1.x);
        telemetry.addData("Is Playing", basicSound.isSoundPlaying());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        basicSound.stopSound();
    }
}
