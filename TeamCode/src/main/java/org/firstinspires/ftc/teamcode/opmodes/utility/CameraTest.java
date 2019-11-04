package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.westtorrancerobotics.lib.Location;

import java.util.Map;

@TeleOp(name = "Camera", group = "none")
public class CameraTest extends OpMode {
    private Robot cambot;
    private Location position;
    private Location skyposition;




    @Override
    public void init() {
    cambot.camera.init(hardwareMap);
    }

    @Override
    public void loop() {
        cambot.camera.start();
        cambot.camera.process();
        position = cambot.camera.currentPosition();
        skyposition =cambot.camera.relativeSkystonePosition();
        telemetry.addData ("pos", position);
        telemetry.addData ("Skypos", skyposition);
        telemetry.update();

    }
}
