package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.hardwareutils.HardwareManager;

import static android.os.SystemClock.sleep;

public class AutoForward extends OpMode {
    public HardwareManager hardware;
    public AutoCommands autoCommands;


    public void init() {
        hardware = new HardwareManager(hardwareMap);
        autoCommands = new AutoCommands(hardware, telemetry);
    }


    public void loop() {
        autoCommands.driveForward(0.75);
        sleep(1000);
        autoCommands.driveForward(0.0);
    }
}
