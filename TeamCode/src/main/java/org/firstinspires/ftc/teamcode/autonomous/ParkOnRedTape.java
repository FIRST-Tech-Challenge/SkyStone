package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.hardware.hardwareutils.HardwareManager;
import org.firstinspires.ftc.teamcode.subsystems.TwinstickMecanum;

@Autonomous
public class ParkOnRedTape extends OpMode {
    public HardwareManager hardware;

    public ColorSensor colorSensor;
    public AutoCommands autoCommands;
    public boolean done;

    @Override
    public void init() {
        hardware = new HardwareManager(hardwareMap);
        autoCommands = new AutoCommands(hardware, telemetry);
        done = false;
    }

    @Override
    public void loop() {
        colorSensor.enableLed(true);
        if (colorSensor.blue() < 20 && !done) {
            autoCommands.HorizontalMove(-1);
        } else {
            done = true;
            colorSensor.enableLed(false);
        }
    }
}
