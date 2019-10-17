package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ExpansionHub;

@TeleOp(name = "Concept Tester", group = "none")
public class ExperimentalStuff extends OpMode {

    private Robot bot;
    private int lastSpake;
    private BNO055IMU backupGyro1;

    @Override
    public void init() {
        bot = Robot.getInstance();
        bot.init(hardwareMap);

        double volts = bot.expansionHubs.get("Expansion Hub 2").voltageBattery(ExpansionHub.VoltageUnits.VOLTS);
        telemetry.addData("Battery Voltage", volts);
        telemetry.addData("Phone Battery", bot.phone.batteryPct());
        telemetry.update();
        if (volts < 11) {
            bot.phone.setBackgroundColor(0xFF, 0x00, 0x00);
        } else if (volts < 12) {
            bot.phone.setBackgroundColor(0xFF, 0x80, 0x00);
        } else if (volts < 12.5) {
            bot.phone.setBackgroundColor(0xFF, 0xFF, 0x00);
        } else {
            bot.phone.resetBackgroundColor();
        }

        bot.phone.toast("Program Initialized.", 2000);

        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu");
    }

    @Override
    public void init_loop() {
        double volts = bot.expansionHubs.get("Expansion Hub 2").voltageBattery(ExpansionHub.VoltageUnits.VOLTS);
        telemetry.addData("Battery Voltage", volts);
        telemetry.update();
    }

    @Override
    public void start() {
        bot.phone.resetBackgroundColor();
        bot.phone.toast("Program started.", 0);
        bot.runtime.reset();
        lastSpake = 30;
    }

    @Override
    public void loop() {
        double rev = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        bot.driveTrain.spinDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (30 - bot.runtime.seconds() < lastSpake && lastSpake >= 0) {
            bot.phone.queueWordSpeak(String.valueOf(lastSpake));
            lastSpake--;
        }
        telemetry.addData("Speaking", bot.phone.hasQueuedSound());

        telemetry.addData("Rev Gyro", rev);
        telemetry.addData("Phone Gyro", bot.phone.getGyroAngle());

        if (gamepad1.b && bot.runtime.seconds() > 30 && !bot.phone.hasQueuedSound()) {
            bot.phone.queueSoundFile("space_odyssey");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        bot.close();
    }
}
