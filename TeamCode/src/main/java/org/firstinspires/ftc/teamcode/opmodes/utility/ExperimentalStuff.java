package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ExpansionHub;

@TeleOp(name = "Concept Tester", group = "none")
public class ExperimentalStuff extends OpMode {

    private Robot bot;
    private BNO055IMU backupGyro1;

    @Override
    public void init() {
        bot = Robot.getInstance();
        bot.init(hardwareMap);

        double volts = bot.secondHub.voltageBattery(ExpansionHub.VoltageUnits.VOLTS);
//        telemetry.addData("Battery Voltage", volts);
        double phoneVolts = bot.phone.batteryPct();
        RobotLog.v("Phone Battery is " + phoneVolts + " percent.");
        telemetry.addData("Phone Battery", phoneVolts);
        telemetry.update();

        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
    }

    @Override
    public void init_loop() {
        double volts = bot.secondHub.voltageBattery(ExpansionHub.VoltageUnits.VOLTS);
        telemetry.addData("Battery Voltage", volts);
        telemetry.update();
    }

    @Override
    public void start() {
        bot.phone.resetBackgroundColor();
        bot.phone.toast("Program started.", 0);
        bot.secondHub.setStatusLedColor(0xff,0x00,0x7f);
        bot.secondHub.setPhoneChargeEnabled(true);
        bot.runtime.reset();
    }

    @Override
    public void loop() {
        double rev = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        bot.driveTrain.spinDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Speaking", bot.phone.hasQueuedSound());

        telemetry.addData("Rev Gyro", rev);
        telemetry.addData("Phone Gyro", bot.phone.getGyroAngle());

        telemetry.update();
    }

    @Override
    public void stop() {
        bot.close();
    }
}
