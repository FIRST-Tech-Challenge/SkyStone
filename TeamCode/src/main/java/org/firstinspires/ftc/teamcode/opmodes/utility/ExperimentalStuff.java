package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private DcMotorEx motorTest;

    @Override
    public void init() {
        bot = Robot.getInstance();
        bot.init(hardwareMap);

        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());

        motorTest = hardwareMap.get(DcMotorEx.class, "leftFront");
    }

    @Override
    public void init_loop() {
        double volts = bot.secondHub.voltageBattery(ExpansionHub.VoltageUnits.VOLTS);
        telemetry.addData("Battery Voltage", volts);
        telemetry.addData("Firmware Version Control", bot.controlHub.getFirmwareVersion());
        telemetry.addData("Firmware Version Second", bot.secondHub.getFirmwareVersion());
        telemetry.update();
    }

    @Override
    public void start() {
        bot.secondHub.setStatusLedColor((byte) 0xff,(byte) 0x00,(byte) 0x7f);
        bot.runtime.reset();
    }

    @Override
    public void loop() {
        double rev = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        bot.driveTrain.spinDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Rev Gyro", rev);
        telemetry.addData("Motor Current", bot.controlHub.getMotorCurrentDraw(motorTest, ExpansionHub.CurrentDrawUnits.AMPS));
        telemetry.addData("Motor voltage", bot.controlHub.getMotorVoltagePercent(motorTest));

        telemetry.update();
    }

    @Override
    public void stop() {
        bot.close();
    }
}
