package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.SparkyTheRobot;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevExtensions2;

import java.util.LinkedList;
import java.util.List;

@TeleOp
public class ServoTest extends LinearOpMode {
    public static int CYCLE_TIME = 2000;

    public static List<ServoImplEx> servos;

    @Override
    public void runOpMode() throws InterruptedException {
        RevExtensions2.init();

        servos = hardwareMap.getAll(ServoImplEx.class);
        ExpansionHubEx expansionHub = hardwareMap.get(ExpansionHubEx.class, "hub");
        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            if (t.milliseconds() < CYCLE_TIME / 2) {
                setPosition(0);
            } else if (t.milliseconds() < CYCLE_TIME) {
                setPosition(1);
            } else {
                t.reset();
            }
            telemetry.addData("Servo current", expansionHub.getServoBusCurrentDraw());
            telemetry.addData("GPIO current", expansionHub.getGpioBusCurrentDraw());
            telemetry.addData("I2C current", expansionHub.getI2cBusCurrentDraw());
            telemetry.addData("Total current", expansionHub.getTotalModuleCurrentDraw());
            telemetry.update();
        }
    }

    public void setPosition(double p) {
        for (ServoImplEx s : servos) {
            s.setPosition(p);
        }
    }
}
