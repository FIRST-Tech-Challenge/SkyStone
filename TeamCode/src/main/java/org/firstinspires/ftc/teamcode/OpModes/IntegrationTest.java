package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad1;

import java.util.Iterator;

@TeleOp
public class IntegrationTest extends LinearOpMode {
    Chassis chassis;

    @Override
    public void runOpMode() {
    /*
        Chassis chassis = new Chassis(hardwareMap);
        telemetry.setMsTransmissionInterval(1);
        telemetry.addLine("Init | v1.0");
        Iterator<DcMotor> motorIterator = hardwareMap.dcMotor.iterator();
        while (motorIterator.hasNext()) {
            DcMotor motor = motorIterator.next();
            telemetry.addData("Motor Name: ", motor.toString());
            telemetry.addLine("Direction: Forward");
            motor.setPower(1);
            sleep(100);
            telemetry.addLine("Direction: Reverse");
            motor.setPower(-1);
            sleep(100);
            motor.setPower(0);
        }
        HzGamepad1 controller = new HzGamepad1(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            //Other unit test code if you want
            telemetry.update();
        }
    */
    }

}