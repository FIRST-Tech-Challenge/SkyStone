package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "BasicTeleOp", group = "Basic")
public class BasicTele extends LinearOpMode {
    public DcMotor armRaise;
    public DcMotor armExtend;

    @Override
    public void runOpMode() throws InterruptedException {
        TypexChart chart = new TypexChart();

        chart.init(hardwareMap);

        armRaise = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");

        waitForStart();

        //Setting drive parameters
        while(opModeIsActive()) {
            chart.TL.setPower(gamepad1.left_stick_y);
            chart.BL.setPower(gamepad1.left_stick_y);
            chart.TR.setPower(gamepad1.right_stick_y);
            chart.BR.setPower(gamepad1.right_stick_y);

            telemetry.addLine();
            telemetry.addData("Distance", "74");
        }
    }
    public int boolPower(boolean power) {
        if (power == true) {
            return (1);
        }
        else {
            return (0);
        }
    }
}

