package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.VuforiaNavigator;


@TeleOp(name = "Vuforia_OneMotor_Test")

public class VuforiaOneMotorReactionTest extends OpMode {
    private VuforiaNavigator vuforiaNavigator;
    HardwareChassis robot;

    private DcMotor motor_1;

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        vuforiaNavigator = new VuforiaNavigator(hardwareMap, robot, telemetry, () -> true);
        motor_1 = hardwareMap.get(DcMotor.class, "hub2_motorport1");
    }

    @Override
    public void loop() {

        //if vuforia recognizes the target, start motor hub1_motorport0 on power 0.2
        if (vuforiaNavigator.skystoneFound() == true) {
            motor_1.setPower(0.2); // "hub2_motorport1"
        } else {
            motor_1.setPower(0
            );
        }

        /*
        //if camera is on x angle between -5 and 5 values, start motor hub1_motorport0 turn other direction
        if (vuforiaNavigator.showTargetX() >= -5 || vuforiaNavigator.showTargetX() <= 5) {
            robot.motor_front_right.setPower(-0.2);
        }
        */



        telemetry.addData("Is Target Visible: ", vuforiaNavigator.skystoneFound());
        telemetry.addData("x: ", vuforiaNavigator.showTargetX());
        telemetry.addData("y: ", vuforiaNavigator.showTargetY());
        telemetry.addData("z: ", vuforiaNavigator.showTargetZ());
        telemetry.addData("rX: ", vuforiaNavigator.showRX());
        telemetry.addData("rY: ", vuforiaNavigator.showRY());
        telemetry.addData("rZ: ", vuforiaNavigator.showRZ());
        telemetry.update();
    }
}
