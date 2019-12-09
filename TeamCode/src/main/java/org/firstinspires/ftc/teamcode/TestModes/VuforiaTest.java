package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.VuforiaNavigator;

@Autonomous(name="VuforiaTest")
public class VuforiaTest extends LinearOpMode {
    private VuforiaNavigator vuforiaNavigator;
    private HardwareChassis robot;
    private ControlledDrive controlledDrive;
    private GeneralTools generalTools;

    @Override public void runOpMode() {
        this.robot = new HardwareChassis(hardwareMap);
        this.vuforiaNavigator = new VuforiaNavigator(hardwareMap, robot, telemetry);
        this.controlledDrive= new ControlledDrive(hardwareMap);
        this.generalTools = new GeneralTools(this, robot);

        telemetry.addData("State", "sideward");
        telemetry.update();

        //drive method, speed forward, speed sideways
        //controlledDrive.driveConditionally(0.5, 0, () -> true);
        //TODO: method grabSkystone() --> void, skystone greifen
        double[] result = OmniWheel.calculate(5.0, 38, 24, 0, -0.1, 0);
        robot.motor_front_left.setPower(result[0]);
        robot.motor_front_right.setPower(result[1]);
        robot.motor_rear_left.setPower(result[2]);
        robot.motor_rear_right.setPower(result[3]);

        while (!vuforiaNavigator.skystoneFound() && opModeIsActive()) {}
        telemetry.addData("State", "found");
        telemetry.update();

        robot.motor_front_left.setPower(0);
        robot.motor_front_right.setPower(0);
        robot.motor_rear_left.setPower(0);
        robot.motor_rear_right.setPower(0);


        vuforiaNavigator.navigateToSkystone(0.2, -0.05);

        generalTools.grabSkysstone();
    }
}
