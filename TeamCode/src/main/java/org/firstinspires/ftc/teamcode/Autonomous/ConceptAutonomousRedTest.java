package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

@Autonomous (name = "C_Autonomous_RedTest")

public class ConceptAutonomousRedTest extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    //ControlledDrive controlledDrive;
    //GeneralTools generalTools;

    @Override
    public void runOpMode(){
        colorTools = new ColorTools();
        robot = new HardwareChassis(hardwareMap);

        /*
        controlledDrive = new ControlledDrive(hardwareMap, telemetry, () -> opModeIsActive()); //
        generalTools = new GeneralTools(this, robot);
        */

        /*
        while (opModeIsActive()) {

            //open clamp
            generalTools.openClamp();

            //drive forward 80cm with speed 0.1
            //stop after 5 seconds
            controlledDrive.driveDistance(80, 0, 0.1, 5);

            //grab skystone
            generalTools.grabSkysstone();
        }
        */
        while (opModeIsActive()){
            if (!colorTools.isRed(robot.color_back)) {
                robot.motor_front_right.setPower(0.1);
                robot.motor_front_left.setPower(0.1);
                robot.motor_rear_right.setPower(0.1);
                robot.motor_rear_left.setPower(0.1);
            }
        }

    }
}
