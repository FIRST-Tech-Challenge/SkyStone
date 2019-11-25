package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

@TeleOp(name="Limit Switch Tester")
public class LimitSwitchTest extends LinearOpMode {
    public void runOpMode(){
        HardwareMap hwMap = new HardwareMap(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("liftReset", hwMap.liftReset.getState());
        }
    }
}
