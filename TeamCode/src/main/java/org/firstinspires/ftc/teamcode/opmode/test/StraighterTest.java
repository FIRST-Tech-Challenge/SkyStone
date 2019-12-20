package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Straighter Test")
@Disabled
public class StraighterTest extends LinearOpMode {

    public void runOpMode(){
        while(!isStopRequested()){
            hardwareMap.dcMotor.get("leftFront").setPower(1);
            hardwareMap.dcMotor.get("leftRear").setPower(1);
            hardwareMap.dcMotor.get("rightFront").setPower(1);
            hardwareMap.dcMotor.get("rightRear").setPower(1);
        }
    }

}
