package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Templates;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.RR2;
@Deprecated
@TeleOp(name="Template Teleop", group="Linear Opmode")
@Disabled //COMMENT OUT TO MAKE PROGRAM APPEAR ON PHONE
public class TemplateTeleop extends LinearOpMode {

    RR2 robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //init logic
        robot = new RR2(hardwareMap, telemetry, this);   //DO NOT DELETE
        while (opModeIsActive()) {
            //code here when start button is pressed
        }
    }
}



