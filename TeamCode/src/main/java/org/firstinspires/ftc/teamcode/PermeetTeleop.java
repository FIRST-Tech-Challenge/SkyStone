package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Permeet's First TeleOp", group="Permeet")
public class PermeetTeleop extends LinearOpMode{

    @Override
    public void runOpMode(){
        telemetry.addData( "Hi there ", "Permeet" );
        telemetry.update();
    }
}
