package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Elevator;
import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

@TeleOp(name = "Elevator Test")
@Config
public class ElevatorTest extends LinearOpMode {
    public static double goalHeight = 10;

    @Override
    public void runOpMode(){
        Elevator elevator = new Elevator(hardwareMap);
        OmegaGamepad omegaGamepad = new OmegaGamepad(gamepad2);
        waitForStart();
        elevator.setPosition(goalHeight);
        elevator.resetEncoder();
        while(!isStopRequested()) {
            elevator.update();
            omegaGamepad.update();
            if(omegaGamepad.ifOnceDPadUp()){
                elevator.setPosition(goalHeight);
            }
            if(omegaGamepad.ifOnceDPadDown()){
                elevator.setPosition(0);
            }
            telemetry.addData("Height", elevator.getRelativeHeight());
            telemetry.addData("State: ", elevator.getCurrentState());
            telemetry.addData("Encoder Position: ", elevator.getEncoderPosition());
            telemetry.update();
        }
    }
}
