package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Crane;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@Autonomous(name = "Claw Test", group = "Concept")



public class ClawTest extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.claw);

        if (opModeIsActive()){
            //rob.encodeCoreHexMovement(1, 3, 10, 0, Crane.movements.linearUp, rob.rightLinear);
            //rob.encodeCoreHexMovement(1, 4, 10, 0, Crane.movements.clawOut, rob.extend);
            rob.rotationservo.setPosition(0.5);
            rob.rightServo.setPosition(1);
        }
    }
}
