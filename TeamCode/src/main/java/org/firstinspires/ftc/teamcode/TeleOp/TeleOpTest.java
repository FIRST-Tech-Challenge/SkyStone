package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Crane;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "TeleOp Test", group = "Concept")



public class TeleOpTest extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive, Crane.setupType.claw);

        while (opModeIsActive()){
            standardGamepadData();
            if (g(8)){
                telemetry.addLine("Rotate Left");
                telemetry.update();
            }
            else if (g(9)){
                telemetry.addLine("Rotate Right");
                telemetry.update();
            }
            else if(g(0)){
                //rob.driveTrainMovement(fb, Crane.movements.forward);
                telemetry.addLine("Forward");
                telemetry.update();
            }
            else if (g(1)){
                //rob.driveTrainMovement(rl, Crane.movements.left);
                telemetry.addLine("Left");
                telemetry.update();
            }
            else if (g(2)){
                telemetry.addLine("Backward");
                telemetry.update();
            }
            else if (g(3)){
                telemetry.addLine("Right");
                telemetry.update();
            }
            else if (g(4)){
                telemetry.addLine("Diagonal Top Right");
                telemetry.update();
            }
            else if (g(5)){
                telemetry.addLine("Diagonal Top Left");
                telemetry.update();
            }
            else if (g(6)){
                telemetry.addLine("Diagonal Bottom Left");
                telemetry.update();
            }
            else if (g(7)){
                telemetry.addLine("Diagonal Bottom Right");
                telemetry.update();
            }
            else{
                rob.stopDrivetrain();
            }


        }
    }
}
