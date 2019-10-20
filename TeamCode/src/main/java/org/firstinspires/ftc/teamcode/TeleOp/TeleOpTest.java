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

        setup(runtime, Crane.setupType.drive, Crane.setupType.bSystem);

        while (opModeIsActive()){
            standardGamepadData();
            if (g(8)){
                rob.driveTrainMovement(fb, Crane.movements.ccw);
                telemetry.addLine("Rotate Left");
                telemetry.update();
            }
            else if (g(9)){
                rob.driveTrainMovement(fb, Crane.movements.cw);
                telemetry.addLine("Rotate Right");
                telemetry.update();
            }
            else if(g(0)){
                rob.driveTrainMovement(fb, Crane.movements.forward);
                telemetry.addLine("Forward");
                telemetry.update();
            }
            else if (g(1)){
                rob.driveTrainMovement(rl, Crane.movements.left);
                telemetry.addLine("Left");
                telemetry.update();
            }
            else if (g(2)){
                rob.driveTrainMovement(fb, Crane.movements.backward);
                telemetry.addLine("Backward");
                telemetry.update();
            }
            else if (g(3)){
                rob.driveTrainMovement(fb, Crane.movements.right);
                telemetry.addLine("Right");
                telemetry.update();
            }
            else if (g(4)){
                rob.driveTrainMovement(fb, Crane.movements.tr);
                telemetry.addLine("Diagonal Top Right");
                telemetry.update();

            }
            else if (g(5)){
                rob.driveTrainMovement(fb, Crane.movements.tl);
                telemetry.addLine("Diagonal Top Left");
                telemetry.update();
            }
            else if (g(6)){
                rob.driveTrainMovement(fb, Crane.movements.bl);
                telemetry.addLine("Diagonal Bottom Left");
                telemetry.update();
            }
            else if (g(7)){
                rob.driveTrainMovement(fb, Crane.movements.br);
                telemetry.addLine("Diagonal Bottom Right");
                telemetry.update();
            }
            else{
                rob.stopDrivetrain();
                telemetry.addLine("Stop");
                telemetry.update();
            }



        }
    }
}
