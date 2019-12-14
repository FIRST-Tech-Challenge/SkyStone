package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Wheels Test", group = "basic")
public class WheelsTest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Crane.setupType.autonomous);

        if(opModeIsActive()){
            rob.driveTrainEncoderMovement(0.01, 10, 10, 2, Crane.movements.right);

            //rob.driveTrainEncoderMovement(0.001, 3, 3, 2, Crane.movements.backward);

            //rob.driveTrainEncoderMovement(0.001, 3, 3, 2, Crane.movements.left);

            //rob.driveTrainEncoderMovement(0.001, 3, 3, 2, Crane.movements.right);

        }
    }
}
