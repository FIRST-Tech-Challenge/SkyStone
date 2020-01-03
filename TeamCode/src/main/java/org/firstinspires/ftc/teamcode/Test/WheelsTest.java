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
            rob.driveTrainEncoderMovement(.1, 1,30,0, Crane.movements.forward);
            sleep(2000);
            rob.driveTrainEncoderMovement(.1, 1,10,0, Crane.movements.backward);
            sleep(2000);
            rob.driveTrainEncoderMovement(.1, 3,10,0, Crane.movements.left);
            sleep(2000);
            rob.driveTrainEncoderMovement(.1, 3,10,0, Crane.movements.right);

        }
    }
}
