package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Crane;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

public class TeleOp extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive);

        while (opModeIsActive()){

            // GAMEPAD OBJECT
            standardGamepadData();





        }
    }
}
