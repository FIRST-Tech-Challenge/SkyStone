//Jun Park
package org.firstinspires.ftc.teamcode.Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@TeleOp(name = "AutonomousMode", group = "Iterative Opmode")
public class AutonomousMode extends LinearOpMode {
    ControlMode FC = new ControlMode();

    public void initMotors() {

    }

    public void runOpMode() {
        initMotors();
        telemetry.addData("status", "Initialized");
        waitForStart();

        //doAction("forward", 5.0);
        //doAction("backward", 5.0);
        //doAction("rotate", 10.0);
        //doAction("left", 5.0);
        //doAction("right", 5.0);

        sleep(1000);

        stop();
    }

}
