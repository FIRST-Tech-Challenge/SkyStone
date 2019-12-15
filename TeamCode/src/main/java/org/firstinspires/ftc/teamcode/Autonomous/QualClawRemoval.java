package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Qual Claw Release TRest", group = "AA" +
        "")
public class QualClawRemoval extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        int n =1;
        setup(runtime, Crane.setupType.drive, Crane.setupType.foundation, Crane.setupType.ultrasoinc, Crane.setupType.imu, Crane.setupType.claw);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            rob.foundationServo1.setPosition(.6);
            rob.foundationServo2.setPosition(0);
            rob.encoderMovement(0.8, 0.2, 3, 200, Crane.movements.linearUp, rob.rightLinear);
            sleep(300);
            rob.rotationservo.setPosition(.5);

        }


    }
}
