package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="IMU Test", group = "basic")
public class IMUTest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Crane.setupType.drive, Crane.setupType.imu);

        if(opModeIsActive()){
            rob.turn(90, Crane.turnside.ccw, 0.5, Crane.axis.center);
            sleep(500);
            //rob.turn(180, Crane.turnside.cw, 0.5, Crane.axis.center);
            sleep(50000);
            /*rob.turn(270, Crane.turnside.cw, 0.5, Crane.axis.center);
            sleep(500);
            rob.turn(360, Crane.turnside.cw, 0.5, Crane.axis.center);
            sleep(500);

            rob.turn(90, Crane.turnside.ccw, 0.5, Crane.axis.center);
            sleep(500);
            rob.turn(180, Crane.turnside.ccw, 0.5, Crane.axis.center);
            sleep(500);
            rob.turn(270, Crane.turnside.ccw, 0.5, Crane.axis.center);
            sleep(500);
            rob.turn(360, Crane.turnside.ccw, 0.5, Crane.axis.center);
            sleep(500);

             */



        }
    }
}
