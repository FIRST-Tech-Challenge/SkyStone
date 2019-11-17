// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Arm Test", group="Exercises")

public class ArmTestAutoOpMode extends LinearOpMode {

    PinchArmBot robot = new PinchArmBot(this);
//    ScoopArmBot robot = new ScoopArmBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        int step = 0;
        while (opModeIsActive()) {
            if (step == 0){

                robot.pickupSkyStone();
                step = 1;
            }
            else if (step == 1){
                robot.dropSkyStone();
                step = 2;
            }
            else if (step == 2){
                robot.resetArm();
                step = 3;
            }
            sleep(10*1000);
        }
//        robot.scoopStone();
    }

}
