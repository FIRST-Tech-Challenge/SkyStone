package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "SKYSTONE-AUTO-V1.2")
public class SkyStoneAuto2 extends autoBase {
    public double powerUp = 0.5, powerDown = -0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        /*
        REVISED SKYSTONE AUTO 1
         */

        goForward();
        wait(0.4, "going forward slightly (PHASE 1)");
        rest();
        sleep(600);

        rotate(87, power);
        wait(2.0, "turning left (PHASE 2)");
        rest();
        sleep(600);

        goForward();
        wait(1.9, "going forward (PHASE 3)");
        rest();
        sleep(600);

        rotate(-86, power);
        wait(2.0, "turning right (PHSAE 4)");
        rest();
        sleep(600);

        goForward();
        while(opModeIsActive()&&!tripWireActive(11)){
            telemetry.addData("", "going forward (PHASE 5)");
            telemetry.update();
        }
        rest();
        sleep(500);


        dropDL();
        wait(2.0, "Dropping Hookers (PHASE 6)");


        goBack();
        wait(2.0, "going back (PHASE 7)");
        rest();
        sleep(600);

        raiseDL();
        sleep(2000);

        goBack();
        wait(0.5, "going back slightly more (PHASE 8)");

        //strafe right
        chart.TL.setPower(powerUp + joltControl());
        chart.TR.setPower(powerDown);
        chart.BL.setPower(powerDown);
        chart.BR.setPower(powerUp);
        wait(2.7, "strafing right (PHASE 9)");


    }
}
