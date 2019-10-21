package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Enigma;

/**
 * Created by Aila on 2017-08-27.
 */

public class VuforiaTest extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {

        final Enigma bot = new Enigma();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer vuf = ClassFactory.createVuforiaLocalizer(params);

        VuforiaTrackables beacons = vuf.loadTrackablesFromAsset("FTC_2016-17");

        beacons.activate();

        VuforiaTrackableDefaultListener tools = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();
        VuforiaTrackableDefaultListener legos = (VuforiaTrackableDefaultListener) beacons.get(2).getListener();



        waitForStart();

        /*
        while (opModeIsActive()){

            //bot.forwardDistance(100, 0.5);
            bot.stop(5);

            RC.t.addData("Vuforia Data", tools.isVisible() + ", " + legos.isVisible());

            if (tools.isVisible()){
                bot.imuTurnL(90, 0.75);
                RC.t.addData("I see tools");
            }
            else if (legos.isVisible()){
                bot.imuTurnR(90, 0.75);
                RC.t.addData("I see legos");
            }
            else{
                RC.t.addData("I see nothing!");
            }
            bot.stops();
        }*/

        while (opModeIsActive()){

            bot.forward(0.5);
            RC.t.addData("Reporting en route");
            sleep(1000);
            bot.imuTurnR(90, 0.5);

            sleep(2000);


        }
    }
}
