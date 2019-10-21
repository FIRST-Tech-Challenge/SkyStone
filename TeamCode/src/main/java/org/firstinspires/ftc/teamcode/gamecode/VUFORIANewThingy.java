package org.firstinspires.ftc.teamcode.gamecode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Robot;

/**
 * Created by FIXIT on 2017-08-27.
 */

public class VUFORIANewThingy extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer ma = ClassFactory.createVuforiaLocalizer(params);

        VuforiaTrackables beacons = ma.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackableDefaultListener tools = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();
        VuforiaTrackableDefaultListener legos = (VuforiaTrackableDefaultListener) beacons.get(2).getListener();

        beacons.activate();

        Robot caesar = new Robot();
        caesar.reverseDriveSystem();
        waitForStart();

        Log.i("Images", tools.isVisible() + ", " + legos.isVisible());

        while (opModeIsActive() && !tools.isVisible() && !legos.isVisible()) {
            caesar.turnL(0.15);
        }//while

        if (tools.isVisible()) {

            caesar.forward(0.15, 500);

        } else if (legos.isVisible()) {
            caesar.backward(0.15, 500);
        }
    }
}
