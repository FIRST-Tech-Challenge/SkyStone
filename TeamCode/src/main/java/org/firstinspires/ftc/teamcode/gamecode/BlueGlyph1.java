package org.firstinspires.ftc.teamcode.gamecode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Felix;

/**
 * Created by Aila on 2017-12-07.
 */

@Autonomous


public class BlueGlyph1 extends AutoOpMode{

    private Felix cocoa = null;

    @Override
    public void runOp() throws InterruptedException {

        cocoa = new Felix();
        cocoa.init(hardwareMap);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        boolean glyphScored = false;

        /*
        cocoa.driveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cocoa.driveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        cocoa.releaseGlyph();

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive() && !glyphScored) {

            RelicRecoveryVuMark vumark = RelicRecoveryVuMark.from(relicTemplate);

            if (vumark == RelicRecoveryVuMark.RIGHT) {
                Log.i("Movement", "Right Vumark");
                telemetry.addData("Movement", "Right Vumark");

            }
            else if (vumark == RelicRecoveryVuMark.CENTER) {
                Log.i("Movement", "Right Vumark");
                telemetry.addData("Movement", "Centre Vumark");

            }
            else if (vumark == RelicRecoveryVuMark.LEFT){
                Log.i("Movement", "Left Vumark");
                telemetry.addData("Movement", "Left Vumark");

            }
            else {
                Log.i("Movement", "No Vumark");
                telemetry.addData("Movement", "No Vumark");

            }

        }

    }
}
