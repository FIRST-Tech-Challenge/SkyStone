package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

@TeleOp(name = "Concept Tester", group = "none")
public class ExperimentalStuff extends OpMode {

    View relativeLayout;
    VoltageSensor batteryVoltage;
    AndroidTextToSpeech speaker;
    private int lastSpake;
    ElapsedTime timer;
    BNO055IMUImpl backupGyro1;
    AndroidGyroscope backupGyro2;
    double lastUpdate;
    double integral;
    Context myApp;
    SoundPlayer.PlaySoundParams params;
    boolean soundPlaying;

    @Override
    public void init() {
        batteryVoltage = hardwareMap.get(VoltageSensor.class, "battery");

        speaker = new AndroidTextToSpeech();
        speaker.initialize();

        myApp = hardwareMap.appContext;
        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        int relativeLayoutId = hardwareMap.appContext.getResources()
                .getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        double volts = batteryVoltage.getVoltage();
        if (volts < 11) {
            relativeLayout.setBackgroundColor(Color.RED);
        } else if (volts < 12) {
            relativeLayout.setBackgroundColor(Color.rgb(0xFF, 0x80, 0x00));
        } else if (volts < 12.5) {
            relativeLayout.setBackgroundColor(Color.YELLOW);
        } else {
            relativeLayout.setBackgroundColor(Color.TRANSPARENT);
        }
        telemetry.addData("Battery Voltage", volts);
        telemetry.update();

        backupGyro1 = hardwareMap.getAll(BNO055IMUImpl.class).get(0);
        backupGyro2 = hardwareMap.getAll(AndroidGyroscope.class).get(0);
        backupGyro2.setAngleUnit(AngleUnit.DEGREES);

        AppUtil.getInstance().showToast(UILocation.BOTH, "Program Initialized.");
    }

    @Override
    public void start() {
        relativeLayout.setBackgroundColor(Color.TRANSPARENT);
        AppUtil.getInstance().showToast(UILocation.BOTH, "Program started.");
        timer.reset();
        lastSpake = 30;
        integral = 0;
    }

    @Override
    public void loop() {

        if (30 - timer.seconds() < lastSpake) {
            speaker.speak(String.valueOf((int) (30 - timer.seconds())));
        }
        telemetry.addData("Speaking", speaker.isSpeaking());

        double rev = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double time = timer.seconds() - lastUpdate;
        lastUpdate = timer.seconds();
        integral += backupGyro2.getZ() * time;

        telemetry.addData("Rev Gyro", rev);
        telemetry.addData("Phone Gyro", integral);

        if (gamepad1.b && timer.seconds() > 30 && !speaker.isSpeaking()) {
            soundPlaying = true;
            SoundPlayer.getInstance().startPlaying(myApp,
                    myApp.getResources().getIdentifier("ss_laser_burst", "raw", myApp.getPackageName()), params, null,
                    new Runnable() {
                        public void run() {
                            soundPlaying = false;
                        }
                    }
            );
        }

        telemetry.update();
    }
}
