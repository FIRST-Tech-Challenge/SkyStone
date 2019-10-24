package org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.ftcrobotcontroller.R;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEDatabase;

import java.lang.ref.WeakReference;
import java.lang.reflect.InvocationTargetException;

import static org.firstinspires.ftc.robotcontroller.moeglobal.ActivityReferenceHolder.activityRef;

public class OpModeLoading {

    public static CodeStatus currentStatus = CodeStatus.External;

    public static void loadOpModes() {
        OpModesFrame.OpModeReadiedData instance = OpModesFrame.instance;
        if (instance == null) {
            activityRef.get().eventLoop.refreshUI();
            return;
        }
        DexHandler handler = new DexHandler(instance);
        ReflectionHolder.replaceOpModes(handler.getOpModes());
        MOEDatabase.sendSynced();
        activityRef.get().eventLoop.refreshUI();
        playInstalledSound();
    }

    private static void playInstalledSound() {
        SoundPlayer.getInstance().startPlaying(activityRef.get(), R.raw.firecode);
    }

    public static void init(FtcRobotControllerActivity activity) {

        activityRef = new WeakReference<>(activity);
        verifyStatus();
        ReflectionHolder.initReflection();
    }

    public static void verifyStatus() {
        try {
            Class<?> aClass = Class.forName("org.firstinspires.ftc.teamcode.external.pushcode.CodeStatus");
            aClass.getMethod("status").invoke(null);
            currentStatus = CodeStatus.Internal;
        } catch (ClassNotFoundException | IllegalAccessException | NoSuchMethodException | InvocationTargetException ignored) {
        }
    }

    public enum CodeStatus {Internal, External}

}
