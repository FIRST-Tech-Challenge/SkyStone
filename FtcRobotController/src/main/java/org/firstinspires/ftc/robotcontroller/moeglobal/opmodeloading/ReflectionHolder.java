package org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndClass;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndInstance;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;

import java.lang.reflect.Field;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class ReflectionHolder {

    private static final List<String> justDefault = Collections.singletonList(OpModeManager.DEFAULT_OP_MODE_NAME);

    public static LinkedHashMap<String, OpModeMetaAndClass> opModeClasses;
    public static LinkedHashMap<String, OpModeMetaAndInstance> opModesInstances;
    public static Object opModesLock;

    public static void initReflection() {
        try {

            RegisteredOpModes instance = RegisteredOpModes.getInstance();
            Field opModeClasses1 = instance.getClass().getDeclaredField("opModeClasses");
            Field opModeInstances1 = instance.getClass().getDeclaredField("opModeInstances");
            Field opModesLock1 = instance.getClass().getDeclaredField("opModesLock");
            opModesLock1.setAccessible(true);
            opModeClasses1.setAccessible(true);
            opModeInstances1.setAccessible(true);
            opModeClasses = (LinkedHashMap<String, OpModeMetaAndClass>) opModeClasses1.get(instance);
            opModesInstances = (LinkedHashMap<String, OpModeMetaAndInstance>) opModeInstances1.get(instance);
            opModesLock = opModesLock1.get(instance);
        } catch (IllegalAccessException | NoSuchFieldException e) {
            e.printStackTrace();
        }
    }


    public static void replaceOpModes(Map<String, OpModeMetaAndClass> opModes) {
        synchronized (opModesLock) {
            clearOpModes();
            loadOpModes(opModes);
            Log.e("done", "opmodes");
        }
    }

    /*
     * synchronize on opModesLock
     */
    private static void loadOpModes(Map<String, OpModeMetaAndClass> opModes) {
        Log.e("placing", "opmodes");
        opModeClasses.putAll(opModes);
    }

    /*
     * synchronize on opModesLock
     */
    private static void clearOpModes() {
        opModeClasses.keySet().retainAll(justDefault);
        opModesInstances.keySet().retainAll(justDefault);
    }
}
