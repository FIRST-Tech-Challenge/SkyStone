package org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading;

import android.util.ArrayMap;
import androidx.core.content.ContextCompat;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import dalvik.system.DexClassLoader;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.moeglobal.FileUtils;
import org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading.OpModesFrame.OpModeReadiedData;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndClass;

import java.io.File;
import java.util.Map;

import static org.firstinspires.ftc.robotcontroller.moeglobal.ActivityReferenceHolder.activityRef;

public class DexHandler {

    Map<String, OpModeMetaAndClass> opModeMetaAndClassMap;

    public DexHandler(OpModeReadiedData instance) {
        this(instance.uuid, instance.opModes);

    }


    public DexHandler(File fireCodeFile, Map<String, OpModeMeta> opModes) {
        if (!fireCodeFile.exists()) throw new IllegalStateException("Missing file:" + fireCodeFile);
        FtcRobotControllerActivity context = activityRef.get();
        DexClassLoader classLoader = new DexClassLoader(fireCodeFile.getAbsolutePath(), ContextCompat.getCodeCacheDir(context).getAbsolutePath(), null, context.getClassLoader());
        opModeMetaAndClassMap = getOpModesFromClassLoader(classLoader, opModes);
    }

    public DexHandler(String uuid, Map<String, OpModeMeta> opModes) {
        this(FileUtils.getFireCodeFileFromUUID(uuid), opModes);

    }

    private Map<String, OpModeMetaAndClass> getOpModesFromClassLoader(DexClassLoader classLoader, Map<String, OpModeMeta> opModes) {
        Map<String, OpModeMetaAndClass> classedMap = new ArrayMap<>(opModes.size());
        for (Map.Entry<String, OpModeMeta> opModeMetaEntry : opModes.entrySet()) {
            String name = opModeMetaEntry.getValue().name;
            try {
                //noinspection unchecked
                Class<OpMode> opModeClass = (Class<OpMode>) classLoader.loadClass(opModeMetaEntry.getKey());
                classedMap.put(name, new OpModeMetaAndClass(opModeMetaEntry.getValue(), opModeClass));
            } catch (ClassNotFoundException e) {
                e.printStackTrace();
            }
        }
        return classedMap;
    }

    public Map<String, OpModeMetaAndClass> getOpModes() {
        return opModeMetaAndClassMap;
    }
}
