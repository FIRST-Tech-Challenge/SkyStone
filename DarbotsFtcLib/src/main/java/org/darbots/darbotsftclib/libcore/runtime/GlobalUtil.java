package org.darbots.darbotsftclib.libcore.runtime;

import org.darbots.darbotsftclib.libcore.integratedfunctions.RobotLogger;

public class GlobalUtil {
    public static void addLog(String module, String captain, Object content, RobotLogger.LogLevel logLevel){
        if(GlobalRegister.runningOpMode != null){
            if(GlobalRegister.runningOpMode.getRobotCore() != null)
                GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog(module,captain,content, logLevel);
        }
    }
}
