package org.firstinspires.ftc.teamcode.monitor;

import org.firstinspires.ftc.teamcode.DeviceMap;

import java.util.ArrayList;
import java.util.List;

public final class MonitorManager {
    private static List<IMonitor> monitors;
    public static void startAll(DeviceMap map) {
        monitors = new ArrayList<>();
        if(map.getImu() != null) monitors.add(new MonitorIMU(map.getImu()));
        if(map.getVuforia() != null) monitors.add(new MonitorCamera(map));

        for(IMonitor mon : monitors)
            mon.start();
    }

    public static void stopAll() {
        if(monitors == null) return;
        for(IMonitor monitor : monitors)
            monitor.stop();
        monitors.clear();
        monitors = null;
    }

    public static <T extends IMonitor> T getMonitor(Class<T> monitorClasz) {
        if(monitors.size() == 0) throw new IllegalStateException("Monitors must not be empty");
        for(IMonitor monitor1 : monitors)
            if(monitor1.getClass() == monitorClasz)
                return (T) monitor1;
        throw new RuntimeException("getMonitor must not return null");
    }
}
