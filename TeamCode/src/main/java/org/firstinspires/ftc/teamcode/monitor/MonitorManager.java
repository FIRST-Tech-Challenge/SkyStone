package org.firstinspires.ftc.teamcode.monitor;

import org.firstinspires.ftc.teamcode.DeviceMap;

import java.util.ArrayList;
import java.util.List;

public final class MonitorManager {
    private static List<IMonitor> monitors;
    public static void startAll(DeviceMap map) {
        monitors = new ArrayList<>();
        monitors.add(new MonitorIMU(map.getImu()));
        monitors.add(new MonitorCamera(map));

    }

    public static void stopAll() {
        for(IMonitor monitor : monitors)
            monitor.stop();
        monitors.clear();
        monitors = null;
    }
}
