package teamcode.common;

import java.util.Timer;
import java.util.TimerTask;

public class TTTimer {

    private static Timer timer;

    static void init() {
        timer = new Timer();
    }

    public static void schedule(TimerTask task, double delaySeconds) {
        long delayMilis = secondsToMilis(delaySeconds);
        timer.schedule(task, delayMilis);
    }

    public static void scheduleAtFixedRate(TimerTask task, double periodSeconds) {
        long periodMilis = secondsToMilis(periodSeconds);
        timer.scheduleAtFixedRate(task, 0L, periodMilis);
    }

    private static long secondsToMilis(double seconds) {
        return (long) (seconds * 1000);
    }

    static void cancel() {
        timer.cancel();
    }

}
