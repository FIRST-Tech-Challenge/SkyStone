package org.baconeers.common;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Created by shaun on 29/07/2017.
 */

public class ColorSensorThread extends BaconComponent implements Runnable {

    private final ReentrantLock m_lock = new ReentrantLock();
    private final ScheduledExecutorService m_executor;
    private final long m_delay;
    private final long m_period;
    private final TimeUnit m_timeUnit;
    private final ColorSensor m_colorSensor;

    public volatile int red = 0;
    public volatile int green = 0;
    public volatile int blue = 0;
    public volatile int intensity = 0;
    public volatile int hue = 0;

    public ColorSensorThread(BaconOpMode baconOpMode, ColorSensor colorSensor, int delay, int period, TimeUnit timeUnit) {
        super(baconOpMode);

        m_delay = delay;
        m_period = period;
        m_timeUnit = timeUnit;
        m_colorSensor = colorSensor;

        m_executor = Executors.newScheduledThreadPool(1);
    }

    public void onInit() {

    }

    public void onStart() {
        m_executor.scheduleWithFixedDelay(this, m_delay, m_period, m_timeUnit);
    }

    public void onStop() {
        m_executor.shutdown();
    }

    public void enableLed(boolean state) {
        m_lock.lock();
        try {
            m_colorSensor.enableLed(state);
        } finally {
            m_lock.unlock();
        }
    }

    @Override
    public void run() {
        // Read the color sensor values in to local variables
        m_lock.lock();
        try {
            int r = m_colorSensor.red();
            int g = m_colorSensor.green();
            int b = m_colorSensor.blue();
            int i = m_colorSensor.alpha();
            int h = m_colorSensor.argb();

            // Write them to the class variables.
            red = r;
            green = g;
            blue = b;
            intensity = i;
            hue = h;
        } finally {
            m_lock.unlock();
        }

    }
}
