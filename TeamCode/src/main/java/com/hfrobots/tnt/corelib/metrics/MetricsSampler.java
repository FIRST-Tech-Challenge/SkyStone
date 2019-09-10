package com.hfrobots.tnt.corelib.metrics;

import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface MetricsSampler {

    void doSamples();

    void addSource(GaugeMetricSource metricSource);

    void addGamepad(String name, NinjaGamePad gamepad);
}
