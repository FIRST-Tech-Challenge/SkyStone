package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class MotorVelocityMetricSource implements GaugeMetricSource {
    @Override
    public String getSampleName() {
        return null;
    }

    @Override
    public double getValue() {
        return 0;
    }
}
