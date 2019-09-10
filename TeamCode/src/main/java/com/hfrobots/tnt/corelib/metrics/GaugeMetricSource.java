package com.hfrobots.tnt.corelib.metrics;

public interface GaugeMetricSource {
    String getSampleName();
    double getValue();
}
