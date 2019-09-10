package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class RangeInputMetricSource implements GaugeMetricSource {
    private final RangeInput rangeInput;

    private final String name;

    public RangeInputMetricSource(RangeInput rangeInput, String gamepadName, String name) {
        this.rangeInput = rangeInput;
        this.name = gamepadName + "_"+ name;
    }

    @Override
    public String getSampleName() {
        return name;
    }

    @Override
    public double getValue() {
        return rangeInput.getPosition();
    }
}
