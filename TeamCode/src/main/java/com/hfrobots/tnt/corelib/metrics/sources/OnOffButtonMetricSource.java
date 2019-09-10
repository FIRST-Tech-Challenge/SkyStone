package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class OnOffButtonMetricSource implements GaugeMetricSource {
    private final OnOffButton onOffButton;

    private final String name;

    public OnOffButtonMetricSource(OnOffButton onOffButton, String gamepadName, String name) {
        this.onOffButton = onOffButton;
        this.name = gamepadName + "_" + name;
    }

    @Override
    public String getSampleName() {
        return name;
    }

    @Override
    public double getValue() {
        if(onOffButton.isPressed()){
            return 1;
        }

        return 0;
    }
}
