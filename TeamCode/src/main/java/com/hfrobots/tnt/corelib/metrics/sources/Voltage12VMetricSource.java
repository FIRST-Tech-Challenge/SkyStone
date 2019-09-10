package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;

import org.openftc.revextensions2.ExpansionHubEx;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class Voltage12VMetricSource implements GaugeMetricSource {

    private final ExpansionHubEx expansionHubEx;

    private final String name;

    public Voltage12VMetricSource(ExpansionHubEx expansionHubEx) {
        this.expansionHubEx = expansionHubEx;
        int moduleAddress = expansionHubEx.getStandardModule().getModuleAddress();

        name = String.format("hub_%d_12V", moduleAddress);
    }

    @Override
    public String getSampleName() {
        return name;
    }

    @Override
    public double getValue() {
        // FIXME: There's a value you can use here from the expansionHubEx member variable
        return expansionHubEx.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
    }
}
