package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;

import org.openftc.revextensions2.ExpansionHubEx;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class Voltage5VMetricSource implements GaugeMetricSource {

    private final ExpansionHubEx expansionHubEx;

    private final String name;

    public Voltage5VMetricSource(ExpansionHubEx expansionHubEx) {
        this.expansionHubEx = expansionHubEx;
        int moduleAddress = expansionHubEx.getStandardModule().getModuleAddress();

        name = String.format("hub_%d_5V", moduleAddress);
    }

    @Override
    public String getSampleName() {
        return name;
    }

    @Override
    public double getValue() {
        return expansionHubEx.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
    }
}
