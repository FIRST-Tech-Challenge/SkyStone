package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;

import org.openftc.revextensions2.ExpansionHubEx;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class DcMotorCurrentMetricSource implements GaugeMetricSource {
    private final ExpansionHubEx expansionHubEx;

    private final int motorPort;

    private final String name;

    public DcMotorCurrentMetricSource(ExpansionHubEx expansionHubEx, int motorPort) {
        this.expansionHubEx = expansionHubEx;
        this.motorPort = motorPort;
        int moduleAddress = expansionHubEx.getStandardModule().getModuleAddress();

        name = String.format("hub_%d_dcm_curr_%d", moduleAddress, motorPort);

    }

    @Override
    public String getSampleName() {
        return name;
    }

    @Override
    public double getValue() {
        return expansionHubEx.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS, motorPort);
    }
}
