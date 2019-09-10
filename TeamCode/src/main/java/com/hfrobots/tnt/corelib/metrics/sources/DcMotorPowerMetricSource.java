package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;
import com.qualcomm.robotcore.hardware.DcMotor;

import lombok.EqualsAndHashCode;
import lombok.NonNull;

@EqualsAndHashCode
public class DcMotorPowerMetricSource implements GaugeMetricSource {
    private final DcMotor motor;

    private final String sampleName;

    public DcMotorPowerMetricSource(@NonNull final DcMotor motor) {
        this.motor = motor;

        int port = motor.getPortNumber();

        // FIXME: THis probably isn't unique
        sampleName = String.format("dcm_pow_%d", port);
    }

    @Override
    public String getSampleName() {
        return sampleName;
    }

    @Override
    public double getValue() {
        return motor.getPower();
    }
}
