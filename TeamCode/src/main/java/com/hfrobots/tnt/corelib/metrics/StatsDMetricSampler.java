package com.hfrobots.tnt.corelib.metrics;

import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.metrics.sources.DcMotorCurrentMetricSource;
import com.hfrobots.tnt.corelib.metrics.sources.DcMotorPowerMetricSource;
import com.hfrobots.tnt.corelib.metrics.sources.OnOffButtonMetricSource;
import com.hfrobots.tnt.corelib.metrics.sources.RangeInputMetricSource;
import com.hfrobots.tnt.corelib.metrics.sources.Voltage12VMetricSource;
import com.hfrobots.tnt.corelib.metrics.sources.Voltage5VMetricSource;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.timgroup.statsd.NonBlockingStatsDClient;

import org.openftc.revextensions2.ExpansionHubEx;

import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import lombok.NonNull;

public class StatsDMetricSampler implements MetricsSampler {
    private final Set<GaugeMetricSource> gaugeSources = new LinkedHashSet<>();

    private final NonBlockingStatsDClient statsDClient;

    private final HardwareMap hardwareMap;

    private final List<ExpansionHubEx> expansionHubExes;

    public StatsDMetricSampler(HardwareMap hardwareMap, NinjaGamePad driverControls, NinjaGamePad operatorControls) {
        this.hardwareMap = hardwareMap;

        expansionHubExes = hardwareMap.getAll(ExpansionHubEx.class);

        addAllByHardwareMap();

        addGamepad("drv", driverControls);
        addGamepad("opr", operatorControls);

        try {
            statsDClient = new NonBlockingStatsDClient("", "192.168.49.239", 8126);
        } catch (Exception ex) {
            throw new RuntimeException("Can't open statsd client", ex);
        }
    }

    @Override
    public void doSamples() {
        long beginSamplingTimeMs = System.currentTimeMillis();

        for (GaugeMetricSource gaugeSource : gaugeSources) {
            statsDClient.gauge(gaugeSource.getSampleName(), gaugeSource.getValue());
        }

        long endSamplingTimeMs = System.currentTimeMillis();

        statsDClient.gauge("metric_sample_time_ms", (endSamplingTimeMs - beginSamplingTimeMs));
    }

    @Override
    public void addSource(GaugeMetricSource metricSource) {
        gaugeSources.add(metricSource);
    }

    private void addAllByHardwareMap() {
        addDcMotors();
        addDcMotorCurrents();
        addVoltages();
    }

    @Override
    public void addGamepad(@NonNull String name, @NonNull NinjaGamePad gamepad) {
        // FIXME: Add all of the range inputs, buttons from the gamepad as metric sources

        RangeInput leftStickX = gamepad.getLeftStickX();
        addSource(new RangeInputMetricSource (leftStickX, name, "l_x"));
        RangeInput leftStickY = gamepad.getLeftStickY();
        addSource(new RangeInputMetricSource (leftStickY, name, "l_y"));

        RangeInput rightStickX = gamepad.getRightStickX();
        addSource(new RangeInputMetricSource (rightStickX, name, "r_x"));
        RangeInput rightStickY = gamepad.getLeftStickY();
        addSource(new RangeInputMetricSource (rightStickY, name, "r_y"));

        RangeInput leftTrigger = gamepad.getLeftTrigger();
        addSource(new RangeInputMetricSource (leftTrigger, name, "l_trigger"));
        RangeInput rightTrigger = gamepad.getRightTrigger();
        addSource(new RangeInputMetricSource (rightTrigger, name, "r_trigger"));

        OnOffButton leftBumper = gamepad.getLeftBumper();
        addSource(new OnOffButtonMetricSource (leftBumper, name, "l_bumper"));
        OnOffButton rightBumper = gamepad.getRightBumper();
        addSource(new OnOffButtonMetricSource (rightBumper, name, "r_bumper"));

        OnOffButton aButton = gamepad.getAButton();
        addSource(new OnOffButtonMetricSource (aButton , name, "a_btn"));
        OnOffButton bButton = gamepad.getBButton();
        addSource(new OnOffButtonMetricSource (bButton, name, "b_btn"));
        OnOffButton xButton = gamepad.getXButton();
        addSource(new OnOffButtonMetricSource (xButton , name, "x_btn"));
        OnOffButton yButton = gamepad.getYButton();
        addSource(new OnOffButtonMetricSource (yButton, name, "y_btn"));

        OnOffButton dpadUp = gamepad.getDpadUp();
        addSource(new OnOffButtonMetricSource (dpadUp , name, "dpad_up"));
        OnOffButton dpadDown = gamepad.getDpadDown();
        addSource(new OnOffButtonMetricSource (dpadDown , name, "dpad_down"));
        OnOffButton dpadLeft = gamepad.getDpadLeft();
        addSource(new OnOffButtonMetricSource (dpadLeft , name, "dpad_left"));
        OnOffButton dpadRight = gamepad.getDpadRight();
        addSource(new OnOffButtonMetricSource (dpadRight , name, "dpad_right"));
    }

    private void addDcMotors() {
        List<DcMotor> allMotors = hardwareMap.getAll(DcMotor.class);

        // FIXME how to deal with 2 expansion hubs, 01234, 01234
        for (DcMotor motor : allMotors) {
            GaugeMetricSource metricSource = new DcMotorPowerMetricSource(motor);
            addSource(metricSource);
        }
    }

    private void addDcMotorCurrents() {
        for (ExpansionHubEx hub : expansionHubExes) {
            for (int port = 0; port < 4; port++) {
                GaugeMetricSource metricSource = new DcMotorCurrentMetricSource(hub, port);
                addSource(metricSource);
            }
        }
    }

    private void addVoltages() {
        for (ExpansionHubEx hub : expansionHubExes) {
            GaugeMetricSource metricSource = new Voltage5VMetricSource(hub);
            addSource(metricSource);

            metricSource = new Voltage12VMetricSource(hub);
            addSource(metricSource);
        }
    }
}
