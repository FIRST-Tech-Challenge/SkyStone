package com.hfrobots.tnt.season1920;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A tele-op that runs our normal tele-op for the season, but
 * sends metrics without having to go through ChaosNinja
 */
@TeleOp(name="Metrics Demo", group="util")
@SuppressWarnings("unused")
public class MetricDemoTeleop extends SkystoneTeleop {
    @Override
    public void start() {
        super.start();

        setupMetricsSampler();
    }
}
