package com.hfrobots.tnt.fakes;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FakeTelemetry implements Telemetry {
    private boolean outputTelemetry = false;

    public void setOutputTelemetry(boolean flag) {
        outputTelemetry = flag;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        if (outputTelemetry) {
            String telemetryLine = (caption != null ? caption : "") + " " + String.format(format, args);
            System.out.println("Telemetry: " + telemetryLine);
        }

        return null;
    }

    @Override
    public Item addData(String caption, Object value) {
        if (outputTelemetry) {
            String telemetryLine = (caption != null ? caption : "") + " " + value != null ? value.toString() : "";
            System.out.println("Telemetry: " + telemetryLine);
        }

        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return null;
    }

    @Override
    public boolean removeItem(Item item) {
        return false;
    }

    @Override
    public void clear() {

    }

    @Override
    public void clearAll() {

    }

    @Override
    public Object addAction(Runnable action) {
        return null;
    }

    @Override
    public boolean removeAction(Object token) {
        return false;
    }

    @Override
    public boolean update() {
        return false;
    }

    @Override
    public Line addLine() {
        return null;
    }

    @Override
    public Line addLine(String lineCaption) {
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        return false;
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) {

    }

    @Override
    public int getMsTransmissionInterval() {
        return 0;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {

    }

    @Override
    public String getItemSeparator() {
        return null;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {

    }

    @Override
    public String getCaptionValueSeparator() {
        return null;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {

    }

    @Override
    public Log log() {
        return null;
    }
}
