package org.baconeers.common;

import org.baconeers.common.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Abstract class to use as parent to the class you will define to mirror a "saved configuration" on the Robot controller
 */
public abstract class RobotConfiguration {

    private Telemetry telemetry;

    /**
     * assign your class instance variables to the saved device names in the hardware map
     * @param hardwareMap
     * @param telemetry
     */
    abstract protected void init(HardwareMap hardwareMap, Telemetry telemetry);

    /**
     * accessor for the telemetry utility
     *
     * @return {@link Telemetry}
     */
    protected Telemetry getTelemetry() {
        return telemetry;
    }

    /**
     * setter method for telemetry utility
     * @param {@link TelemetryUtil}
     */
    protected void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Convenience method for reading the devce from the hardwareMap without having to check for exceptions
     * @param name name of device saved in the configuration file
     * @param hardwareDeviceMapping
     * @return
     */
    protected  HardwareDevice getHardwareOn(String name, Object hardwareDeviceMapping) {

        HardwareDevice hardwareDevice = null;
        try {
            HardwareMap.DeviceMapping<HardwareDevice> deviceMapping  =  (HardwareMap.DeviceMapping<HardwareDevice>) hardwareDeviceMapping;
            hardwareDevice = (HardwareDevice)deviceMapping.get(name);
        }
        catch (Throwable e)
        {
            try {
                ErrorUtil.handleCatchAllException(e, getTelemetry());
            } catch (InterruptedException e1) {
                DbgLog.msg(e.getLocalizedMessage());
            }

        }

        return hardwareDevice;
    }





}
