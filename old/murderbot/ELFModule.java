package org.eastsideprep.murderbot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

import static android.os.SystemClock.sleep;


/**
 * Created by gmein on 2/13/2018.
 */
// I2CSensor is deprecated, see https://github.com/ftctechnh/ftc_app/wiki/Writing-an-I2C-Driver to fix
@I2cSensor(name = "EPS Laser Tag Module", description = "Laser cannon turret and hit detector", xmlTag = "ELF01")
public class ELFModule extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    final int ELF_I2C_ADDRESS = 0x47;

    /* constants from Arduino file
            #define MANUFACTURER_CODE_REGISTER  1
            #define RESERVED_REGISTER           2
            #define SENSOR_CODE_REGISTER        3
            #define HITS_REGISTER               10
            #define ENERGY_REGISTER             11
            #define LEVEL_REGISTER              12
            #define STATE_NEUTRAL               100
            #define COMMAND_FIRE_LASER          101
            #define COMMAND_SET_LASER_HEADING   102
            #define COMMAND_CALIBRATE_LEVEL     103
    */

    private enum Register {
        MANUFACTURER_CODE(1),
        RESERVED(2),
        SENSOR_CODE(3),
        HITS(10),
        ENERGY(11),
        LEVEL(12),
        FIRE_LASER(101),
        SET_LASER_HEADING(102),
        CALIBRATE_LEVEL(4);

        public int code;

        Register(int code) {
            this.code = code;
        }
    }


    @Override
    public Manufacturer getManufacturer() {

        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {

        return "EPS Laser Tag Module";
    }

    public ELFModule(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(ELF_I2C_ADDRESS));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void fire(int ms) {
        ELFCommand(Register.FIRE_LASER, (byte)Math.min(ms,255));
    }
    public int getHitCount() {
        return ELFRead(Register.HITS);
    }


    private void ELFCommand(Register commandRegister, byte parameter) {
        byte[] b = new byte[1];
        b[0] = parameter;
        this.deviceClient.write(commandRegister.code, b);
    }

    private byte ELFRead(Register register){
        return this.deviceClient.read(register.code, 1)[0];
    }

}
