package org.firstinspires.ftc.teamcode.Utilities.Control;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.Supplier;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public interface LEDRiver {
    Class<? extends LEDRiverImpl> IMPL = LEDRiverImpl.class;

    LEDRiver setHide(boolean hidden);
    LEDRiver setMode(Mode mode);
    LEDRiver setLEDMode(LEDMode ledMode);
    LEDRiver setColorDepth(ColorDepth colorDepth);
    LEDRiver setLEDCount(int numLEDs);
    LEDRiver setColor(Color color);
    LEDRiver setColor(int androidColor);
    LEDRiver setColor(int index, Color color);
    LEDRiver setColor(int index, int androidColor);
    LEDRiver setPattern(Pattern pattern);
    LEDRiver setBrightness(double dimmer);
    void apply();
    void apply(boolean force);
    void reset();
    void defaults();
    void save();


    int DEFAULT_SLAVE_ADDRESS = 0x40;
    enum Register {
        REG_CTRLA(0x00),
        REG_CTRLB(0x01),
        REG_LED_COUNT_LSB(0x02),
        REG_LED_COUNT_MSB(0x03),
        REG_DATA_START(0x04),
        REG_PATTERN_ID(0x10),
        REG_PATTERN_SETTINGS_START(0x11);

        int val;

        Register(int val) {
            this.val = val;
        }
    }


    enum Mode {
        SOLID(0),
        INDIVIDUAL(1),
        PATTERN(2);

        final int id;

        Mode(int id) {
            this.id = id;
        }
    }

    enum LEDMode {
        RGB(0, false),
        RGB_NW(1, false),
        RGBW(2, true),
        DOTSTAR(3, false);

        final int id;
        final boolean w;

        LEDMode(int id, boolean w) {
            this.id = id;
            this.w = w;
        }
    }

    enum ColorDepth {
        BIT_8(0, 400),
        BIT_16(1, 200),
        BIT_24(2, 133),
        BIT_32(3, 100);

        final int id;
        final int max_led;

        ColorDepth(int id, int max_led) {
            this.id = id;
            this.max_led = max_led;
        }
    }

    class Color {
        private int r, g, b, w;

        public Color(int r, int g, int b, int w) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.w = w;
        }

        public Color(int androidColor) {
            this.r = android.graphics.Color.red(androidColor);
            this.g = android.graphics.Color.green(androidColor);
            this.b = android.graphics.Color.blue(androidColor);
            this.w = 1 - android.graphics.Color.alpha(androidColor);
        }
    }

    class PatternID<T extends Pattern> {
        final int id;
        private final Supplier<T> builder;

        PatternID(int id, Supplier<T> builder) {
            this.id = id;
            this.builder = builder;
        }

        public T builder() {
            return builder.get();
        }
    }

    abstract class Pattern<T extends Pattern> {
        PatternID patternID;
        byte[] settings;

        private Pattern(PatternID patternID) {
            this.patternID = patternID;
            this.settings = new byte[15];
        }

        public T setSetting(int index, byte value) {
            settings[index] = value;
            return (T) this;
        }

        public T setSetting(int index, int value) {
            settings[index] = (byte) value;
            return (T) this;
        }

        public T setSpeed(int cycles) {
            setSetting(0, cycles & 0xFF);
            setSetting(1, (cycles >> 8) & 0xFF);
            return (T) this;
        }

        public static final PatternID<Pattern_Strobe> STROBE = new PatternID<>(0, new Supplier<Pattern_Strobe>() {public Pattern_Strobe get() { return new Pattern_Strobe(); }});
        public static final PatternID<Pattern_Heartbeat> HEARTBEAT = new PatternID<>(1, new Supplier<Pattern_Heartbeat>() { public Pattern_Heartbeat get() { return new Pattern_Heartbeat(); }});
        public static final PatternID<Pattern_Breathing> BREATHING = new PatternID<>(2, new Supplier<Pattern_Breathing>() { public Pattern_Breathing get() { return new Pattern_Breathing(); }});
        public static final PatternID<Pattern_Running> RUNNING = new PatternID<>(3, new Supplier<Pattern_Running>() { public Pattern_Running get() { return new Pattern_Running(); }});
        public static final PatternID<Pattern_Bouncing> BOUNCING = new PatternID<>(4, new Supplier<Pattern_Bouncing>() { public Pattern_Bouncing get() { return new Pattern_Bouncing(); }});
        public static final PatternID<Pattern_Theatre_Running> THEATRE_RUNNING = new PatternID<>(5, new Supplier<Pattern_Theatre_Running>() { public Pattern_Theatre_Running get() { return new Pattern_Theatre_Running(); }});
        public static final PatternID<Pattern_Theatre_Bouncing> THEATRE_BOUNCING = new PatternID<>(6, new Supplier<Pattern_Theatre_Bouncing>() { public Pattern_Theatre_Bouncing get() { return new Pattern_Theatre_Bouncing(); }});
        public static final PatternID<Pattern_Color_Wheel> COLOR_WHEEL = new PatternID<>(7, new Supplier<Pattern_Color_Wheel>() { public Pattern_Color_Wheel get() { return new Pattern_Color_Wheel(); }});
    }

    class Pattern_Strobe extends Pattern<Pattern_Strobe> {
        private Pattern_Strobe() {
            super(Pattern.STROBE);
            setSpeed(700);
        }
    }

    class Pattern_Heartbeat extends Pattern<Pattern_Heartbeat> {
        private Pattern_Heartbeat() {
            super(Pattern.HEARTBEAT);
            setSpeed(20);
            setDelay(5);
        }

        public Pattern_Heartbeat setDelay(int cycles) {
            setSetting(2, cycles & 0xFF);
            return this;
        }
    }

    class Pattern_Breathing extends Pattern<Pattern_Breathing> {
        private Pattern_Breathing() {
            super(Pattern.BREATHING);
            setSpeed(20);
            setDelay(5);
        }

        public Pattern_Breathing setDelay(int cycles) {
            setSetting(2, cycles & 0xFF);
            return this;
        }
    }

    class Pattern_Running extends Pattern<Pattern_Running> {
        private Pattern_Running() {
            super(Pattern.RUNNING);
            setSpeed(10);
            setIncrement(1);
            setLength(5);
            setDirection(false);
            setDelay(5);
        }

        public Pattern_Running setIncrement(int increment) {
            setSetting(2, increment & 0xFF);
            return this;
        }

        public Pattern_Running setLength(int length) {
            setSetting(3, length & 0xFF);
            return this;
        }

        public Pattern_Running setDirection(boolean movingAway) {
            setSetting(4, movingAway ? 0 : 1);
            return this;
        }

        public Pattern_Running setDelay(int cycles) {
            setSetting(5, cycles & 0xFF);
            return this;
        }
    }

    class Pattern_Bouncing extends Pattern<Pattern_Bouncing> {
        private Pattern_Bouncing() {
            super(Pattern.BOUNCING);
            setSpeed(10);
            setIncrement(1);
            setLength(5);
        }

        public Pattern_Bouncing setIncrement(int increment) {
            setSetting(2, increment & 0xFF);
            return this;
        }

        public Pattern_Bouncing setLength(int length) {
            setSetting(3, length & 0xFF);
            return this;
        }
    }

    class Pattern_Theatre_Running extends Pattern<Pattern_Theatre_Running> {
        private Pattern_Theatre_Running() {
            super(Pattern.THEATRE_RUNNING);
            setSpeed(200);
            setDirection(true);
            setLength(5);
            setSpacing(3);
        }

        public Pattern_Theatre_Running setDirection(boolean movingAway) {
            setSetting(2, movingAway ? 0 : 1);
            return this;
        }

        public Pattern_Theatre_Running setLength(int length) {
            setSetting(3, length & 0xFF);
            return this;
        }

        public Pattern_Theatre_Running setSpacing(int spacing) {
            setSetting(4, spacing & 0xFF);
            return this;
        }
    }

    class Pattern_Theatre_Bouncing extends Pattern<Pattern_Theatre_Bouncing> {
        private Pattern_Theatre_Bouncing() {
            super(Pattern.THEATRE_BOUNCING);
            setSpeed(200);
            setLength(5);
            setSpacing(3);
            setReverseSets(4);
        }

        public Pattern_Theatre_Bouncing setLength(int length) {
            setSetting(2, length & 0xFF);
            return this;
        }

        public Pattern_Theatre_Bouncing setSpacing(int spacing) {
            setSetting(3, spacing & 0xFF);
            return this;
        }

        public Pattern_Theatre_Bouncing setReverseSets(int sets) {
            setSetting(4, sets & 0xFF);
            return this;
        }
    }

    class Pattern_Color_Wheel extends Pattern<Pattern_Color_Wheel> {
        private Pattern_Color_Wheel() {
            super(Pattern.COLOR_WHEEL);
            setSpeed(20);
            setDirection(true);
            setLength(2);
            setMinHue(130);
            setMaxHue(270);
            setBrightness(0);
        }

        public Pattern_Color_Wheel setDirection(boolean movingAway) {
            setSetting(2, movingAway ? 1 : 0);
            return this;
        }

        public Pattern_Color_Wheel setLength(int length) {
            setSetting(3, length & 0xFF);
            return this;
        }

        public Pattern_Color_Wheel setMinHue(double minHue) {
            int minHue_norm = (int) Math.round(minHue / 360 * 255);
            setSetting(4, minHue_norm & 0xFF);
            return this;
        }

        public Pattern_Color_Wheel setMaxHue(double maxHue) {
            int maxHue_norm = (int) Math.round(maxHue / 360 * 255);
            setSetting(5, maxHue_norm & 0xFF);
            return this;
        }

        public Pattern_Color_Wheel setBrightness(int brightnessShift) {
            setSetting(6, brightnessShift);
            return this;
        }
    }

    @I2cDeviceType
    @DeviceProperties(name = "LEDRiver", description = "RGB LED Controller from Next Tech Robotics", xmlTag = "LEDRIVER")
    class LEDRiverImpl extends I2cDeviceSynchDevice<I2cDeviceSynch> implements LEDRiver {

        public LEDRiverImpl(I2cDeviceSynch i2cDeviceSynch) {
            super(i2cDeviceSynch, true);

            super.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_SLAVE_ADDRESS));
            super.deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(0x00, 26, I2cDeviceSynch.ReadMode.ONLY_ONCE));
            super.deviceClient.engage();
        }

        protected void writeColor(Color color, ByteBuffer buffer) {
            byte ob0 = 0, ob1 = 0, ob2 = 0, ob3 = 0;
            switch (colorDepth) {
                case BIT_8:
                    if(ledMode.w) {
                        ob0 |= ((int) Math.round(color.r * brightness / 255.0 * 3)) << 6;
                        ob0 |= ((int) Math.round(color.g * brightness / 255.0 * 3)) << 4;
                        ob0 |= ((int) Math.round(color.b * brightness / 255.0 * 3)) << 2;
                        ob0 |= ((int) Math.round(color.w * brightness / 255.0 * 3)) << 0;
                    } else {
                        ob0 |= ((int) Math.round(color.r * brightness / 255.0 * 7)) << 5;
                        ob0 |= ((int) Math.round(color.g * brightness / 255.0 * 7)) << 2;
                        ob0 |= ((int) Math.round(color.b * brightness / 255.0 * 3)) << 0;
                    }
                    buffer.put(ob0);
                    break;
                case BIT_16:
                    if(ledMode.w) {
                        ob1 |= ((int) Math.round(color.r * brightness / 255.0 * 15)) << 4;
                        ob1 |= ((int) Math.round(color.g * brightness / 255.0 * 15)) << 0;
                        ob0 |= ((int) Math.round(color.b * brightness / 255.0 * 15)) << 4;
                        ob0 |= ((int) Math.round(color.w * brightness / 255.0 * 15)) << 0;
                    } else {
                        int val = ((((int) (color.r * brightness))>>3) << 11) | ((((int) (color.g * brightness))>>2) << 5) | (((int) (color.b * brightness))>>3);
                        ob0 = (byte) (val & 0xFF);
                        ob1 = (byte) ((val >> 8) & 0xFF);
                    }
                    buffer.put(ob0);
                    buffer.put(ob1);
                    break;
                case BIT_24:
                    if(ledMode.w) {
                        ob2 |= ((int) Math.round(color.r * brightness / 255.0 * 63)) << 2;
                        ob2 |= ((int) Math.round(color.g * brightness / 255.0 * 63)) >> 4;
                        ob1 |= ((int) Math.round(color.g * brightness / 255.0 * 63)) << 4;
                        ob1 |= ((int) Math.round(color.b * brightness / 255.0 * 63)) >> 2;
                        ob0 |= ((int) Math.round(color.b * brightness / 255.0 * 63)) << 6;
                        ob0 |= ((int) Math.round(color.w * brightness / 255.0 * 63)) << 0;
                    } else {
                        ob2 |= (int) Math.round(color.r * brightness);
                        ob1 |= (int) Math.round(color.g * brightness);
                        ob0 |= (int) Math.round(color.b * brightness);
                    }
                    buffer.put(ob0);
                    buffer.put(ob1);
                    buffer.put(ob2);
                    break;
                case BIT_32:
                    if(ledMode.w) {
                        ob3 |= (int) Math.round(color.r * brightness);
                        ob2 |= (int) Math.round(color.g * brightness);
                        ob1 |= (int) Math.round(color.b * brightness);
                        ob0 |= (int) Math.round(color.w * brightness);
                    } else {
                        throw new IllegalStateException("LEDRIVER: 32 bit color is not available for LED Modes other than 2.");
                    }
                    buffer.put(ob0);
                    buffer.put(ob1);
                    buffer.put(ob2);
                    buffer.put(ob3);
                    break;
            }
        }

        private int slaveAddressOffset = 0;
        private boolean hidden = false;
        private Mode mode = Mode.SOLID;
        private LEDMode ledMode = LEDMode.RGB;
        private ColorDepth colorDepth = ColorDepth.BIT_24;
        private int ledCount = 60;
        private double brightness = 1.0;
        private Color[] colorMatrix = new Color[400];
        private Pattern pattern = Pattern.STROBE.builder();

        private byte[] lastData;
        private byte lastctrla = 0;

        {
            Color initColor = new Color(0, 0, 0, 0);
            for(int i = 0; i < colorMatrix.length; i++) {
                colorMatrix[i] = initColor;
            }
        }

        @Override
        public LEDRiver setHide(boolean hidden) {
            this.hidden = hidden;
            return this;
        }

        @Override
        public LEDRiver setMode(Mode mode) {
            this.mode = mode;
            return this;
        }

        @Override
        public LEDRiver setLEDMode(LEDMode ledMode) {
            this.ledMode = ledMode;
            return this;
        }

        @Override
        public LEDRiver setColorDepth(ColorDepth colorDepth) {
            this.colorDepth = colorDepth;
            return this;
        }

        @Override
        public LEDRiver setLEDCount(int numLEDs) {
            this.ledCount = numLEDs;
            return this;
        }

        @Override
        public LEDRiver setColor(Color color) {
            setColor(0, color);
            return this;
        }

        @Override
        public LEDRiver setColor(int androidColor) {
            setColor(0, androidColor);
            return this;
        }

        @Override
        public LEDRiver setColor(int index, Color color) {
            colorMatrix[index] = color;
            return this;
        }

        @Override
        public LEDRiver setColor(int index, int androidColor) {
            setColor(index, new Color(androidColor));
            return this;
        }

        @Override
        public LEDRiver setPattern(Pattern pattern) {
            this.pattern = pattern;
            return this;
        }

        @Override
        public LEDRiver setBrightness(double dimmer) {
            this.brightness = dimmer;
            return this;
        }

        @Override
        public void apply() {
            apply(false);
        }

        private byte calculateCTRLA(boolean apply, boolean reset, boolean defaults, boolean save) {
            byte ctrla = 0;
            ctrla |= mode.id << 5;
            if(save) ctrla |= 1 << 4;
            if(reset) ctrla |= 1 << 3;
            if(defaults) ctrla |= 1 << 2;
            if(hidden) ctrla |= 1 << 1;
            if(apply) ctrla |= 1 << 0;
            return ctrla;
        }

        private byte calculateCTRLA(boolean apply) {
            return calculateCTRLA(apply, false, false, false);
        }

        private byte calculateCTRLA() {
            return calculateCTRLA(true);
        }

        private byte calculateCTRLB() {
            byte ctrlb = 0;
            ctrlb |= slaveAddressOffset << 4;
            ctrlb |= ledMode.id << 2;
            ctrlb |= colorDepth.id << 0;
            return ctrlb;
        }

        @Override
        public void apply(boolean force) {
            // Assemble data
            byte ctrla = calculateCTRLA();
            byte ctrlb = calculateCTRLB();
            byte ledsl = (byte) (ledCount & 0xFF);
            byte ledsh = (byte) ((ledCount >> 8) & 0xFF);

            final ByteBuffer byteBuffer = ByteBuffer.allocate(403);
            byteBuffer.put(ctrlb);
            byteBuffer.put(ledsl);
            byteBuffer.put(ledsh);

            switch(mode) {
                case SOLID:
                    writeColor(colorMatrix[0], byteBuffer);

                    break;
                case INDIVIDUAL:
                    // Verification
                    if(ledCount > colorDepth.max_led) {
                        throw new IllegalArgumentException("LED count too large. " + ledCount + " > " + colorDepth.max_led + " for " + colorDepth.name());
                    }

                    for (int i = 0; i < ledCount; i++) {
                        writeColor(colorMatrix[i], byteBuffer);
                    }
                    break;
                case PATTERN:
                    for (int i = 0; i < 3; i++) {
                        writeColor(colorMatrix[i], byteBuffer);
                    }
                    byteBuffer.position(15);
                    byteBuffer.put((byte) pattern.patternID.id);
                    byteBuffer.put(pattern.settings);
                    break;
            }

            byte[] byteArray = Arrays.copyOf(byteBuffer.array(), byteBuffer.position());
            if(Arrays.equals(byteArray, lastData) && ctrla == lastctrla && !force) {
                return;
            }

            lastData = byteArray;
            lastctrla = ctrla;

            bulkWrite(ctrla, byteArray);
        }

        @Override
        public void reset() {
            write(Register.REG_CTRLA.val, 1 << 3);
        }

        @Override
        public void defaults() {
            write(Register.REG_CTRLA.val, 1 << 2);
        }

        @Override
        public void save() {
            write(Register.REG_CTRLA.val, (1 << 4) | (mode.id << 5));
        }

        @Override
        protected boolean doInitialize() {
            return true;
        }

        @Override
        public Manufacturer getManufacturer() {
            return Manufacturer.Other;
        }

        @Override
        public String getDeviceName() {
            return "Next Tech LEDRiver RGB LED Controller";
        }

        protected void write(int register, int val) {
            this.deviceClient.write(register & 0xFF, new byte[] {(byte) ((register >> 8) & 0xFF), (byte) val});
        }

        protected void bulkWrite(byte ctrla, byte[] data) {
            int register = 0x01;
            for(byte[] arr : divideArray(data, 98)) {
                byte[] chunk = new byte[arr.length + 1];
                System.arraycopy(arr, 0, chunk, 1, arr.length);
                arr[0] = (byte) ((register >> 8) & 0xFF);
                deviceClient.write(register & 0xFF, chunk, I2cWaitControl.WRITTEN);
                try {
                    if(arr.length > 50) {
                        Thread.sleep(5);
                    }
                } catch (InterruptedException ex) {
                    ex.printStackTrace();
                    Thread.currentThread().interrupt();
                }
                register += arr.length;
            }
            deviceClient.write(0, new byte[] {0, ctrla});
        }

        private static List<byte[]> divideArray(byte[] source, int chunkSize) {
            List<byte[]> result = new ArrayList<>();
            int start = 0;
            while(start < source.length) {
                int end = Math.min(source.length, start + chunkSize);
                result.add(Arrays.copyOfRange(source, start, end));
                start += chunkSize;
            }
            return result;
        }
    }
}