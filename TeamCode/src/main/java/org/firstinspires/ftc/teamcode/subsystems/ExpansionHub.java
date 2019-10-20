/*
 * Group of methods hidden from the FTC SDK that are available to REV Expansion hubs,
 * including motor current and battery voltage. Based on the OpenFTC library.
 */

/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxPhoneChargeControlCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusResponse;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ExpansionHub extends LynxController {

    // utility method
    public double getMotorCurrentDraw(DcMotorEx motor, CurrentDrawUnits units) {
        int port = motor.getPortNumber();
        LynxGetADCCommand.Channel channel = null;
        if (port == 0) {
            channel = LynxGetADCCommand.Channel.MOTOR0_CURRENT;
        }
        else if (port == 1) {
            channel = LynxGetADCCommand.Channel.MOTOR1_CURRENT;
        }
        else if (port == 2) {
            channel = LynxGetADCCommand.Channel.MOTOR2_CURRENT;
        }
        else if (port == 3) {
            channel = LynxGetADCCommand.Channel.MOTOR3_CURRENT;
        }
        if (channel == null) {
            throw new IllegalArgumentException("No motor on port " + port + " exists on this hub.");
        }
        LynxGetADCCommand command = new LynxGetADCCommand(lynxModule, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            int ma = response.getValue();
            if (units == CurrentDrawUnits.MILLIAMPS) {
                return ma;
            }
            else if (units == CurrentDrawUnits.AMPS) {
                return ma / 1000.0;
            }
        } catch (InterruptedException | RuntimeException | LynxNackException e) {
            handleException(e);
        }
        return -1;
    }

    /**
     * Not guaranteed behavior, merely asks if the motor "has lost counts" and returns the opposite
     * @param motor th motor to check
     * @return
     */
    public boolean encoderWorks(DcMotorEx motor) {
        int port = motor.getPortNumber();
        LynxGetModuleStatusCommand command = new LynxGetModuleStatusCommand(lynxModule);
        try {
            LynxGetModuleStatusResponse response = command.sendReceive();
            return !response.hasMotorLostCounts(port);
        } catch (Exception e) {
            handleException(e);
        }
        return false;
    }

    private static HashMap<String, ExpansionHub> availableHubs = new HashMap<>();
    private static HardwareMap hardwareMap = null;

    private final LynxModule lynxModule;

    public static synchronized HashMap<String, ExpansionHub> getAvailableHubs(HardwareMap hardwareMap) {
        return hardwareMap.equals(ExpansionHub.hardwareMap) ? availableHubs : (availableHubs = findHubs(ExpansionHub.hardwareMap = hardwareMap));
    }

    private static HashMap<String, ExpansionHub> findHubs(HardwareMap hardwareMap) {
        List<LynxDcMotorController> map = hardwareMap.getAll(LynxDcMotorController.class);
        HashMap<String, ExpansionHub> out = new HashMap<>();
        for (LynxDcMotorController motorController : map) {
            LynxModule module;
            module = controllerToModule(motorController);
            Set<String> names = hardwareMap.getNamesOf(motorController);
            if (names.size() > 1) {
                throw new IllegalArgumentException("A hub in the hardware map exists with multiple names.");
            }
            out.put(names.iterator().next(), new ExpansionHub(hardwareMap.appContext, module));
        }
        return out;
    }

    private static LynxModule controllerToModule(LynxController controller) {
        Field moduleField;
        try {
            moduleField = LynxController.class.getDeclaredField("module");
            moduleField.setAccessible(true);
            return (LynxModule) moduleField.get(controller);
        } catch (ReflectiveOperationException | ClassCastException ex) {
            throw new IllegalArgumentException();
        }
    }

    private ExpansionHub (Context context, LynxModule lynxModule) {
        super(context, lynxModule);
        this.lynxModule = lynxModule;
    }

    @Override
    protected String getTag() {
        return "Expansion Hub";
    }

    @Override
    public String getDeviceName() {
        return lynxModule.getDeviceName();
    }

    public void setStatusLedColor(int r, int g, int b) {
        if(r > 255 || g > 255 || b > 255) {
            throw new IllegalArgumentException();
        }
        setStatusLedColor((byte)r, (byte)g, (byte)b);
    }

    public synchronized void setStatusLedColor(byte r, byte g, byte b) {
        LynxSetModuleLEDColorCommand colorCommand = new LynxSetModuleLEDColorCommand(lynxModule, r, g, b);
        try {
            colorCommand.send();
        } catch (InterruptedException | LynxNackException e) {
            handleException(e);
        }
    }

    public synchronized double getTotalCurrentDraw(CurrentDrawUnits units) {
        LynxGetADCCommand command = new LynxGetADCCommand(lynxModule,
                LynxGetADCCommand.Channel.BATTERY_CURRENT, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            int ma = response.getValue();
            if (units == CurrentDrawUnits.MILLIAMPS) {
                return ma;
            }
            else if (units == CurrentDrawUnits.AMPS) {
                return ma/1000d;
            }
        } catch (InterruptedException | RuntimeException | LynxNackException e) {
            handleException(e);
        }
        return -1;
    }

    public synchronized void setPhoneChargeEnabled(boolean chargeEnabled) {
        LynxPhoneChargeControlCommand controlCommand = new LynxPhoneChargeControlCommand(lynxModule, chargeEnabled);
        try {
            controlCommand.send();
        } catch (InterruptedException | LynxNackException e) {
            handleException(e);
        }
    }

    public synchronized double voltage5vPorts(VoltageUnits units) {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.FIVE_VOLT_MONITOR;
        LynxGetADCCommand command = new LynxGetADCCommand(lynxModule, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            int mv = response.getValue();
            if (units == VoltageUnits.MILLIVOLTS) {
                return mv;
            }
            else if (units == VoltageUnits.VOLTS) {
                return mv/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e) {
            handleException(e);
        }
        return -1;
    }

    public enum VoltageUnits {
        MILLIVOLTS,
        VOLTS
    }

    public synchronized double voltageBattery(VoltageUnits units) {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.BATTERY_MONITOR;
        LynxGetADCCommand command = new LynxGetADCCommand(lynxModule, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            int mv = response.getValue();
            if (units == VoltageUnits.MILLIVOLTS) {
                return mv;
            }
            else if (units == VoltageUnits.VOLTS) {
                return mv/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e) {
            handleException(e);
        }
        return -1;
    }

    public enum CurrentDrawUnits {
        MILLIAMPS,
        AMPS
    }

    public synchronized String getFirmwareVersion() {
        String inputstring = lynxModule.getFirmwareVersionString();
        if (inputstring.equals("unknown firmware") || inputstring.equals("firmware version unavailable")) {
            return inputstring;
        }

        int majVer = regexField("Maj", inputstring);
        int minVer = regexField("Min", inputstring);
        int engVer = regexField("Eng", inputstring);
        return String.format("%s.%s.%s", majVer, minVer, engVer);
    }

    private int regexField(String thing, String input) {
        Pattern pattern = Pattern.compile(thing + ":\\ \\d+");
        Matcher matcher = pattern.matcher(input);
        matcher.find();

        String string = matcher.group();

        return regexNum(string);
    }

    private int regexNum(String str) {
        Pattern pattern = Pattern.compile("\\d+");
        Matcher matcher = pattern.matcher(str);
        matcher.find();

        return Integer.parseInt(matcher.group());
    }
}
