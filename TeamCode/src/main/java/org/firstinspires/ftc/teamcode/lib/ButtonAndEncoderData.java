package org.firstinspires.ftc.teamcode.lib;

import android.util.SparseArray;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.ExpansionHub;

import java.lang.reflect.Field;

public class ButtonAndEncoderData {

    private SparseArray<LynxGetBulkInputDataResponse> responses;

    private ButtonAndEncoderData() {
        responses = new SparseArray<>();
    }

    public void addHubData(ExpansionHub hub) {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(hub.getModule());
        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            responses.append(hub.getModule().getModuleAddress(), response);
        } catch (Exception ex) {
            hub.handleException(ex);
        }
    }

    public void clear() {
        responses.clear();
    }

    public boolean isPressed(RevTouchSensor touchSensor) {
        try {
            Field controllerField = RevTouchSensor.class.getDeclaredField("digitalChannelController");
            Field portField = RevTouchSensor.class.getDeclaredField("physicalPort");
            controllerField.setAccessible(true);
            portField.setAccessible(true);
            LynxDigitalChannelController controller = (LynxDigitalChannelController) controllerField.get(touchSensor);
            Field moduleField = LynxController.class.getDeclaredField("module");
            moduleField.setAccessible(true);
            int port = (int) portField.get(touchSensor);
            return responses.get(((LynxModule) moduleField.get(controller)).getModuleAddress()).getDigitalInput(port);
        } catch (ReflectiveOperationException | ClassCastException ex) {
            throw new IllegalArgumentException();
        }
    }

    public long getCurrentPosition(DcMotorEx motor) {
        try {
            Field moduleField = LynxController.class.getDeclaredField("module");
            moduleField.setAccessible(true);
            return responses.get(((LynxModule) moduleField.get(motor.getController())).getModuleAddress())
                    .getEncoder(motor.getPortNumber());
        } catch (ReflectiveOperationException | ClassCastException ex) {
            throw new IllegalArgumentException();
        }
    }

    private static ButtonAndEncoderData latest;

    public static ButtonAndEncoderData getLatest() {
        return latest == null ? (latest = new ButtonAndEncoderData()) : latest;
    }

}
