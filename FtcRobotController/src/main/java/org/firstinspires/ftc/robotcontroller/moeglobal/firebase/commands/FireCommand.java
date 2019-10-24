package org.firstinspires.ftc.robotcontroller.moeglobal.firebase.commands;

import com.qualcomm.ftccommon.CommandList;
import com.qualcomm.robotcore.robocol.Command;

public class FireCommand {
    public String type;
    public String opMode;

    public FireCommand() {

    }

    public Command toRealCommand() {
        COMMAND_TYPE command_type = COMMAND_TYPE.valueOf(type);
        String toSend = command_type.equals(COMMAND_TYPE.INIT) ? CommandList.CMD_INIT_OP_MODE : CommandList.CMD_RUN_OP_MODE;
        return new Command(toSend, opMode);
    }

    public enum COMMAND_TYPE {
        INIT, RUN
    }
}
