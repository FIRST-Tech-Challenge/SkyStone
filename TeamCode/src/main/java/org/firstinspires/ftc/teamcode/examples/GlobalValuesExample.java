package org.firstinspires.ftc.teamcode.examples;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

/**
 * Created by Alec Krawciw on 2017-10-24.
 */

public class GlobalValuesExample extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {
        RC.globalDouble("name");
        RC.globalBool("boolname");
    }
}
