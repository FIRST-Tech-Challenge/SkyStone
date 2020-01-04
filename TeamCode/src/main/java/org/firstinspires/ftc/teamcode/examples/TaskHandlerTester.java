package org.firstinspires.ftc.teamcode.examples;

import android.util.Log;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;

/**
 * Created by FIXIT on 2017-04-02.
 */

public class TaskHandlerTester extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {

        waitForStart();

        TaskHandler.addLoopedTask("Tester", new Runnable() {
            @Override
            public void run() {
                Log.i("Checking", "Hello!");

//                Log.i("Checking!-", TaskHandler.pauseLocks.get("Tester") + "");
            }
        }, 10);

        sleep(1000);

        TaskHandler.pauseTask("Tester");

        sleep(500);

        TaskHandler.resumeTask("Tester");

        sleep(1000);

    }
}
