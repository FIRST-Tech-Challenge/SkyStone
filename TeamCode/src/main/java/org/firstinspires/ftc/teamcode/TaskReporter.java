package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class TaskReporter {
    static public void  report(ArrayList<RobotControl> taskList) {
        report("", taskList, true);
    }

    static void report(String indent, ArrayList<RobotControl> taskList, boolean ordered) {
        for(int i=0; i<taskList.size(); i++) {
            String line = indent + (ordered ? "" + (i+1) + "- ":"== ") + taskList.get(i);
            Logger.logFile(line);
            if (taskList.get(i) instanceof SequentialComboTask) {
                report(indent+ "  ", ((SequentialComboTask)taskList.get(i)).sqTaskList, true);
            }
            if (taskList.get(i) instanceof ParallelComboTask) {
                report(indent+ "  ", ((ParallelComboTask)taskList.get(i)).taskList, false);
            }
        }
    }
}
