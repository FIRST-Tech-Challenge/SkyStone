package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.text.DecimalFormat;

public class SequentialComboTask implements RobotControl {

    transient RobotHardware robotHardware;
    transient RobotProfile robotProfile;
    String taskName = "SequentialComboTask";

    int i = 0; //counter
    boolean completed;

    ArrayList<RobotControl> sqTaskList = new ArrayList<>();

    public void prepare(){
        completed = false;
        i=0;
        if (sqTaskList.size()>0) {
            RobotControl robotControl = sqTaskList.get(0);
            robotControl.prepare();
        }
    }

    public void execute() {
        if (sqTaskList.size() > i) {
            sqTaskList.get(i).execute();
            //Logger.logFile("Sequential Task Size = " + taskList.size());
            if (sqTaskList.get(i).isDone()) {
                Logger.logFile("Sequential  Task Is Done: " + sqTaskList.get(i));
                sqTaskList.get(i).cleanUp();
                i++;
                if (sqTaskList.size() > i) {
                    sqTaskList.get(i).prepare();
                }
                else {
                    completed = true;
                }
            }
        }
    }

    public void cleanUp(){
        i = 0;
    }

    public boolean isDone() {
        return completed;
    }

    public void setTaskList(ArrayList<RobotControl> taskArray) {
        sqTaskList = taskArray;
    }

    public void setTaskName(String taskName) {
        this.taskName = taskName;
    }

    public String toString() {
        return taskName;
    }

    public void addTask(RobotControl task) {
        sqTaskList.add(task);
    }
}
