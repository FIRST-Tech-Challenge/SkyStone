package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class SequentialComboTask implements RobotControl {

    transient RobotHardware robotHardware;
    transient RobotProfile robotProfile;

    int i = 0; //counter
    boolean completed;

    ArrayList<RobotControl> taskList = new ArrayList<>();

    public void prepare(){
        completed = false;
        i=0;
        if (taskList.size()>0) {
            taskList.get(0).prepare();
        }
    }

    public void execute() {
        if (taskList.size() > i) {
            taskList.get(i).execute();
            if (taskList.get(i).isDone()) {
                taskList.get(i).cleanUp();
                i++;
                if (taskList.size() > i) {
                    taskList.get(i).prepare();
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
        taskList = taskArray;
    }
}
