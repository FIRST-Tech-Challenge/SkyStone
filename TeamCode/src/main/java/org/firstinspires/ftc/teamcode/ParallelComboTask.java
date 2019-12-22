package org.firstinspires.ftc.teamcode;

/**
 * 2019.11.16
 * Created by Athena Z.
 *
 * lift and extend and move?
 */

import java.util.ArrayList;

public class ParallelComboTask implements RobotControl {
    String taskName = "ParallelComboTask";

    ArrayList<RobotControl> taskList = new ArrayList<>();
    boolean[] doneList; //update doneList to be true if the task is cleaned up; don't need to be isDone() anymore
    int counterDone; //+1 for every True in doneList

    @Override
    public void prepare() {
        doneList = new boolean[taskList.size()];
        counterDone = 0;

        for (int i = 0; i < taskList.size(); i++) {
            taskList.get(i).prepare();
            doneList[i] = false;
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < taskList.size(); i++) {
            if (doneList[i] == false) {
                RobotControl task = taskList.get(i);

                if (!task.isDone()) {
                    task.execute();
                } else {
                    Logger.logFile("parallel task completed:" + task);
                    task.cleanUp();

                    doneList[i] = true;
                    counterDone++;
                }
            }
        }
    }

    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return counterDone == taskList.size();
    }

    public void setTaskList(ArrayList<RobotControl> taskList) {
        this.taskList = taskList;
    }

    public void setTaskName(String taskName) {
        this.taskName = taskName;
    }

    public String toString() {
        return taskName;
    }
}