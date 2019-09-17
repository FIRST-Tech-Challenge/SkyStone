package org.firstinspires.ftc.teamcode.autonomous;

public class ActionClass {

    private final String[] actions = {"forward", "backward", "left", "right", "rotate"};

    private int action;
    private double degree;

    public ActionClass(String action, double degree) {
        this.degree = degree;
        for (int i = 0; i < 5; i++) {
            if (action.equals(actions[i]))
                this.action = i;
        }
    }

}