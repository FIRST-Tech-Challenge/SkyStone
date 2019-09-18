package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ActionClass {

    private final String[] actions = {"forward", "backward", "left", "right", "rotate"};

    private int action;
    private double degree;

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    public ActionClass(String action, double degree, DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR) {
        this.degree = degree;
        for (int i = 0; i < 5; i++) {
            if (action.equals(actions[i]))
                this.action = i;
        }
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
    }

}