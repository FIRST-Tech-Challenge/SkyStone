package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class ActionList {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    ArrayList<ActionClass> actionList;

    public ActionList(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR) {
        actionList = new ArrayList<>();
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
    }

}