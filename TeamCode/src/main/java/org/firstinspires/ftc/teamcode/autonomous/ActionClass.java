package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ActionClass {

    private static final String[] actions = {"forward", "backward", "left", "right", "rotate"};

    private static int action;
    private static double degree;

    static DcMotor motorFL;
    static DcMotor motorFR;
    static DcMotor motorBL;
    static DcMotor motorBR;

    static double calibFL;
    static double calibFR;
    static double calibBL;
    static double calibBR;

    public static void doAction(double fl, double fr, double bl, double br, DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, String a, double d) {

        motorFL = FL;
        motorFR = FR;
        motorBL = BL;
        motorBR = BR;

        calibFL = fl;
        calibFR = fr;
        calibBL = bl;
        calibBR = br;

        degree = d;
        for (int i = 0; i < 5; i++) {
            if (a.equals(actions[i]))
                action = i;
        }

        switch (action) {
            case 0:
                moveForward(degree);
                break;
            case 1:
                moveForward(-degree);
                break;
            case 2:
                straifLeft(degree);
                break;
            case 3:
                straifLeft(-degree);
                break;
            case 4:
                rotateLeft(degree);
                break;
        }
    }

    public static void moveForward(double power) {
        motorFL.setPower(calibFL * power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * power);
        motorBR.setPower(calibBR * power);
    }

    public static void rotateLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * -power);
        motorBR.setPower(calibBR * power);
    }

    public static void straifLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBR * power);
        motorBR.setPower(calibBR * -power);
    }

}