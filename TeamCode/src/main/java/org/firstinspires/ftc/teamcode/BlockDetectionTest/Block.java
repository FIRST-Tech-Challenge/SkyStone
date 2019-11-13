package org.firstinspires.ftc.teamcode.PreseasonTest;

public class Block {
    static double tlx;
    static double tly;
    static double brx;
    static double bry;
    static double centerX;
    static double centerY;
    static String label;
    Block(double TLX, double TLY, double BRX, double BRY, String LABEL){
        tlx = TLX;
        tly = TLY;
        brx = BRX;
        bry = BRY;
        label = LABEL;
        centerX = (tlx + brx)/2;
        centerY = (tly + bry)/2;
    }
}
