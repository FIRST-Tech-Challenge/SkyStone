package org.firstinspires.ftc.teamcode.util;


/**
 * Created by FIXIT on 16-11-20.
 */
public class PID {

    public double kProp = 0;
    public double kDeriv = 0;
    public double kIntegr = 0;

    private double integrStore = 0;
    private double derivStore = 0;
    private Type algorithmType = Type.PID;

    public PID (Type type, double... gains) {

        this.algorithmType = type;

        switch (type) {
            case PID:
                kProp = gains[0];
                kIntegr = gains[1];
                kDeriv = gains[2];
                break;
            case ID:
                kIntegr = gains[0];
                kDeriv = gains[1];
                break;
            case PI:
                kProp = gains[0];
                kIntegr = gains[1];
                break;
            case PD:
                kProp = gains[0];
                kDeriv = gains[1];
                break;
            case P:
                kProp = gains[0];
                break;
            case I:
                kIntegr = gains[0];
                break;
            case D:
                kDeriv = gains[0];
                break;
        }//switch

    }//PID

    public enum Type {
        PID,
        PI,
        PD,
        ID,
        P,
        I,
        D;

        public boolean containsP() {
            return this.equals(PID) || this.equals(PI) || this.equals(PD) || this.equals(P);
        }//containsP

        public boolean containsI() {
            return this.equals(PID) || this.equals(PI) || this.equals(ID) || this.equals(I);
        }//containsI

        public boolean containsD() {
            return this.equals(PID) || this.equals(ID) || this.equals(PD) || this.equals(D);
        }//containsD
    }//Type

    //method to call every time a new error is calculated
    public double update(double error) {

        if (algorithmType.containsI()) {
            integrStore += error;
        }//if

        double val = kProp * error + kIntegr * integrStore + kDeriv * (error - derivStore);

        if (algorithmType.containsD()) {
            derivStore = error;
        }//if

        return val;
    }//update

}//PID
