package org.firstinspires.ftc.teamcode.DutchFTCCore;

public class Drivetraintypes {
    public enum Drivetrains{
        TANKDRIVE,
        FOURWHEELTANKDRIVE,
        FIVEWHEELHDRIVE,
        THREEWHEELHDRIVE,
        MECHANUMDRIVE,
        KIWIDRIVE
    }

    public void name(){
        Drivetrains Train;

        Train  = Robotconfig.DriveTrain;

        switch (Train){
            case MECHANUMDRIVE:
                break;

            case TANKDRIVE:
                break;

            case KIWIDRIVE:
                break;

            case FIVEWHEELHDRIVE:
                break;

            case THREEWHEELHDRIVE:
                break;

            case FOURWHEELTANKDRIVE:
                break;

                default:
                    System.out.println("PLEASE SELECT A DRIVETRAIN");
                    break;
        }





    }
}
