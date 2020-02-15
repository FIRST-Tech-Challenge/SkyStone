package org.firstinspires.ftc.teamcode.DutchFTCCore;

import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.MovementSubSystem;

public class Drivetraintypes {

    public Robot bot;
    public MovementSubSystem movSys;

    public enum Drivetrains{
        TANKDRIVE,
        FOURWHEELTANKDRIVE,
        FIVEWHEELHDRIVE,
        THREEWHEELHDRIVE,
        MECHANUMDRIVE,
        KIWIDRIVE
    }

    public void initDrivetrain(Robot _bot){
        Drivetrains Train;
        bot = _bot;
        Train  = Robotconfig.DriveTrain;


        switch (Train){
            case MECHANUMDRIVE:
                bot.MotorBackLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackLeft);
                bot.MotorFrontLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontLeft);
                bot.MotorFrontRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontRight);
                bot.MotorBackRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackRight);
                break;

            case TANKDRIVE:
                bot.MotorBackLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackLeft);
                bot.MotorBackRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackRight);
                break;

            case FOURWHEELTANKDRIVE:
                bot.MotorBackLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackLeft);
                bot.MotorFrontLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontLeft);
                bot.MotorFrontRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontRight);
                bot.MotorBackRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackRight);
                break;

            case KIWIDRIVE:
                bot.MotorBackLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackLeft);
                bot.MotorFrontLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontLeft);
                bot.MotorFrontRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontRight);
                bot.MotorBackRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackRight);
                break;

            case FIVEWHEELHDRIVE:
                bot.MotorBackLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackLeft);
                bot.MotorFrontLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontLeft);
                bot.MotorFrontRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorFrontRight);
                bot.MotorBackRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackRight);
                bot.MotorMiddle = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorMiddle);
                break;

            case THREEWHEELHDRIVE:
                bot.MotorBackLeft = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackLeft);
                bot.MotorBackRight = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorBackRight);
                bot.MotorMiddle = bot.opMode.hardwareMap.dcMotor.get(Robotconfig.MotorMiddle);
                break;

                default:
                    System.out.println("PLEASE SELECT A DRIVETRAIN");
                    break;
        }

    }

    public void DriveChecks(){
        Drivetrains Train;

        Train  = Robotconfig.DriveTrain;

        switch (Train){
            case MECHANUMDRIVE:
                movSys.instance.DriveChecksMechanum();
                break;

            case TANKDRIVE:
                movSys.instance.DriveChecksTankDrive();
                break;

            case FOURWHEELTANKDRIVE:
                movSys.instance.DriveChecks4WheelTankDrive();
                break;

            case KIWIDRIVE:
                movSys.instance.DriveChecksKiwiDrive();
                break;

            case FIVEWHEELHDRIVE:
                movSys.instance.DriveChecksHDrive5Motors();
                break;

            case THREEWHEELHDRIVE:
                movSys.instance.DriveChecksHDrive3Motors();
                break;

            default:
                //System.out.println("PLEASE SELECT A DRIVETRAIN");
                break;
        }
    }
}
