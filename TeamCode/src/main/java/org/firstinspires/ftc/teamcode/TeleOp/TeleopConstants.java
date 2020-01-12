package org.firstinspires.ftc.teamcode.TeleOp;

public class TeleopConstants {
    public static double drivePowerNormal = 0.8;
    public static double drivePowerTurbo = 1;
    public static double drivePowerSlow = 0.5;
    public static double turnPower = 0.5;
    public static double intakePower = 0.8;
    public static double liftPower = 1;

    public static double clawServo1Block = 0.80;
    public static double clawServo1PosClose = 0.85;    //@TODO Get clawServo1 & clawServo2 positions
    public static double clawServo1PosOpen = 0.4;

    public static double clawServo2PosClose = 0.3209;
    public static double clawServo2PosOpen = 0.687;

    public static double transferLockPosPlatform = 0.46017;
    public static double transferLockPosUp = 0.4939;
    public static double transferLockPosOut = 0.2716;

    public static double foundationLockUnlock = 0.44;
    public static double foundationLockLock = 0.14;

    public static double transferHornPosReady = 0.5862;
    public static double transferHornPosPush = 0;
    public static double transferHornCapstone = 0.1203;

    public static double clawInitPosReset = 0;
    public static double clawInitPosCapstone = 0.479;
    public static double clawInitPosCapstoneForReal = 0.547;

    public static double innerTransferPosTucked = 0.4316;
    public static double innerTransferPosBlock = 0.01632;     //@TODO Get servo position innerTransfer "block" position

    public static double intakeInitPosLeft = 0.65953;     //@TODO Get intakeInit servo positions
    public static double intakeInitPosRight = 0;
    public static double intakeInitPosReset = 0.30124;

    public static double autoClaw1Init = 0.33622089656588355;
    public static double autoClaw1Strafe = 0.0815;
    public static double autoClaw1Down = 0;
    public static double autoClaw1Up = 0.23452223351931448;
    public static double autoClaw1TeleOp = 0.3032;

    public static double autoClaw2Init = 0;
    public static double autoClaw2Open = 0.556;
    public static double autoClaw2Close = 0.386;

    public static double parkingServoPosUnlock = 0.4959;
    public static double parkingServoPosLock = 0.3815;

    public static double liftOdometryDown = 0.142;
    public static double liftOdometryUp = 0.425;

    // Positions in teleop
    public static double autoClaw1Stowed = 0.335;
    public static double autoClaw2Stowed = 0.14;

    public static int[] stoneEncoderValues = new int[] {0, -681, -1120, -1428, -1806};
}
