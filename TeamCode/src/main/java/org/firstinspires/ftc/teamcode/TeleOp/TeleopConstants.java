package org.firstinspires.ftc.teamcode.TeleOp;

public class TeleopConstants {

    public static double liftSpeedSlow = 0.3;

    public static double drivePowerNormal = 0.8;
    public static double drivePowerTurbo = 1;
    public static double drivePowerSlow = 0.5;
    public static double turnPower = 0.5;
    public static double intakePower = 0.8;
    public static double liftPower = 1;

    public static double clawServo1PosClose = 0.075;    //@TODO Get clawServo1 & clawServo2 positions
    public static double clawServo1PosOpen = 0.3003;
    public static double clawServo1PosReceive = 0.54;
    public static double clawServo1Prep = 0.2196;
    public static double clawServo1Capstone = 0.4069;

    public static double clawServo2Block = 0.74281;
    public static double clawServo2PosClose = 0.522;
    public static double clawServo2PosOpen = 0.95;

    public static double transferLockPosPlatform = 0.52;
    public static double transferLockPosUp = 0.3855;
    public static double transferLockPosOut = 0.2716;

    public static double foundationLockUnlock = 0.44;
    public static double foundationLockLock = 0.168;

    public static double transferHornPosReady = 0.62;
    public static double transferHornPosPush = 0;
    public static double transferHornCapstone = 0.191;

    public static double clawInitPosReset = 0.21;
    public static double clawInitPosCapstone = 0.6623;
    public static double clawInitPosCapstoneForReal = 0.547;

    public static double innerTransferPosTucked = 0.3416; //0.1201 closed,
    public static double innerTransferPosReleased = 0.055;
    public static double innerTransferPosClosed = 0;     //@TODO Get servo position innerTransfer "block" position

    public static double intakeInitPosLeft = 0.4787;     //@TODO Get intakeInit servo positions
    public static double intakeInitPosRight = 0.13306;
    public static double intakeInitPosReset = 0.3108;

    public static double autoClaw1Retracted = 0.1048;
    public static double autoClaw1Drop = 0.37829;
    public static double autoClaw1Stone = 0.7461;
    public static double autoClaw1Extended = 0.58858;

    public static double autoClaw2PickUp = 0.6445;
    public static double autoClaw2Init = 0.55122;
    public static double autoClaw2Prep = 0.40908;
    public static double autoClaw2Grabbing = 0.25561;

    public static double autoClaw3Init = 0.8554;
    public static double autoClaw3Closed = 0.5709;
    public static double autoClaw3Open = 0.3669;

    public static double parkingServoPosUnlock = 0.4959;
    public static double parkingServoPosLock = 0.3815;

    public static double liftOdometryDown = 0.142;
    public static double liftOdometryUp = 0.425;

    public static int[] stoneEncoderValues = new int[] {0, -681, -1120, -1428, -1806};
}
