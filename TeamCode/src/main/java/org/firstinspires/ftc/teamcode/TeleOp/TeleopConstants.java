package org.firstinspires.ftc.teamcode.TeleOp;

public class TeleopConstants {
    public static double drivePowerNormal = 0.7;
    public static double drivePowerTurbo = 1;
    public static double drivePowerSlow = 0.5;
    public static double turnPower = 0.5;
    public static double intakePower = 1;
    public static double liftPower = -1;

    public static double clawServo1Block = 0.80;
    public static double clawServo1PosClose = 0.85;    //@TODO Get clawServo1 & clawServo2 positions
    public static double clawServo1PosOpen = 0.4;

    public static double clawServo2PosClose = 0.3209;
    public static double clawServo2PosOpen = 0.687;

    public static double odometerLockPosUp = 0.452;
    public static double odometerLockPosDown = 0.17134;

    public static double transferLockPosPlatform = 0.732;
    public static double transferLockPosUp = 0.386;
    public static double transferLockPosOut = 0;

    public static double foundationLockUnlock = 0;
    public static double foundationLockLock = 0.339596;

    public static double plateLifterPosUp = 0.598;
    public static double plateLifterPosDown = 0.763;

    public static double transferHornPosReady = 0.5862;
    public static double transferHornPosPush = 0;
    public static double transferHornCapstone = 0.1203;

    public static double clawInitPosReset = 0.2;
    public static double clawInitPosCapstone = 0.822;
    public static double clawInitPosCapstoneForReal = 0.91;

    public static double innerTransferPosTucked = 0.4316;
    public static double innerTransferPosBlock = 0.01632;     //@TODO Get servo position innerTransfer "block" position

    public static double intakeInitPosLeft = 0.55953;     //@TODO Get intakeInit servo positions
    public static double intakeInitPosRight = 0.08368;
    public static double intakeInitPosReset = 0.30124;

    public static int[] stoneEncoderValues = new int[] {0, -681, -1120, -1428, -1806};
}
