package org.firstinspires.ftc.teamcode.util;

/**
 * Created by FIXIT on 16-07-08.
 */
public class BitUtils {

    public static byte setBit(int bitNum, byte toSet) {
        return (byte) (toSet | 1 << bitNum);
    }

    public static byte clearBit(int bitNum, byte toSet) {
        return (byte) (toSet & ~(1 << bitNum));
    }

    public static int combineBytes (byte leastSig, byte mostSig) {
        return ((mostSig << 8) | (leastSig & 0xFF));
    }

}
