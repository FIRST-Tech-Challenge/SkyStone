package org.firstinspires.ftc.robotcontroller.moeglobal.slam;

public class Constants {
    static int LIZARD_VID = 0x03e7;
    static int T265_VID = 0x8087;
    static int CHUNK_SIZE = 512 * 32;
    static int SMALL_TIMEOUT = 100;
    static byte[] SLAM_CONTROL = new byte[]{0x08, 0x00, 0x00, 0x00, 0x06, 0x10, 0x01, 0x06, 0};
    static byte[] POSE_CONTROL = new byte[]{0x09, 0, 0, 0, 0x02, 0x20, 0, 0, 0};
    static byte[] DEV_START = new byte[]{0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0};
    static int OUTPUT_X_OFFSET = 8;
    static int OUTPUT_Y_OFFSET = 12;
    static int OUTPUT_Z_OFFSET = 16;

}
