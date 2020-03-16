package org.firstinspires.ftc.teamcode.Skystone;

public class Constants {
    // Outtake Slide Positions
    public static final double OUTTAKE_SLIDE_EXTENDED = 0.09;
    public static final double OUTTAKE_SLIDE_RETRACTED = 0.85;
//    public static final double OUTTAKE_SLIDE_PARTIAL_EXTEND = 0.27; // First peg .27, second peg .121

    // Clamp positions
    public static final double FRONTCLAMP_ACTIVATECAPSTONE = 0;
    public static final double FRONTCLAMP_CLAMPED = .52;
    public static final double FRONTCLAMP_RELEASED = .08;
    public static final double BACKCLAMP_CLAMPED = .22;
    public static final double BACKCLAMP_RELEASED = .635;

    // Outtake Pusher Positions
    public static final double PUSHER_PUSHED = 0.91;
    public static final double PUSHER_RETRACTED = .475;

    // Foundation Mover Positions
    public static final double LEFTFOUNDATION_EXTENDED = .65;
    public static final double LEFTFOUNDATION_RETRACTED = .86;

    public static final double RIGHTFOUNDATION_EXTENDED = .94;
    public static final double RIGHTFOUNDATION_RETRACTED = .72;

    // Timer delays for outtake actions. All in ms
    public static final long DELAY_SLIDE_ON_EXTEND = 0;

    public static final long DELAY_RELEASE_CLAMP_ON_RETRACT = 0;
    public static final long DELAY_PUSHER_ON_RETRACT = 0;
    public static final long DELAY_SLIDE_ON_RETRACT = 300;

    public static final long DELAY_PUSHER_ON_CLAMP = 0;
    public static final long DELAY_RETRACT_PUSHER_ON_CLAMP = 450;
    public static final long DELAY_CLAMP_ON_CLAMP = 700;

    // Constants for spool encoder positions
    public static final int[] spoolHeights = {150, 400, 763, 1075, 1420, 1764, 2097, 2446, 2778, 3135, 3500, 3825};
}
