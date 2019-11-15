package org.firstinspires.ftc.teamcode;

public class EncoderWheel {

    public double x, y, heading;
    public int row; // Row in matrix
    public int port; // Port we're plugged into

    EncoderWheel(double x, double y, double heading, int row, int port) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.row = row;
        this.port = port;
    }
}
