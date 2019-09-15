package org.firstinspires.ftc.teamcode.drive;

/**
 * Using enums, the motions in the picture will be much easier to use
 * https://seeeddoc.github.io/4WD_Mecanum_Wheel_Robot_Kit_Series/img/Working_Principle-Simplified-.PNG
 */
public enum Direction {
    //LeftTop, RightTop, Leftbottom, RightBottom
    //or
    //LeftTop(0), RightTop (1)
    //LeftBottom(2), RightBottom (3)
    FORWARD(1, 1, 1, 1),
    LEFT(-1, 1, 1, -1),
    RIGHT(1, -1, -1, 1),
    BACKWARD(-1, -1, -1, -1),

    UPPERLEFT(0, 1, 1, 0),
    UPPERRIGHT(1, 0, 0, 1),
    BOTTOMLEFT(-1, 0, 0, -1),
    BOTTOMRIGHT(0, -1, -1, 0),

    COUNTERCLOCKWISE(-1, 1, -1, 1),
    CLOCKWISE(1,-1, 1, -1);

    private int[] movement;

    Direction(int... movement) {
        this.movement = movement;
    }

    public int getLeftTop() {
        return movement[0];
    }
    public int getRightTop() {
        return movement[1];
    }
    public int getLeftBottom() {
        return movement[2];
    }
    public int getRightBottom() {
        return movement[3];
    }
}
