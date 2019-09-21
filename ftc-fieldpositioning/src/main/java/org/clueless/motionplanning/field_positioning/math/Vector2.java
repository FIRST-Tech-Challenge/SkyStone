package org.clueless.motionplanning.field_positioning.math;

public class Vector2 {
    public double x, y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public static Vector2 add(Vector2 first, Vector2 second) {
        return new Vector2(first.x + second.x, first.y + second.y);
    }

    public Vector2 multiply(double scalar) {
        x *= scalar;
        y *= scalar;
        return this;
    }
}
