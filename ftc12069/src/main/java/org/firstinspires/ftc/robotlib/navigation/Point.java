package org.firstinspires.ftc.robotlib.navigation;

public class Point {
    public float x;
    public float y;

    /**
     * Creates a point on the FTC field
     */
    public Point(float x, float y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a point on the FTC field (loss of data when casting to float)
     */
    public Point(double x, double y) {
        this.x = (float) x;
        this.y = (float) y;
    }

    /**
     * Multiplies two points together
     * @param point other point
     * @return Product
     */
    public double multiply(Point point) {
        return this.x * point.x + this.y * point.y;
    }

    /**
     * Calculates the distance between two 2-Dimensional points
     * @param point other point
     * @return Distance
     */
    public double distance(Point point) {
        return Math.sqrt(Math.pow(this.x - point.x, 2) + Math.pow(this.y - point.y, 2));
    }

    /**
     * Provides the equivalent point for the opposing alliance
     */
    public Point opponentPoint() {
        return new Point(this.x, -this.y);
    }
}
