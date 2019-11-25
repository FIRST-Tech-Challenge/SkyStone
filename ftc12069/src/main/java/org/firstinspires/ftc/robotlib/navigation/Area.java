package org.firstinspires.ftc.robotlib.navigation;

import org.firstinspires.ftc.robotlib.navigation.Point;

public class Area {
    private Point cornerPoint;
    private Point cornerPoint2;

    /**
     * Creates a rectangular area on the FTC field
     * @param cornerPoint one corner point of the area
     * @param cornerPoint2 other corner point of the area
     */
    public Area(Point cornerPoint, Point cornerPoint2) {
        this.cornerPoint = cornerPoint;
        this.cornerPoint2 = cornerPoint2;
    }

    /**
     * Retrieves the first corner point
     */
    public Point getCornerPoint() {
        return cornerPoint;
    }

    /**
     * Retrieves the second corner point
     */
    public Point getCornerPoint2() {
        return cornerPoint2;
    }

    /**
     * Calculates the middle X of the area
     */
    public double getMiddleX() {
        return (this.cornerPoint.x + this.cornerPoint2.x) / 2;
    }

    /**
     * Calculates the middle Y of the area
     */
    public double getMiddleY() {
        return (this.cornerPoint.y + this.cornerPoint2.y) / 2;
    }

    /**
     * Checks if a point is in the area
     * @param point point to check
     * @return boolean (true if point is in area)
     */
    public boolean isPointInArea(Point point) {
        return isValueInArea(point.x) && isValueInArea(point.y);
    }

    /**
     * Checks if a value (x or y) is in the area
     * @param value value to check
     * @return boolean (true if value is in area)
     */
    private boolean isValueInArea(double value) {
        double absCornerPoint = Math.abs(cornerPoint.x);
        double absCornerPoint2 = Math.abs(cornerPoint2.x);

        return Math.min(absCornerPoint, absCornerPoint2) <= value && value <= Math.max(absCornerPoint, absCornerPoint2);
    }
}
