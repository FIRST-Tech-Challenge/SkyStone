package org.firstinspires.ftc.robotlib.util;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Point {
    public float x;
    public float y;
    public float z;

    /**
     * Creates a new point using x, y, and z
     */
    public Point(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Creates a point from an FTC VectorF
     * @param translation
     */
    public Point(VectorF translation) {
        this.x = translation.get(0);
        this.y = translation.get(1);
        this.z = translation.get(2);
    }

    /**
     * Creates a point from an OpenGLMatrix
     * @param openGLMatrix OpenGLMatrix
     */
    public Point(OpenGLMatrix openGLMatrix) {
        this.x = Helpers.getValueFromMatrix(openGLMatrix, 0, 0);
        this.y = Helpers.getValueFromMatrix(openGLMatrix, 1, 1);
        this.z = Helpers.getValueFromMatrix(openGLMatrix, 2, 2);
    }

    /**
     * Multiplies two points together
     * @param point other point
     * @return Product
     */
    public double multiply(Point point) {
        return this.x * point.x + this.y * point.y + this.z * point.z;
    }

    /**
     * Calculates the distance between to 3-Dimensional points
     * @param point other point
     * @return Distance
     */
    public double distance(Point point) {
        return Math.sqrt(Math.pow(this.x - point.x, 2) + Math.pow(this.y - point.y, 2) + Math.pow(this.z - point.y, 2));
    }
}
