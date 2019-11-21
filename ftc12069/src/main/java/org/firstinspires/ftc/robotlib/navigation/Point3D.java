package org.firstinspires.ftc.robotlib.navigation;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotlib.util.Helpers;

public class Point3D extends Point {
    public float z;

    /**
     * Creates a new point using x, y, and z
     */
    public Point3D(float x, float y, float z) {
        super(x, y);
        this.z = z;
    }

    /**
     * Creates a point from an FTC VectorF
     * @param translation
     */
    public Point3D(VectorF translation) {
        super(translation.get(0), translation.get(1));
        this.z = translation.get(2);
    }

    /**
     * Creates a point from an OpenGLMatrix
     * @param openGLMatrix OpenGLMatrix
     */
    public Point3D(OpenGLMatrix openGLMatrix) {
        this(openGLMatrix.getTranslation());
        /*super(Helpers.getValueFromMatrix(openGLMatrix, 0, 0), Helpers.getValueFromMatrix(openGLMatrix, 1, 1));
        this.z = Helpers.getValueFromMatrix(openGLMatrix, 2, 2);*/
    }

    /**
     * Multiplies two points together
     * @param point3D other point3D
     * @return Product
     */
    public double multiply3D(Point3D point3D) {
        return this.x * point3D.x + this.y * point3D.y + this.z * point3D.z;
    }

    /**
     * Calculates the distance between two 3-Dimensional points
     * @param point3D other point3D
     * @return Distance
     */
    public double distance3D(Point3D point3D) {
        return Math.sqrt(Math.pow(this.x - point3D.x, 2) + Math.pow(this.y - point3D.y, 2) + Math.pow(this.z - point3D.y, 2));
    }
}
