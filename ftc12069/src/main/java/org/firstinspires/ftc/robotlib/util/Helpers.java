package org.firstinspires.ftc.robotlib.util;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.ArrayList;

/**
 * Various generic helper methods used throughout the library.
 */
public class Helpers {
    /**
     * Averages the doubles in an ArrayList.
     * @param arrayList list to average values
     * @return the average of the values
     */
    public static double averageArrayList(ArrayList<Double> arrayList) {
        double sum = 0;
        for (int i = 0; i < arrayList.size(); i++) sum += arrayList.get(i);
        return sum / arrayList.size();
    }

    /**
     * Retrieves a value from an OpenGLMatrix (OpenGLMatrix is stored in Column-Major).
     * @deprecated
     * @param openGLMatrix OpenGLMatrix
     * @param row desired row
     * @param column desired column
     * @return Index of value in matrix
     */
    public static int getValueFromMatrix(OpenGLMatrix openGLMatrix, int row, int column) {
        return column * openGLMatrix.numRows() + row;
    }

    /**
     * Checks if 3 side lengths make a triangle.
     * @param a
     * @param b
     * @param c
     * @return true if triangle
     */
    public static boolean isTriangle(double a, double b, double c) {
        if (a + b < c) return false;
        if (a + c < b) return false;
        if (b + c < a) return false;
        return true;
    }

    /**
     * Checks if an array of 3 side lengths make a triangle.
     * @param sides Array with size 3 of side lengths
     * @return true if triangle
     * @see #isTriangle(double, double, double)
     */
    public static boolean isTriangle(double[] sides) {
        return isTriangle(sides[0], sides[1], sides[2]);
    }
}