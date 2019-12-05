package org.firstinspires.ftc.robotlib.util;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.ArrayList;

/**
 * Various generic helper methods used throughout the library
 */
public class Helpers {
    /**
     * Averages the doubles in an ArrayList
     * @param arrayList list to average values
     * @return the average of the values
     */
    public static double averageArrayList(ArrayList<Double> arrayList) {
        double sum = 0;
        for (int i = 0; i < arrayList.size(); i++) sum += arrayList.get(i);
        return sum / arrayList.size();
    }

    /**
     * Retrieves a value from an OpenGLMatrix (OpenGLMatrix is stored in Column-Major)
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
     * Checks if an array of sides create a valid triangle
     * @param a Side Length
     * @param b Side Length
     * @param c Side Length
     * @return True if valid triangle
     */
    public static boolean isTriangle(double a, double b, double c) {
        if (a + b < c) return false;
        if (a + c < b) return false;
        if (b + c < a) return false;
        return true;
    }

    /**
     * Checks if an array of sides create a valid triangle
     * @see #isTriangle(double, double, double)
     * @param sides array of doubles with side lengths
     * @return True if valid triangle
     */
    public static boolean isTriangle(double[] sides) {
        return isTriangle(sides[0], sides[1], sides[2]);
    }

    /**
     * Attempts to increment or decrement in an array
     * @param array Array to look through
     * @param current Value to operate off of
     * @param operator Value to change index by
     * @return New value
     */
    public static <T> T arrayOperate(T[] array, T current, int operator) {
        for (int i = 0; i < array.length; i++) {
            if (array[i].equals(current)) {
                if (i + operator >= 0 && i + operator < array.length) return array[i + operator];
            }
        }
        return current;
    }
}