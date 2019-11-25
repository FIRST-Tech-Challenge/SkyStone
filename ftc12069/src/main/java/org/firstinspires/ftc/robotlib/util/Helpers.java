package org.firstinspires.ftc.robotlib.util;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Various generic methods used throughout the library
 * @deprecated
 */
public class Helpers {
    /**
     * Retrieves a value from an OpenGLMatrix (OpenGLMatrix is stored in Column-Major)
     * @param openGLMatrix OpenGLMatrix
     * @param row desired row
     * @param column desired column
     * @return Index of value in matrix
     */
    public static int getValueFromMatrix(OpenGLMatrix openGLMatrix, int row, int column) {
        return column * openGLMatrix.numRows() + row;
    }
}