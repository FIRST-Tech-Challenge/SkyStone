package org.firstinspires.ftc.teamcode.Library;

public class OmniWheel {
    public static double[] matrixMultiply(double[][] a, double[] x) {
        int m = a.length;
        int n = a[0].length;
        if (x.length != n) throw new RuntimeException("Illegal matrix dimensions.");
        double[] y = new double[m];
        for (int i = 0; i < m; i++){
            for (int j = 0; j < n; j++){
                y[i] += a[i][j] * x[j];
            }
        }
        return y;
    }

    public static double[] maxToOne(double[] originalValues) {
        double[] outputValues = {0,0,0,0};
        for (int i = 0; i < originalValues.length; i++) {
            if (originalValues[i] <= 1 && originalValues[i] >= -1) {
                outputValues[i] = originalValues[i];
            } else if (originalValues[i] >= -1) {
                outputValues[i] = -1;
            } else {
                outputValues[i] = 1;
            }
        }
        return outputValues;
    }

    /**
     * Calculate omniwheel speeds, see http://zero.sci-hub.tw/2288/7b6d38e6cae805b5a249205a2a290b1b/10.1007@978-3-540-70534-5.pdf#page=159
     *
     * @param wheelRadius  the radius of the wheel
     * @param sideDistance  distance between left and right wheel pairs
     * @param frontBackDistance  distance between front and back wheel pairs
     * @param forwardVelocity vehicle velocity in forward direction
     * @param sidewardsVelocity vehicle velocity in sideways direction
     * @param rotationVelocity  vehicle rotational velocity
     * @return a array of wheel-speeds
     */
    public static double[] calculate(double wheelRadius, double sideDistance, double frontBackDistance, double forwardVelocity, double sidewardsVelocity, double rotationVelocity) {
        double[][] wheel_matrix = {
                {1, -1, -(sideDistance+frontBackDistance)/2},
                {1,  1, (sideDistance+frontBackDistance)/2},
                {1,  1, -(sideDistance+frontBackDistance)/2},
                {1, -1, (sideDistance+frontBackDistance)/2}
        };
        double[] velocities = {forwardVelocity*20, sidewardsVelocity*20, rotationVelocity*0.5};
        double[] mat_multiplied = matrixMultiply(wheel_matrix, velocities);

        double[] wheel_speeds = new double[mat_multiplied.length];
        for (int i = 0; i < mat_multiplied.length; i++){
            wheel_speeds[i] = mat_multiplied[i]*1/(2*Math.PI*wheelRadius);
        }
        return maxToOne(wheel_speeds);
    }
}
