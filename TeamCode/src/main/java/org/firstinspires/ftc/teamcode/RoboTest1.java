package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JonasMotor")


public class RoboTest1 extends OpMode {

    DcMotor first;
    DcMotor second;
    DcMotor third;
    DcMotor fourth;

    @Override
    public void init() {
        first = hardwareMap.dcMotor.get("first");
        second = hardwareMap.dcMotor.get("second");
        third = hardwareMap.dcMotor.get("third");
        fourth = hardwareMap.dcMotor.get("fourth");



    }

    @Override
    public void loop() {

        /*
        if(gamepad1.a) {
            first.setPower(0.5);
            second.setPower(-0.5);

        }else if(gamepad1.b){
            first.setPower(-0.5);
            second.setPower(0.5);
        }
        else {
            first.setPower(0);
            second.setPower(0);
        }
         */

        first.setPower(gamepad1.left_stick_x);
        second.setPower(-gamepad1.left_stick_x);
        third.setPower(gamepad1.left_stick_y);
        fourth.setPower(-gamepad1.left_stick_y);
        

    }

    /**
     * a tangens hyperbolic function for a smooth gaming experience 
     * @param p
     * @return 0.390911 * tanh (6.66 * (p - 0.5))) + 0.5 )
     */
    private int smoothThePower(int p){
        return (0.390911*(java.lang.Math.tanh(6.66*(p-0.5)))+0.5)
    }

    /**
     * Matrix multiply double [ ].
     *
     * @param a the first matrix
     * @param x the second matrix
     * @return the reslt of the calculation
     */
// matrix-vector multiplication (y = A * x)
    public static double[] matrix_multiply(double[][] a, double[] x) {
        int m = a.length;
        int n = a[0].length;
        if (x.length != n) throw new RuntimeException("Illegal matrix dimensions.");
        double[] y = new double[m];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                y[i] += a[i][j] * x[j];
        return y;
    }

    /**
     * Calculate omnidrive double [ ], see http://zero.sci-hub.tw/2288/7b6d38e6cae805b5a249205a2a290b1b/10.1007@978-3-540-70534-5.pdf#page=159
     *
     * @param r  wheel radius
     * @param d  distance between left and right wheel pairs
     * @param e  distance between front and back wheel pairs
     * @param vx vehicle velocity in forward direction
     * @param vy the vy
     * @param w  vehicle rotational velocity
     * @return a array of wheel-speeds in rpm
     */
    public static double[] calculate_omnidrive(double r, double d, double e, double vx, double vy, double w) {
        double[] part1 = 1/(2*Math.PI*r);
        double[] wheel_matrix = [
                    [1, -1, -(d+e)/2],
                    [1,  1,  (d+e)/2],
                    [1,  1, -(d+e)/2],
                    [1, -1,  (d+e)/2]
                ];
        double[] velocities = [vx, vy, w];
        return matrix_multiply(matrix_multiply(part1, wheel_matrix), velocities);
    }
}
