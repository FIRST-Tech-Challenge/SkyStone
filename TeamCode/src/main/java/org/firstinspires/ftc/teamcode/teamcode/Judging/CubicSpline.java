package org.firstinspires.ftc.teamcode.teamcode.Judging;

import java.util.ArrayList;

public class CubicSpline {

    public static void main(String[] argument)
    {
        CubicSpline q = new CubicSpline();
        ArrayList<Point> p = new ArrayList<>();
        ArrayList<Function> functions = new ArrayList<>();
        ArrayList<Motor_Power_Spline> motor_power_splines = new ArrayList<>();
        Function[] temp;


        p.add(new Point(0, 0, 0));
        p.add(new Point(5, 10, 10));
        p.add(new Point(10, 20, 0));

        int number_of_equations = p.size() - 2;

        ArrayList<Point> inpoints;



        for(int i = 0; i < number_of_equations; i++)
        {
            inpoints = new ArrayList<>();
            inpoints.add(p.get(i));
            inpoints.add(p.get(i + 1));
            inpoints.add(p.get(i + 2));


            if(i < 1)
            {
                temp = q.makeSpline(inpoints);
                functions.add(temp[0]);
                functions.add(temp[1]);

            }
            else if(i >= 1)
            {
                functions.add(q.makeSplineSecond(functions.get(i), inpoints));
            }

        }

        for(Function f : functions)
        {
            System.out.println(f);
        }

        double leftpower;
        double rightpower;
        double secondDer;
        double der;


        ArrayList<Point> splinePoints = new ArrayList<>();

        for(Function f : functions)
        {
            for(double t = f.startT + .001 ; t  < f.endT; t += .001)
            {
                der = f.getDerY(t) / f.getDerX(t);
                secondDer = f.getSecondDerY(t) / f.getSecondDerX(t);

                splinePoints.add(new Point(t, f.getFuncX(t), f.getFuncY(t), der, secondDer));


            }
        }

        for(Point points : splinePoints)
        {
            leftpower = Motor_Power_Spline.setLeftPower(points.secondDerivative , points.derivative, 1, .01);
            rightpower = Motor_Power_Spline.setRightPower(points.secondDerivative, points.derivative, 1, .01);

            motor_power_splines.add(new Motor_Power_Spline(leftpower, rightpower));
        }

        for(int i = 0; i < motor_power_splines.size(); i++)
        {
            System.out.println(motor_power_splines.get(i));
        }

    }


    public CubicSpline() {


    }






    public Function[] makeSpline(ArrayList<Point> points)
    {

        double[][] constraints = new double[16][16];
        double[] solutions = new double[16];

        double t1 = points.get(0).getT();
        double t2 = points.get(1).getT();
        double t3 = points.get(2).getT();


        int Xc1 = 1;
        int Xc2 = 1;
        int Yc1 = 1;
        int Yc2 = 1;

        if(points.get(0).getX() < points.get(1).getX()) Xc1 = 1;
        else if (points.get(0).getX() > points.get(1).getX()) Xc1 = -1;

        if(points.get(1).getX() < points.get(2).getX()) Xc2 = 1;
        else if (points.get(1).getX() > points.get(2).getX()) Xc2 = -1;

        if(points.get(0).getY() < points.get(1).getY()) Yc1 = 1;
        else if (points.get(0).getY() > points.get(1).getY()) Yc1 = -1;

        if(points.get(1).getY() < points.get(2).getY()) Yc2 = 1;
        else if(points.get(1).getY() > points.get(2).getY()) Yc2 = -1;
        /*
            1.) Set solutions to constraints
            2.) Fix constraints matrix
            3.) Use gaussian elimination to find solutions
            4.) First 10 rows are X functions, Second set of 10 rows is Y functions
            5.) Create functions and put them into array and return them
         */


        //1.)   Solutions for Constraints

        solutions[0] = points.get(0).getX();
        solutions[1] = points.get(1).getX();
        solutions[2] = points.get(1).getX();
        solutions[3] = points.get(2).getX();
        solutions[4] = 0;
        solutions[5] = 0;
        solutions[6] = points.get(0).getY();
        solutions[7] = points.get(1).getY();
        solutions[8] = points.get(1).getY();
        solutions[9] = points.get(2).getY();
        solutions[10] = 0;
        solutions[11] = 0;

        solutions[12] = 0;
        solutions[13] = 0;
        solutions[14] = 0;
        solutions[15] = 0;


        //2.)   Set Constraints


        //X constraints

        constraints[0][0] = 1;

        constraints[1][0] =  1;
        constraints[1][1] = t2 - t1;
        constraints[1][2] = Math.pow(t2 - t1, 2);
        constraints[1][3] = Math.pow(t2 - t1, 3);

        constraints[2][4] =  1;

        constraints[3][4] =  1;
        constraints[3][5] = t3 - t2;
        constraints[3][6] = Math.pow(t3 - t2, 2);
        constraints[3][7] = Math.pow(t3 - t2, 3);

        constraints[4][1] = 1;
        constraints[4][5] = -1;
        constraints[4][2] = 2 * (t2 - t1);
        constraints[4][3] = 3 * Math.pow(t2 - t1, 2);

        constraints[5][2] = 2;
        constraints[5][6] = -2;
        constraints[5][3] = 6 * (t2 - t1);


        // Y Constraints

        constraints[6][8] =  1;

        constraints[7][8] =  1;
        constraints[7][9] = t2 - t1;
        constraints[7][10] = Math.pow(t2 - t1, 2);
        constraints[7][11] = Math.pow(t2 - t1, 3);

        constraints[8][12] =  1;

        constraints[9][12] =  1;
        constraints[9][13] = t3 - t2;
        constraints[9][14] = Math.pow(t3 - t2, 2);
        constraints[9][15] = Math.pow(t3 - t2, 3);

        constraints[10][9] = 1;
        constraints[10][13] = -1;
        constraints[10][10] = 2 * (t2 - t1);
        constraints[10][11] = 3 * Math.pow(t2 - t1, 2);

        constraints[11][10] = 2;
        constraints[11][14] = -2;
        constraints[11][11] = 6 * (t2 - t1);


        //X and Y Optimization Constraints - Ouroboros Method

        double c1 = Math.round(Math.sqrt(1 + t1 * t1) / Math.sqrt(1 + t2 * t2) * 10000000000L) / 10000000000L;
        double c2 = Math.sqrt(1 + t2 * t2) / Math.sqrt(1 + t3 * t3);

        constraints[12][1] = 1 -  c1 * 1;
        constraints[12][2] = 2 * (t1 - t1) - c1 * 2 * (t2 - t1);
        constraints[12][3] = 3 * Math.pow((t1 - t1), 2) - c1 * 3 * Math.pow((t2 - t1), 2);

        constraints[13][5] = 1 - c2 * 1;
        constraints[13][6] = 2 * (t2 - t2) - c2 * 2 * (t3 - t2);
        constraints[13][7] = 3 * Math.pow((t2 - t2), 2) - c2 * 3 * Math.pow((t3 - t2), 2);

        constraints[14][9] = 1 - c1 * 1;
        constraints[14][10] = 2 * (t1 - t1) - c1 * 2 * (t2 - t1);
        constraints[14][11] = 3 * Math.pow((t1 - t1), 2) - c1 * 3 * Math.pow((t2 - t1), 2);

        constraints[15][13] = 1 -c2 * 1;
        constraints[15][14] = 2 * (t2 - t2) - c2 * 2 * (t3 - t2);
        constraints[15][15] = 3 * Math.pow((t2 - t2), 2) -  c2 * 3 * Math.pow((t3 - t2), 2);

        /*int i = 0;
        for(double[] row : constraints)
        {
            for(double val : row)
            {
                System.out.print(val + " ");
            }
            System.out.print(" " + solutions[i]);
            System.out.println("");
            i++;
        }*/


        double[] solvedConstraints;

        GaussianElimination eq = new GaussianElimination();

        solvedConstraints = eq.solve(constraints, solutions);


        Function[] funcs = new Function[2];

        funcs[0] = new Function(solvedConstraints[0], solvedConstraints[1], solvedConstraints[2], solvedConstraints[3],
                solvedConstraints[8], solvedConstraints[9], solvedConstraints[10], solvedConstraints[11], t1, t2);
        funcs[1] = new Function(solvedConstraints[4], solvedConstraints[5], solvedConstraints[6], solvedConstraints[7],
                solvedConstraints[12], solvedConstraints[13], solvedConstraints[14], solvedConstraints[15], t2, t3);

        return funcs;
    }


    public Function makeSplineSecond(Function f, ArrayList<Point> points)
    {

        double ax = f.aX;
        double bx = f.bX;
        double cx = f.cX;
        double dx = f.dX;
        double ay = f.aY;
        double by = f.bY;
        double cy = f.cY;
        double dy = f.dY;





        double[][] constraints = new double[8][8];
        double[] solutions = new double[8];

        double t2 = points.get(1).getT();
        double t3 = points.get(2).getT();

        int Xc2 = 1;
        int Yc2 = 1;

        if(points.get(1).getX() < points.get(2).getX()) Xc2 = 1;
        else if (points.get(1).getX() > points.get(2).getX()) Xc2 = -1;


        if(points.get(1).getY() < points.get(2).getY()) Yc2 = 1;
        else if(points.get(1).getY() > points.get(2).getY()) Yc2 = -1;

        /*
            1.) Set solutions to constraints
            2.) Fix constraints matrix
            3.) Use gaussian elimination to find solutions
            4.) First 10 rows are X functions, Second set of 10 rows is Y functions
            5.) Create functions and put them into array and return them
         */


        //1.)   Solutions for Constraints

        solutions[0] = points.get(1).getX();
        solutions[1] = points.get(2).getX();
        solutions[2] = f.getDerX(t2);
        solutions[3] = 0;
        solutions[4] = points.get(1).getY();
        solutions[5] = points.get(2).getY();
        solutions[6] = f.getDerY(t2);
        solutions[7] = 0;


        //2.)   Set Constraints


        //X constraints


        constraints[0][0] =  1;

        constraints[1][0] =  1;
        constraints[1][1] = t3 - t2;
        constraints[1][2] = Math.pow(t3 - t2, 2);
        constraints[1][3] = Math.pow(t3 - t2, 3);

        constraints[2][1] = 1;
        constraints[2][2] = 2 * (t2 - t2);
        constraints[2][3] = 3 * Math.pow((t2 - t2), 2);

        constraints[3][1] = 1 - 1 * Xc2;
        constraints[3][2] = 2*(t2 - t2) - Xc2 * 2 * (t3 - t2);
        constraints[3][4] = 3*(t2 - t2) - Xc2 * 3 * Math.pow(t3 - t2, 2);

        constraints[4][4] = 1;

        constraints[5][4] = 1;
        constraints[5][5] = t3 - t2;
        constraints[5][6] = Math.pow(t3 - t2, 2);
        constraints[5][7] = Math.pow(t3 - t2, 3);

        constraints[6][5] = 1;
        constraints[6][6] = 2 * (t2 - t2);
        constraints[6][7] = 3 * Math.pow((t2 - t2), 2);

        constraints[7][5] = 1 - 1 * Yc2;
        constraints[7][6] = 2*(t2 - t2) - Yc2 * 2 * (t3 - t2);
        constraints[7][7] = 3*(t2 - t2) - Yc2 * 3 * Math.pow(t3 - t2, 2);



        double[] solvedConstraints = new double[8];

        GaussianElimination eq = new GaussianElimination();

        solvedConstraints = eq.solve(constraints, solutions);


        for(int i = 0; i < solvedConstraints.length; i++)
        {
            if(solvedConstraints[i] < 1E-10)
            {
                solvedConstraints[i] = 0;
            }
        }


        return new Function(solvedConstraints[0], solvedConstraints[1], solvedConstraints[2],
                solvedConstraints[3], solvedConstraints[4], solvedConstraints[5], solvedConstraints[6], solvedConstraints[7], t2, t3);
    }
}
