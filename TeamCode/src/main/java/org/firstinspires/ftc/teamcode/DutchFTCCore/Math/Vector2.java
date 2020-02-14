package org.firstinspires.ftc.teamcode.DutchFTCCore.Math;

public class Vector2 {
    public double X;
    public double Y;

    public double DistanceToVector2(Vector2 b) {
        return Math.sqrt((this.X - b.X)*(this.X - b.X) + (this.Y - b.Y)*(this.Y - b.Y));
    }

    public Vector2 subtract (Vector2 b) {
        Vector2 a = new Vector2();
        a.X = this.X - b.X;
        a.Y = this.Y - b.Y;
        return a;
    }

    public Vector2 add (Vector2 b) {
        Vector2 a = new Vector2();
        a.X = this.X + b.X;
        a.Y = this.Y +b.Y;
        return a;
    }
    public  static Vector2 add (Vector2 a, Vector2 b){
        Vector2 c = new Vector2(
                a.X + b.X,
                a.Y + b.Y
        );
        return c;
    }

    public Vector2 (double x, double y) {
        X = x;
        Y = y;
    }
    public Vector2() {
        X = 0;
        Y = 0;
    }



    public double Slope (){
        return (Y/X);
    }

    public double Angle(){
        return Math.toDegrees(Math.atan(Slope()));
    }

}
