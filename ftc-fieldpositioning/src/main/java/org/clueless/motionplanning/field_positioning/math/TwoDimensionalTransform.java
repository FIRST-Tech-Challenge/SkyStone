package org.clueless.motionplanning.field_positioning.math;

public class TwoDimensionalTransform {
    public Vector2 vector;
    public double angle;


    public TwoDimensionalTransform(Vector2 vector, double angle){
        this.vector = vector;
        this.angle = angle;
    }

    public void translate(Vector2 translation) {
        vector = Vector2.add(vector, translation);
    }

    public void rotate(double angle) {
        this.angle += angle;
    }
}
