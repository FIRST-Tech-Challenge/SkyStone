package org.firstinspires.ftc.teamcode.algorithms;

public class Stone {

    /**
     * kind of stone
     * 0 = normal
     * 1 = sky
     */
    int kind = 0;

    /**
     * top of stone
     */
    float top;

    /**
     * bottom of stone
     */
    float bottom;

    /**
     * left side of stone
     */
    float left;

    /**
     * right side of stone
     */
    float right;

    public int getKind() {
        return kind;
    }

    public void setKind(int kind) {
        this.kind = kind;
    }

    public float getTop() {
        return top;
    }

    public void setTop(float top) {
        this.top = top;
    }

    public float getBottom() {
        return bottom;
    }

    public void setBottom(float bottom) {
        this.bottom = bottom;
    }

    public float getLeft() {
        return left;
    }

    public void setLeft(float left) {
        this.left = left;
    }

    public float getRight() {
        return right;
    }

    public void setRight(float right) {
        this.right = right;
    }
    public float getCenterX(){
        return (this.getRight() + this.getLeft())/2;
    }

    public float getCenterY(){
        return (this.getTop() + this.getBottom())/2;
    }

    public Stone(int kind, float top, float bottom, float left, float right) {
        this.kind = kind;
        this.top = top;
        this.bottom = bottom;
        this.left = left;
        this.right = right;
    }

    public Stone(float top, float bottom, float left, float right) {
        this.kind = 0;
        this.top = top;
        this.bottom = bottom;
        this.left = left;
        this.right = right;
    }

    public Stone() {
        this.kind = 0;
        this.top = 0;
        this.bottom = 0;
        this.left = 0;
        this.right = 0;
    }
}

