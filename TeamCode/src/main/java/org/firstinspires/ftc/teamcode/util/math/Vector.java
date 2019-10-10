package org.firstinspires.ftc.teamcode.util.math;

import java.lang.Math;

public class Vector {
	// components
	public double magnitude;
	public Angle angle;
	public double x;
	public double y;

	/**
	 * @param magnitude magnitude of vector
	 * @param angle     angle of vector in degrees
	 */
	public Vector(double magnitude, double angle) {
		this.angle = new Angle(angle);
		this.magnitude = magnitude;
		this.x = (magnitude * Math.cos(this.angle.radians()));
		this.y = (magnitude * Math.sin(this.angle.radians()));
	}

	/**
	 * @param x     x component of vector
	 * @param y     y component of vector
	 * @param angle angle of vector in degrees
	 */
	public Vector(double x, double y, double angle) {
		this.angle = new Angle(angle);
		this.magnitude = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
		this.x = x;
		this.y = y;
	}
}