package org.firstinspires.ftc.teamcode.localization;

/*
    Represents and x and y component of a two dimensional vector
 */

public class Vector2d {
    public double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d() {
        this.x = 0;
        this.y = 0;
    }

    double norm() { return Math.sqrt(x * x + y * y); }
    double angle() { return Math.atan2(y, x); }
    Vector2d add(Vector2d other) { return new Vector2d(x + other.x, y + other.y); }
    Vector2d minus(Vector2d other) { return new Vector2d(x - other.x, y - other.y); }
    Vector2d times(double scalar) { return new Vector2d(x * scalar, y * scalar); }
    Vector2d divide(double scalar) { return new Vector2d(x / scalar, y  / scalar); }

    public Vector2d rotated(double angle) {
        // vnew = R matrix * vold
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2d(newX, newY);
    }


}
