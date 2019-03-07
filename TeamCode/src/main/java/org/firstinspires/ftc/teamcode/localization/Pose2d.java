package org.firstinspires.ftc.teamcode.localization;

/*

 */
public class Pose2d {
    private Vector2d pos;
    public double heading;

    public Pose2d(Vector2d pos, double heading) {
        this.pos = pos;
        this.heading = heading;
    }

    public Pose2d() {
        pos = new Vector2d();
        heading = 0;
    }

    public double x() { return pos.x; }
    public double y() { return pos.y; }

    public Vector2d pos() { return pos; }

    public Pose2d plus(Pose2d other) { return new Pose2d(new Vector2d(other.x() + pos.x, other.y() + pos.y), other.heading + heading);}
    public Pose2d minus(Pose2d other) { return new Pose2d(new Vector2d( pos.x - other.x(), pos.x - other.y()), heading - other.heading);}
    public Pose2d times(Pose2d other) { return new Pose2d(new Vector2d(other.x() * pos.x, other.y() * pos.y), other.heading * heading);}

}