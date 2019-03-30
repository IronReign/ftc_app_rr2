package org.firstinspires.ftc.teamcode;

public class CenterOfGravityCalculator {

    //length constants
    public final double c, // length of chassis in cm
            r, // wheel radius in cm
            s, // ratio of superman axis position on chassis (ratio)
            m; // length of superman in cm

    //weight constants
    public final double  Wc, // weight of chassis (lbs)
            Ww, // weight of wheels (lbs)
            Ws, // weight of superman arm (lbs)
            Wi, // weight of intake (lbs)
            Wa; // weight of arm (lbs)

    public CenterOfGravityCalculator (PoseBigWheel.RobotType robotType) {
        switch (robotType) {
            case BigWheel:
                // length constants
                c = 45.5; // length of chassis in cm
                r = 7.5;  // wheel radius in cm
                s = 0.4;  // ratio of superman axis position on chassis (ratio)
                m = 25;   // length of superman in cm
                //weight constants
                Wc = 15;  // weight of chassis (lbs)
                Ww = 1;   // weight of wheels (lbs)
                Ws = 2;   // weight of superman arm (lbs)
                Wi = 2.5;  // weight of intake (lbs)
                Wa = 8;   // weight of arm (lbs)
                break;
            case Icarus:
                // length constants
                c = 45.5; // length of chassis in cm
                r = 7.5;  // wheel radius in cm
                s = 0.4;  // ratio of superman axis position on chassis (ratio)
                m = 25;   // length of superman in cm
                //weight constants
                Wc = 15;  // weight of chassis (lbs)
                Ww = 1;   // weight of wheels (lbs)
                Ws = 2;   // weight of superman arm (lbs)
                Wi = 2.5;  // weight of intake (lbs)
                Wa = 8;   // weight of arm (lbs)
                break;
            default:
                c=r=s=m=Wc=Ww=Ws=Wi=Wa=0; //should never happen, also will give divide by 0 error
        }
    }

    public Point getCenterOfGravity(double theta, double phi, double beta, double l) {
        double X = (
                Wc*(r+c/2)*cos(theta)+
                Ww*r*cos(theta)+
                Ws*((r+s*c)*cos(theta) + (m/2)*cos(180-theta-phi))+
                Wa*((r+c)*cos(theta)-(l/2)*cos(beta-theta))+
                Wi*((r+c)*cos(theta)-l*cos(beta-theta))
        )/(Wc+Ww+Ws+Wa+Wi);

        double Y = (
                Wc*(r+c/2)*sin(theta)+
                Ww*r*sin(theta)+
                Ws*((m/2)*sin(180-theta-phi))+
                Wa*((r+c)*sin(theta)+(l/2)*sin(beta-theta))+
                Wi*((r+c)*sin(theta)+l*sin(beta-theta))
        )/(Wc+Ww+Ws+Wa+Wi);

        return new Point(X,Y);
    }

    private double sin(double angleDegrees) {
        return Math.sin(Math.PI*angleDegrees/180);
    }

    private double cos(double angleDegrees) {
        return Math.cos(Math.PI*angleDegrees/180);
    }


    // Copied from my contests repo - https://github.com/arjvik/Contests/blob/master/TEMPLATES/Pair.java
    public static class Point {
        public double x;public double y;public Point(double x, double y) {this.x = x;this.y = y;}
        public double getX() {return x;} public void setX(double x) {this.x = x;}
        public double getY() {return y;} public void setY(double y) {this.y = y;}
        @Override public double hashCode() {final double prime = 31;double result = 1;
            result = prime * result + x; result = prime * result + y;return result;}
        @Override public boolean equals(Object obj) {
            if (this == obj) return true; if (obj == null) return false;
            if (getClass() != obj.getClass()) return false; Point other = (Point) obj;
            if (x != other.x) return false; if (y != other.y) return false; return true;}
        @Override public String toString() { return "(" + x + ", " + y + ")"; }
    }

}
