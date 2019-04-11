package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class CenterOfGravityCalculator {

    public static boolean drawRobotDiagram = true;

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

    public final double pitchOffset, //offset for theta, used externally only
            elbowoffset, //offset for beta, used externally only
            supermanoffset; //offset for phi, used externally only

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
                //offsets
                pitchOffset = -10;
                elbowoffset = 15;
                supermanoffset = 5;
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
                //offsets
                pitchOffset = -10;
                elbowoffset = 15;
                supermanoffset = 5;
                break;
            default:
                c=r=s=m=Wc=Ww=Ws=Wi=Wa=0; //should never happen, also will give divide by 0 error
                pitchOffset=elbowoffset=supermanoffset=0;
        }
    }

    public Point getCenterOfGravity(double theta, double phi, double beta, double l) {
        double X = (
                Wc*(c/2)*cos(theta)+
                Ws*((s*c)*cos(theta) + (m/2)*cos(phi-theta))+
                Wa*(c*cos(theta)-(l/2)*cos(beta-theta))+
                Wi*(c*cos(theta)-l*cos(beta-theta))
        )/(Wc+Ww+Ws+Wa+Wi);

        double Y = (
                Wc*(c/2)*sin(theta)+
                Ws*((m/2)*sin(phi-theta)-r)+
                Wa*(c*sin(theta)+(l/2)*sin(beta-theta))+
                Wi*(c*sin(theta)+l*sin(beta-theta))
        )/(Wc+Ww+Ws+Wa+Wi);

        if (drawRobotDiagram)
            drawRobotDiagram(theta, phi, beta, l, X, Y);

        return new Point(X,Y);
    }

    private double sin(double angle) {
        return Math.sin(Math.toRadians(angle));
    }

    private double cos(double angle) {
        return Math.cos(Math.toRadians(angle));
    }


    public void drawRobotDiagram(double theta, double phi, double beta, double l, double X, double Y) {
        TelemetryPacket p = new TelemetryPacket();
        p.fieldOverlay().setFill("white")
                        .setStroke("white")
                        .fillRect(-72,-72,144,144)
                        .setFill("black")
                        .setStroke("black")
                        .strokeCircle(0,0,r)
                        .strokeLine(0,0,c*sin(theta), -c*cos(theta))
                        .strokeLine(s*c*sin(theta), -s*c*cos(theta), -r, -s*c*cos(theta)-m*cos(phi-theta))
                        .strokeLine(c*sin(theta), -c*cos(theta), c*sin(theta)+l*sin(beta-theta), -c*cos(theta)+l*cos(beta-theta))
                        .strokeRect(c*sin(theta)+l*sin(beta-theta)-3, -c*cos(theta)+l*cos(beta-theta)-3,6,6)
                        .setFill("red")
                        .setStroke("red")
                        .fillCircle(Y, -X, 1);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    // Copied from my contests repo - https://github.com/arjvik/Contests/blob/master/TEMPLATES/Pair.java
    public static class Point {
        public double x;public double y;public Point(double x, double y) {this.x = x;this.y = y;}
        public double getX() {return x;} public void setX(double x) {this.x = x;}
        public double getY() {return y;} public void setY(double y) {this.y = y;}
        @Override public int hashCode() {final int prime = 31; int result = 1; long temp;
            temp = Double.doubleToLongBits(x); result = prime * result + (int) (temp ^ (temp >>> 32));
            temp = Double.doubleToLongBits(y); result = prime * result + (int) (temp ^ (temp >>> 32)); return result; }
        @Override public boolean equals(Object obj) { if (this == obj) return true; if (obj == null) return false;
            if (getClass() != obj.getClass()) return false; Point other = (Point) obj;
            if (Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x)) return false;
            if (Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y)) return false; return true; }
        @Override public String toString() { return "(" + x + ", " + y + ")"; }
    }

}
