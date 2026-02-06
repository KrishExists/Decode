package org.firstinspires.ftc.teamcode.util;

public class Arc {
    public double cx, cy;   // center
    public double radius;
    public double startAngle, endAngle; // radians
    public boolean ccw;     // counter-clockwise?

    public Arc(double cx, double cy, double radius, double startAngle, double endAngle, boolean ccw) {
        this.cx = cx;
        this.cy = cy;
        this.radius = radius;
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        this.ccw = ccw;
    }

    // get a point on the arc [t=0=start, t=1=end]
    public double[] getPoint(double t) {
        double angle;
        if(ccw) {
            angle = startAngle + t * (endAngle - startAngle);
        } else {
            angle = startAngle - t * (startAngle - endAngle);
        }
        double x = cx + radius * Math.cos(angle);
        double y = cy + radius * Math.sin(angle);
        return new double[]{x, y};
    }
}
