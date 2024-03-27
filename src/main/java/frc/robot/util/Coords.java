package frc.robot.util;

/**
 * Defines the position of the robot.
 */
public class Coords {
    @SuppressWarnings("MemberNameCheck")
    private double x;

    @SuppressWarnings("MemberNameCheck")
    private double y;

    @SuppressWarnings("MemberNameCheck")
    private double rotation;

    /**
     * Constructor.
     */
    public Coords(double xvalue, double yvalue, double rotationValue) {
        x = xvalue;
        y = yvalue;
        rotation = rotationValue;

    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getRotation() {
        return rotation;
    }

    public void setRotation(double newRotation) {
        rotation = newRotation;
    }

    public void setY(double newY) {
        y = newY;
    }

    public void setX(double newX) {
        x = newX;
    }

    /**
     * Returns the distance between two points.
     */
    public double getDistance(Coords coords) {
        double xdistance = Math.abs(x - coords.getX());
        double ydistance = Math.abs(y - coords.getY());
        double a = Math.pow(xdistance, 2);
        double b = Math.pow(ydistance, 2);
        double c = a + b;
        return Math.sqrt(c);
    }
}
