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
        double xdistance;
        double ydistance;
        if (x > coords.getX()) {
            xdistance = x - coords.getX();
        }
        else {
            xdistance = coords.getX() - x;
        }
        if (y > coords.getY()) {
            ydistance = y - coords.getY();
        }
        else {
            ydistance = coords.getY() - y;
        }
        double a = Math.pow(xdistance, 2);
        double b = Math.pow(ydistance, 2);
        double c = a + b;
        return Math.sqrt(c);
    }

}
