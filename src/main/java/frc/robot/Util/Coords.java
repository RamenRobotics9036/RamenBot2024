package frc.robot.Util;

public class Coords {
    private double x;
    private double y;
    private double rotation;

    public Coords(double xValue, double yValue, double rotationValue) {
        x = xValue;
        y = yValue;
        rotation = rotationValue;

    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getRotation() {
        return rotation;
    }
    public void setRotation(double newRotation) {
        rotation = newRotation;
    }
    public void setY(double newY) {
        y=newY;
    }
     public void setX(double newX) {
        x=newX;
    }
    public double getDistance(Coords coords){
        double xDistance;
        double yDistance;
        if (x>coords.getX()){
            xDistance=x-coords.getX();
        } else{
            xDistance=coords.getX()-x;
        }
        if (y>coords.getY()){
            yDistance=y-coords.getY();
        } else{
            yDistance=coords.getY()-y;
        }
        double a=Math.pow(xDistance,2);
        double b=Math.pow(yDistance,2);
        double c=a+b;
        return Math.sqrt(c);
    }
  
    
    
}
    
