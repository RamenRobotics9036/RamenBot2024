package frc.robot.Util;

public class Coords {
    private double x;
    private double y;

    public Coords(double xValue, double yValue) {
        x = xValue;
        y = yValue;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public void setY(double newY) {
        y=newY;
    }
     public void setX(double newX) {
        y=newX;
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
    
