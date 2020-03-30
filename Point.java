

/**
 * This Class resambles a position in the world with x and y coordinate.
 */
public class Point
{

    public double x, y;

    public Point(double pos[])
    {
        this.x = pos[0];
        this.y = pos[1];
    }

    public Point(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public Point(int[] pos){
        this.x = pos[0];
        this.y = pos[1];
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public double getDistance(Point p) {
        return Math.sqrt(getSquaredDistance(p));
    }

    public double getSquaredDistance(Point p) {
        return Math.pow(p.x-x, 2) + Math.pow(p.y-y, 2);
    }

    // Bearing to another position, relative to 'North'
    // Bearing have several meanings, in this case the angle between
    // north and the position p.
    public double getBearingTo(Point p) {
        return Math.atan2(p.y - y, p.x - x);
    }

    public String toString() {
        return "x="+x+", y="+y;
    }
}
