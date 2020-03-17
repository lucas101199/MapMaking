/**
 * Simple class that stores a sequence of points and is able to iterate through it
 */

public class Path
{
   public Point[] path;
   public int nextPoint = 0;

   public Path(Point[] wayPoints) {
       path = wayPoints;
   }

    public Point getGoalPoint(Point pos, double targetDistance) {
       Point goal = pos;
        while(!pathFinished() && path[nextPoint].getDistance(pos) < targetDistance) {
            System.out.println();
            nextPoint++;
        }
        return getNextPoint();
    }

    public boolean pathFinished() {
        return nextPoint + 1 >= path.length;
    }

    public Point getNextPoint() {
        return path[nextPoint];
    }

    /**
     * Return the path as an array of Point objects
     * @return Array of Points
     */
   public Point[] getPath()
   {
      return path;
   }
}
