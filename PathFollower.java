/**
 * This class contains the methods that are used for following the path created by the <code>Pathfinder</code>.
 */
public class PathFollower {
    public static double FINAL_DISTANCE = 0.2;
    public Point pos;
    public  double speed = 0.5; //0.5 is safe, 1.0 is fine for not overly curvy paths
    public  double lookDistance = 1.0; //1 is safe, 1.5 rather risky, below 0.5 does not work
    public  Path path;
    private  RobotCommunication comm = new RobotCommunication("http://127.0.0.1", 50000);
    private  DifferentialDriveRequest driveRequest = new DifferentialDriveRequest();
    private  LocalizationResponse locResponse = new LocalizationResponse();

    PathFollower(Path path) {
        this.path = path;
    }


    //In the end this code is not necessary hear anymore. Instead it should be in the main method of our robot

    //In the end this code is not necessary hear anymore. Instead it should be in the main method of our robot
    void step(Point pos, LocalizationResponse lr) throws Exception {
        Point goalPoint = path.getGoalPoint(pos, lookDistance);
        double curvature = getCurvature(goalPoint, pos, lr);
        driveRequest.setLinearSpeed(speed);
        driveRequest.setAngularSpeed(speed * curvature);
        comm.putRequest(driveRequest);
    }

    static void normalizeRadian(double r) {
        r += r>0 ? 0 : 2*Math.PI;
    }

    private Point getPosition() {
        double[] position = locResponse.getPosition();
        return new Point(position[0], position[1]);
    }

    /**
     * Takes a point in world coordinates and convertes it into the robots coordinate system.
     * @param p Point that shall be converted into robot coordinates.
     * @param pos The current position of the robot.
     * @return A <code>Point</code> with the x and y value beeing in robot coordinates.
     */
    Point worldToRobot(Point p, Point pos, LocalizationResponse lr) {
        double angle = lr.getHeadingAngle();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = (p.x-pos.x) * cos + (p.y-pos.y) * sin;
        double y = - (p.x-pos.x) * sin + (p.y-pos.y) * cos;
        return new Point(x,y);
    }

    /**
     * Takes a point in robot coordinates and convertes it into world coordinate system.
     * @param p Point that shall be converted.
     * @param pos The current position of the robot.
     * @return A <code>Point</code> with the x and y value beeing in world coordinates.
     */
    Point robotToWorld(Point p, Point pos) {
        double angle = locResponse.getHeadingAngle();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = pos.x + p.x*cos - p.y*sin;
        double y = pos.y + p.x*sin + p.y*cos;
        return new Point(x,y);
    }

    //takes world point

    /**
     * Calculates the curvature of the circle through the point and the robot.
     * @param p Point in world coordinates.
     * @param pos Robot position.
     * @return The curvature
     */
    double getCurvature(Point p, Point pos, LocalizationResponse lr) {
        Point wtR = worldToRobot(p,pos, lr);
        return 2 * wtR.y / p.getSquaredDistance(pos);
    }

    void printPosition() {
        double position[] = locResponse.getPosition();
        System.out.println("position = " + position[0] + ", " + position[1]);
    }

    void printHeading() {
        double angle = locResponse.getHeadingAngle();
        System.out.println("heading = " + angle);
    }

    void stopRobot() throws Exception {
        driveRequest.setLinearSpeed(0);
        driveRequest.setAngularSpeed(0);
        comm.putRequest(driveRequest);
    }

    void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie);
        }
    }
}
