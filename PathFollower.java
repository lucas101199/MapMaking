public class PathFollower {
    public static double FINAL_DISTANCE = 0.2;
    public Point pos;
    public double speed = 0.5; //0.5 is safe, 1.0 is fine for not overly curvy paths
    public double lookDistance = 1; //1 is safe, 1.5 rather risky, below 0.5 does not work
    public Path path;
    private RobotCommunication comm = new RobotCommunication("http://127.0.0.1", 50000);
    private DifferentialDriveRequest driveRequest = new DifferentialDriveRequest();
    private LocalizationResponse locResponse = new LocalizationResponse();

    PathFollower(Path path) {
        this.path = path;
    }

    void run() throws Exception {
        comm.getResponse(locResponse);
        pos = getPosition();
        while ( (!path.pathFinished()) || (path.getNextPoint().getDistance(pos) < FINAL_DISTANCE) ) {
            step();
            sleep(30);
        }
        stopRobot();
        System.out.println("Path done!");
    }

    void step() throws Exception {
        comm.getResponse(locResponse);
        pos = getPosition();
        Point goalPoint = path.getGoalPoint(pos,lookDistance);
        System.out.println("Goalpoint X: " + goalPoint.getX() + "  Y: " + goalPoint.getY());
        double curvature = getCurvature(goalPoint, pos);
        driveRequest.setLinearSpeed(speed);
        driveRequest.setAngularSpeed(speed*curvature);
        comm.putRequest(driveRequest);
    }

    static void normalizeRadian(double r) {
        r += r>0 ? 0 : 2*Math.PI;
    }

    private Point getPosition() {
        double[] position = locResponse.getPosition();
        return new Point(position[0], position[1]);
    }

    Point worldToRobot(Point p, Point pos) {
        double angle = locResponse.getHeadingAngle();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = (p.x-pos.x) * cos + (p.y-pos.y) * sin;
        double y = - (p.x-pos.x) * sin + (p.y-pos.y) * cos;
        return new Point(x,y);
    }

    Point robotToWorld(Point p, Point pos) {
        double angle = locResponse.getHeadingAngle();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = pos.x + p.x*cos - p.y*sin;
        double y = pos.y + p.x*sin + p.y*cos;
        return new Point(x,y);
    }

    //takes world point
    double getCurvature(Point p, Point pos) {
        Point wtR = worldToRobot(p,pos);
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

    static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie);
        }
    }
}
