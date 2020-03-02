import java.util.LinkedList;

/**
 * TestRobot interfaces to the (real or virtual) robot over a network
 * connection. It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 ->
 * Lokarria(Robulab) -> Core -> MRDS4
 *
 * @author Thomas Johansson, dept. of Computing Science, Umeå University, Umeå,
 *         Sweden Mail: thomasj@cs.umu.se
 *
 *         Updated by Ola Ringdahl 2015-03-13, 2015-12-16, 2018-11-23
 */
public class TestRobot2 {

    private RobotCommunication robotcomm; // communication drivers
    public static int x_min;
    public static int y_min;
    public static int x_max;
    public static int y_max;
    public static int[] coord;
    public boolean first_time = true;
    public float[][] grid;
    public float[][] image_grid;
    /**
     * Create a robot connected to host "host" at port "port"
     *
     * @param host
     *            normally http://127.0.0.1
     * @param port
     *            normally 50000
     */
    public TestRobot2(String host, int port) {
        robotcomm = new RobotCommunication(host, port);
    }
    /**
     * This simple main program creates a robot, sets up some speed and turning
     * rate and then displays angle and position for 16 seconds.
     *
     * @param args
     *            not used
     * @throws Exception
     *             not caught
     */
    public static void main(String[] args) throws Exception {
        System.out.println("Creating Robot");
        TestRobot2 robot = new TestRobot2("http://127.0.0.1", 50000);
        //TestRobot2 robot = new TestRobot2("http://bratwurst.cs.umu.se", 50000);*
        x_min = -30;
        y_min = -40;
        x_max = 40;
        y_max = 20;
        coord = new int[]{x_min, y_min, x_max, y_max};

        robot.run();
        /*try {
            // Check for connection c
            robot.run();
        }
        catch (Exception e) {
            System.out.println(e);
            System.exit(-1);
        }*/
    }

    private void run() throws Exception {
        System.out.println("Creating response");
        LocalizationResponse lr = new LocalizationResponse();
        LaserEchoesResponse ler = new LaserEchoesResponse();
        LaserPropertiesResponse lpr = new LaserPropertiesResponse();

        System.out.println("Creating request");
        DifferentialDriveRequest dr = new DifferentialDriveRequest();

        System.out.println("Start to move robot");
        int rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);

        // Ask for the laser beam angles
        double angle = 0;
        double[] echoes = new double[0];
        robotcomm.getResponse(lpr);
        robotcomm.getResponse(lr);
        robotcomm.getResponse(ler);

        double[] angles = getLaserAngles(lpr);
        angle = lr.getHeadingAngle();
        echoes = ler.getEchoes();

        grid = createMap(lr, angle, echoes, angles); // create an example map
        dr.setAngularSpeed(Math.PI * 0.5);
        robotcomm.putRequest(dr);
        sleep(500);
        dr.setAngularSpeed(Math.PI * 0);
        robotcomm.putRequest(dr);

        ObjectAvoider objectAvoider = new ObjectAvoider();

        for (;;) {
            robotcomm.getResponse(lpr);
            robotcomm.getResponse(lr);
            robotcomm.getResponse(ler);
            angles = getLaserAngles(lpr);
            angle = lr.getHeadingAngle();
            echoes = ler.getEchoes();

            grid = createMap(lr, angle, echoes, angles); // create an example map

            //Compute A Path
            double[] rob_pos = lr.getPosition();
            double[] rob_pos_grid = new double[2];
            rob_pos_grid[0] = rob_pos[0] + Math.abs(x_min);
            rob_pos_grid[1] = rob_pos[1] + Math.abs(y_min);

            Point start = new Point(rob_pos_grid);

            Pathfinder scout = new Pathfinder(0.2, 0.2, (double) x_min, (double) y_min, grid);
            Path path = scout.findPath(start, grid);
            System.out.println("Pathlength: " + path.path.length);

            //Follow the Path
            PathFollower follower = new PathFollower(path);
            System.out.print("Pathfollowing begins");

            Point pos = new Point(rob_pos_grid);
            while ( (!path.pathFinished()) || (path.getNextPoint().getDistance(pos) < PathFollower.FINAL_DISTANCE) ) {
                robotcomm.getResponse(lpr);
                robotcomm.getResponse(lr);
                robotcomm.getResponse(ler);
                angles = getLaserAngles(lpr);
                angle = lr.getHeadingAngle();
                echoes = ler.getEchoes();

                if (objectAvoider.avoidObject(echoes, angle)) { //if obstacle encounter stop the robot
                    break;
                }

                grid = createMap(lr, angle, echoes, angles); // create an example map
                pos.setX(lr.getPosition()[0]);
                pos.setY(lr.getPosition()[1]);

                follower.step(pos);
                follower.sleep(30);
            }

        }

        /*System.out.println("Stop robot");
        rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);*/
    }

    /**
     * A simple example of how to use the ShowMap class that creates a map from
     * your grid, update it and save it to file
     */
    private float[][] createMap(LocalizationResponse localizationResponse, double angle,
                           double[] echoes, double[] angles) {
        /* use the same no. of rows and cols in map and grid */
        int nCols = (int) (Math.abs(x_max - x_min) / 0.2);
        System.out.println("nCols: " + nCols);
        int nRows = (int) (Math.abs(y_max - y_min) / 0.2);

        boolean showGUI = true; // set this to false if you run in putty
        ShowMap map = new ShowMap(nRows, nCols, showGUI, coord);

        /* Creating a grid with 0.5 */
        if (first_time) {
            grid = new float[nRows][nCols];
            image_grid = new float[nRows][nCols];
            for (int i = nRows - 1; i > 0; i--) {
                for (int j = 0; j < nCols; j++) {
                    grid[i][j] = (float) 0.5;
                    image_grid[i][j] = (float) 0.4;

                }
                first_time = false;
            }
        }

        // Position of the robot in the grid (red dot)
        double[] position_robot = localizationResponse.getPosition();
        int robotCol = (int) Math.round(position_robot[0]); //y
        int robotRow = (int) Math.round(position_robot[1]); //x

        double tt = localizationResponse.getHeadingAngle(); //angle in radians

        for (int i = 0; i < echoes.length; i++) {

            double y_end_line = robotRow + (echoes[i] * Math.sin(angles[i] + tt)); // y2 = y1 + (lenght * sin(angle)) grid without minus for path planning
            double x_end_line = robotCol + (echoes[i] *  Math.cos(angles[i] + tt)); // x2 = x1 + (lenght * cos(angle)) angle in radians
            double y_end_line_image = robotRow + (echoes[i] * -Math.sin(angles[i] + tt)); // y2 = y1 + (lenght * sin(angle)) grid with minus

            int[] obstacle = map.xy_to_rc(x_end_line, y_end_line);
            int[] robot = map.xy_to_rc(robotCol, robotRow);
            int[] obstacle_image = map.xy_to_rc(x_end_line, y_end_line_image);

            if (x_end_line > x_min && x_end_line < x_max && y_end_line > y_min && y_end_line < y_max) {
                colorGrid(grid, obstacle[0], obstacle[1], robot[0], robot[1], map);
            }
            if (x_end_line > x_min && x_end_line < x_max && y_end_line_image > y_min && y_end_line_image < y_max) {
                colorGrid(image_grid, obstacle_image[0], obstacle_image[1], robot[0], robot[1], map);
            }
        }

        System.out.println(" tt = " + tt * (180 / Math.PI));
        System.out.println(" tt = " + angles[135] * (180 / Math.PI));

        // Update the grid
        map.updateMap(image_grid, robotRow, robotCol, echoes, angles);
        System.out.println("grid x length: " + grid[0].length);
        return grid;
    }




    //Color the grid using bayes
    public void colorGrid(float[][] grid, int x, int y, int start_x, int start_y, ShowMap map) {
        LinkedList<Point> visited_points = map.drawBresenhamLine(start_x, start_y, x, y);
        double[] pos_robot_xy = map.rc_to_xy(start_x, start_y);
        Point point_robot = new Point(pos_robot_xy[0], pos_robot_xy[1]);

        for (Point point : visited_points) {
            float prob_cell = grid[(int)point.y][(int)point.y];
            double[] point_rayon_xy = map.rc_to_xy((int) point.x, (int) point.y);
            Point new_point = new Point(point_rayon_xy[0], point_rayon_xy[1]);
            double distance_robot_cell = point_robot.getDistance(new_point);
            float prob_occ;
            //if point is in region 2
            if(point != visited_points.getLast()) {
                prob_occ = map.Bayes(distance_robot_cell);
            }
            else { //point is in region 1
                prob_occ = map.Bayes_R1(distance_robot_cell);
            }
            float prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
            grid[(int) point.y][(int) point.x] = prob_occ_recur;
        }
    }

    /**
     * Get corresponding angles to each laser beam
     *
     * @param lpr
     * @return laser angles in radians
     */
    double[] getLaserAngles(LaserPropertiesResponse lpr) {
        int beamCount = (int) ((lpr.getEndAngle() - lpr.getStartAngle()) / lpr.getAngleIncrement()) + 1;
        double[] angles = new double[beamCount];
        double a = lpr.getStartAngle();
        for (int i = 0; i < beamCount; i++) {
            angles[i] = a;
            // We get roundoff errors if we use AngleIncrement. Use 1 degree in
            // radians instead
            a += 1 * Math.PI / 180;// lpr.getAngleIncrement();
        }
        return angles;
    }

    void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie);
        }
    }
}
