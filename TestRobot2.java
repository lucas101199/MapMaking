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
    private double cell_size = 0.1;
    private double R = 4.0;
    public static int[] coord;
    public boolean first_time = true;
    public float[][] grid;
    public float[][] image_grid;
    public ShowMap map;

    public static boolean showGUI;

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
        x_min = Integer.parseInt(args[1]);
        y_min = Integer.parseInt(args[2]);
        x_max = Integer.parseInt(args[3]);
        y_max = Integer.parseInt(args[4]);
        coord = new int[]{x_min, y_min, x_max, y_max};
        showGUI = Boolean.parseBoolean(args[5]);
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
        double angle;
        double[] echoes;

        double[] angles;
        double[] robotPos;
        double[] laserPos;


        int nCols = (int) (Math.abs(x_max - x_min) / cell_size);
        int nRows = (int) (Math.abs(y_max - y_min) / cell_size);
        boolean showGUI = true; // set this to false if you run in putty
        map = new ShowMap(nRows, nCols, showGUI, coord, cell_size, R);

        ObjectAvoider objectAvoider = new ObjectAvoider();


        robotcomm.getResponse(lpr);
        robotcomm.getResponse(lr);
        robotcomm.getResponse(ler);


        angles = getLaserAngles(lpr);
        angle = lr.getHeadingAngle();
        echoes = ler.getEchoes();
        robotPos = lr.getPosition();
        laserPos = lpr.getPosition();
        grid = createMap(robotPos, echoes, angles, angle); // create an example map

        //Turn around
        dr.setAngularSpeed(Math.PI * 0.5);
        robotcomm.putRequest(dr);
        sleep(2000);
        dr.setLinearSpeed(0);
        dr.setAngularSpeed(Math.PI * 0);
        robotcomm.putRequest(dr);

        robotcomm.getResponse(lpr);
        robotcomm.getResponse(lr);
        robotcomm.getResponse(ler);
        angles = getLaserAngles(lpr);
        angle = lr.getHeadingAngle();
        echoes = ler.getEchoes();
        robotPos = lr.getPosition();
        laserPos = lpr.getPosition();
        grid = createMap(robotPos, echoes, angles, angle); // create an example map

        Pathfinder scout = new Pathfinder(cell_size, cell_size, (double) x_min, (double) y_min, grid);
        boolean end_of_map = true;
        while (!scout.finished) {

            //stop if the robot has discover 90% of the map
            end_of_map = Stop_robot(grid);

            robotcomm.getResponse(lpr);
            robotcomm.getResponse(lr);
            robotcomm.getResponse(ler);
            angles = getLaserAngles(lpr);
            angle = lr.getHeadingAngle();
            echoes = ler.getEchoes();
            robotPos = lr.getPosition();

            grid = createMap(robotPos, echoes, angles, angle); // create an example map


        ObjectAvoider objectAvoider = new ObjectAvoider();


            //Compute A Path
            int[] rob_pos_grid = map.xy_to_rc(robotPos[0], robotPos[1]);

            Point start = new Point(rob_pos_grid);

            Path path = scout.findPath(start, grid);
            System.out.println("Pathlength: " + path.path.length);

            map.showGoal(grid, scout.pub_goal.getX(), scout.pub_goal.getY());

            //sleep(1000);
            //Follow the Path
            PathFollower follower = new PathFollower(path);
            System.out.println("Pathfollowing begins");

            Point pos = new Point(robotPos);
            while ( (!path.pathFinished()) ) {
                /*dr.setLinearSpeed(0);
                dr.setAngularSpeed(0);
                robotcomm.putRequest(dr);
                sleep(200);*/
                robotcomm.getResponse(lpr);
                robotcomm.getResponse(lr);
                robotcomm.getResponse(ler);
                angles = getLaserAngles(lpr);
                angle = lr.getHeadingAngle();
                echoes = ler.getEchoes();
                robotPos = lr.getPosition();

                if (objectAvoider.avoidObject(echoes)) { //if obstacle encounter stop the robot
                    grid = createMap(robotPos, echoes, angles, angle);
                    map.showGoal(grid, scout.pub_goal.getX(), scout.pub_goal.getY());
                    System.out.println("Object Avoidance");
                    break;
                }

                follower.step(pos, lr, angle);

                grid = createMap(robotPos, echoes, angles, angle); // create an example map
                map.showGoal(grid, scout.pub_goal.getX(), scout.pub_goal.getY());
                pos.setX(lr.getPosition()[0]);
                pos.setY(lr.getPosition()[1]);

                follower.sleep(30);
            }
            System.out.println("Path finished!\n");
            dr.setAngularSpeed(Math.PI * 0);
            dr.setLinearSpeed(0);
            robotcomm.putRequest(dr);
            sleep(3000);
        }
        sleep(5000);
        System.out.println("Map creation finished!");

    }


    /**
     * A simple example of how to use the ShowMap class that creates a map from
     * your grid, update it and save it to file
     */

    private float[][] createMap(double[] robotPosition,
                           double[] echoes, double[] angles, double heading_angle) {
        /* use the same no. of rows and cols in map and grid */
        int nCols = (int) (Math.abs(x_max - x_min) / cell_size);
        int nRows = (int) (Math.abs(y_max - y_min) / cell_size);


        /* Creating a grid with 0.5 */
        if (first_time) {
            first_time = false;
            grid = new float[nRows][nCols];
            image_grid = new float[nRows][nCols];
            for (int i = nRows - 1; i >= 0; i--) {

                for (int j = 0; j < nCols; j++) {

                    grid[i][j] = (float) 0.5;
                    image_grid[i][j] = (float) 0.5;
                }
            }
        }

        double[] laserPosition = new double[2];
        laserPosition[0] = robotPosition[0] + (0.15 *  Math.cos(heading_angle));
        laserPosition[1] = robotPosition[1] + (0.15 *  Math.sin(heading_angle));





        for (int i = 0; i < echoes.length; i++) {

            double[] obstacle = new double[2];
            double[] obstacle_image = new double[2];


            obstacle[1] = laserPosition[1] + (echoes[i] * Math.sin(angles[i] + heading_angle)); // y2 = y1 + (lenght * sin(angle)) grid without minus for path planning
            obstacle[0] = laserPosition[0] + (echoes[i] *  Math.cos(angles[i] + heading_angle)); // x2 = x1 + (lenght * cos(angle)) angle in radians
            obstacle_image[0] = laserPosition[0] + (echoes[i] *  Math.cos(angles[i] + heading_angle)); // x2 = x1 + (lenght * cos(angle)) angle in radians
            obstacle_image[1] = laserPosition[1] + (echoes[i] * Math.sin(angles[i] + heading_angle)); // y2 = y1 + (lenght * sin(angle)) grid with minus


            colorGrid(grid, echoes[i], obstacle, laserPosition, map);
            colorGrid(image_grid, echoes[i], obstacle_image, laserPosition, map);


        }

        //int[] robPosGrid = map.xy_to_rc(robotPosition[0], robotPosition[1]);
        //float[][] grownObstacles = growObstacles(new Point(robPosGrid), grid);

        // Update the grid
        map.updateMap(grid, robotPosition[1], robotPosition[0]);

        return grid;
    }

    //Color the grid using bayes
    public void colorGrid(float[][] grid, double echo, double[] obstacle, double[] laserPos, ShowMap map) {
        int[] gridObstacle = map.xy_to_rc(obstacle[0], obstacle[1]);
        int[] gridLaserPos = map.xy_to_rc(laserPos[0], laserPos[1]);
        LinkedList<Point> visited_points = map.drawBresenhamLine(gridLaserPos[0], gridLaserPos[1], gridObstacle[0], gridObstacle[1]);
        Point point_robot = new Point(laserPos[0], laserPos[1]);

        for (Point point : visited_points) {
            double[] point_rayon_xy = map.rc_to_xy((int) point.x, (int) point.y);
            float prob_cell = grid[(int)point.y][(int)point.x];
            Point new_point = new Point(point_rayon_xy[0], point_rayon_xy[1]);
            double distance_robot_cell = point_robot.getDistance(new_point);
            float prob_occ;
            float prob_occ_recur = prob_cell;
            //check if the echo is in 10m range
            //System.out.println("Distance to Object: " + echo);
            //System.out.println("Distance to Cell: " + distance_robot_cell);
            if (echo < R) {
                //System.out.println("Within R");
                //Check if cell is in region 2
                if (distance_robot_cell < (echo - 0.05)) {
                    //System.out.println("Cell in Region 2");
                    prob_occ = map.Bayes(distance_robot_cell);
                    prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
                } else {
                    if (distance_robot_cell > (echo + 0.05)) {
                        //System.out.println("Cell in Region 3");
                        // cell in region 3
                        prob_occ_recur = prob_cell;
                    } else {
                        //System.out.println("Cell in Region 1");
                        //Check if the cell is in region 1
                        prob_occ = map.Bayes_R1(distance_robot_cell);
                        prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
                    }
                }
            } else {
                //The echo is grater than R so every cell within distance R is free (Region 2)
                if (distance_robot_cell < R) {
                    //System.out.println("Echo grater R");
                    prob_occ = map.Bayes(distance_robot_cell);
                    prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
                }
            }

            //float prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
            //System.out.println("Occupancy probabillity: " + prob_occ_recur + "\n");
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

    public boolean Stop_robot(float[][] grid) {
        double percentage_map = 0;
        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[0].length; j++) {
                if (grid[i][j] == 0.5) {
                    percentage_map += 1;
                }
            }
        }
        boolean stop = true;
        double percentage = percentage_map / (grid.length * grid[0].length);
        if (percentage < 0.1) {
            stop = false;
        }
        return stop;
    }


    public boolean Stop_robot(float[][] grid) {
        double percentage_map = 0;
        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[0].length; j++) {
                if (grid[i][j] == 0.5) {
                    percentage_map += 1;
                }
            }
        }
        boolean stop = true;
        double percentage = percentage_map / (grid.length * grid[0].length);
        if (percentage < 0.1) {
            stop = false;
        }
        return stop;
    }
}
