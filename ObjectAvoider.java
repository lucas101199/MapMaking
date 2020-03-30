/**
 * Class that contains the methods to recognize an obstacle in front and react to it.
 */
public class ObjectAvoider {
    private RobotCommunication comm = new RobotCommunication("http://127.0.0.1", 50000);
    private LocalizationResponse lr = new LocalizationResponse();
    private DifferentialDriveRequest dr = new DifferentialDriveRequest();
    private static double distance = 0.5;

    /**
     * Checks the laser echoes 25 degrees to the left and right of the main axis of the robot. If there are echoes smaller
     * than <code>distance</code> the robot turns around 180 degrees on the spot and then stops the robot completely.
     * @return True if there was an obstacle to avoid, false if not.
     * @throws Exception
     */
    public boolean avoidObject(double[] echoes) throws Exception {
        //Check 25 degrees left and right of the robot for obstacles
        for (int i = 110; i < 160; i++) {
                if (echoes[i] < distance) {
                    dr.setAngularSpeed(Math.PI * 0.5);
                    comm.putRequest(dr);
                    sleep(2000);
                    /*dr.setLinearSpeed(1);
                    dr.setAngularSpeed(Math.PI * 0);
                    comm.putRequest(dr);
                    sleep(1000);*/
                    dr.setLinearSpeed(0);

                    dr.setAngularSpeed(Math.PI * 0);
                    comm.putRequest(dr);
                    return true;
                }
        }
    return false;
    }

    /**
     * Computes the angles in degree for each laser beam relative to the line of sight of the robot.
     * @param lpr The response of the laser properties.
     * @param ler The laser echoes response.
     * @return
     */
    private double[] getAngles(LaserPropertiesResponse lpr, LaserEchoesResponse ler) {
        int nrOfBeams = ler.getEchoes().length;
        double[] angles = new double[nrOfBeams];
        double ang = lpr.getStartAngle();
        for (int i = 0; i < nrOfBeams; i++) {
            angles[i] = ang;
            //1 degree in radians instead of AngleIncrement because of rounding errors
            ang += 1 * Math.PI / 180;
        }
        return angles;
    }

    /**
     * Puts the thread to sleep for <code>ms</code> of time.
     * @param ms Number of ms the thread will be put to sleep.
     */
    static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie);
        }
    }

}
