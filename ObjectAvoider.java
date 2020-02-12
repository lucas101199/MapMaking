public class ObjectAvoider {
    private RobotCommunication comm = new RobotCommunication("http://127.0.0.1", 50000);
    private LaserEchoesResponse ler = new LaserEchoesResponse();
    private LaserPropertiesResponse lpr = new LaserPropertiesResponse();
    private LocalizationResponse lr = new LocalizationResponse();
    private DifferentialDriveRequest dr = new DifferentialDriveRequest();

    public void avoidObject() throws Exception {
        comm.getResponse(lr);
        comm.getResponse(ler);
        comm.getResponse(lpr);


        double[] echoes = ler.getEchoes();
        double[] angles = getAngles(lpr, ler);
        //Check 45 degrees left and right of the robot for obstacles
        double leftBound = lr.getHeadingAngle() - (45 * Math.PI / 180);
        double rightBound = lr.getHeadingAngle() + (45 * Math.PI / 180);
        for (int i = 0; i < echoes.length; i++) {
            if ((angles[i] > leftBound) && (angles[i] < rightBound)) {
                if (echoes[i] < 0.5) {
                    //stopping the robot and exiting the loop
                    //next step could be to plan a new path
                    dr.setLinearSpeed(0);
                    dr.setAngularSpeed(0);
                    break;
                }
            }
        }


    }

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
}
