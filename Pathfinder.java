import java.util.*;

/**
 * This class is used to find the next suitable go to point in the world that presumably yields the most exploration
 * potential. Then the shortest path from the current position to this point is calculated and returend.
 */
public class Pathfinder {

    private HeatmapTile[][] heatMap;
    private double tileWidth;
    private double tileHeight;
    private static float unknown = 0.5f;
    private double x_min;
    private double y_min;


    public Pathfinder(double width, double height, double x_min, double y_min, float[][] map) {
        this.tileWidth = width;
        this.tileHeight = height;
        this.x_min = x_min;
        this.y_min = y_min;
        this.heatMap = new HeatmapTile[map.length][map[0].length];
        for (int i = 0; i < map.length; i++ ) {
            for (int j = 0; j < map[0].length; j++) {
                this.heatMap[i][j] = new HeatmapTile(j, i);
            }
        }

    }

    /**
     * Takes position of the robot and the regular grid. Computes a goal point on frontier based method and creates a path
     * how to get there and returns it.
     * @param start The position of the robot in world coordinates.
     * @param map The regular grid of the world.
     * @return Array of points that mark the path to the goal point.
     */
    public Path findPath(Point start, float[][] map) {

        resetHeatmap(map);
        Point goal = getGoal(map);

        resetHeatmap(map);
        heatWave(start, goal, map);


        if (heatMap[y2Grid(goal.getY())][x2Grid(goal.getX())].getChecked()) {
            System.out.println("Path creation starts");
            return createPath(start, goal);
        }
        System.out.println("Error in Path creation!");
        Point[] self = {start};
        return new Path(self);
    }

    /**
     * Resets the <code>HeatmapTiles</code> in the <code>heatMap</code> to their initial values
     * @param map the regula grid that represents the world
     */
    private void resetHeatmap(float[][] map) {
        for (int i = 0; i < map.length; i++ ) {
            for (int j = 0; j < map[0].length; j++) {
                heatMap[i][j].reset();
            }
        }
    }

    /**
     * Marks the starting tile of the heatmap as checked and initializes the LinkedList that is used to store the <code>HeatmapTiles</code>
     * that currently build the wavefront. Expands the wavefront until one of the tiles is the goal point.
     * @param s Robot position in world coordinates
     * @param g Goal point in world coordinates
     * @param map Regular grid of the world
     */
    private void heatWave(Point s, Point g, float[][] map) {
        Queue<HeatmapTile> waveFront = new LinkedList<>();

        //Setting tile from the start point on checked and adding it to the queue
        heatMap[y2Grid(s.getY())][x2Grid(s.getX())].setChecked(true);
        waveFront.add(heatMap[y2Grid(s.getY())][x2Grid(s.getX())]);
        //while the there are still unexplored tiles in the queue and the goal tile is not reached yet the heat expands
        //to the adjacent tiles and those are added to the queue
        int cnt = 0;
        while (waveFront.size() != 0) {
            if ( (waveFront.peek().getX() != x2Grid(g.getX())) || (waveFront.peek().getY() != y2Grid(g.getY())) ) {
                expandWavefront(waveFront, map, waveFront.peek().getX(), waveFront.peek().getY());
                waveFront.remove();
                cnt++;
            } else {
                heatMap[waveFront.peek().getY()][waveFront.peek().getX()].setChecked(true);
                waveFront.peek().setChecked(true);
                break;
            }
        }
    }

    /**
     * Checks if the tiles in northward, eastward, southward and westward direction of the Tile, whose x and y value are
     * handed over to method, are still in the bounds of the regular grid or not and if those are free or blocked by an obstacle.
     * If they are free the tiles direction value is marked correspondent from which direction they were explored and they
     * are appended to the <code>waveFront</code>. Undependet of if the tiles are free or not their boolean <code>checked</code>
     * is set to true to mark them as checked.
     * @param waveFront LinkeList that stores the <code>HeatmapTile</code> that currently build the wavefront.
     * @param map The regular grid that represents the world
     * @param x The x value of the HeatmapTile whose neighbors should be explored
     * @param y The y value of the HeatmapTile whose neighbors should be explored
     */
    private void expandWavefront(Queue<HeatmapTile> waveFront, float[][] map, int x, int y) {
        //Check North
        if (((y + 1) < map.length) && (x < map[0].length) && (x >= 0)) {
            if ((!heatMap[y + 1][x].getChecked()) && (map[y + 1][x] < unknown)) {
                heatMap[y + 1][x].setChecked(true);
                heatMap[y + 1][x].setDir(HeatmapTile.Dir.SOUTH);
                waveFront.add(heatMap[y + 1][x]);
            } else {
                heatMap[y + 1][x].setChecked(true);
            }
        }

        //Check East
        if (((x + 1) < map[0].length) && (y < map.length) && (y >= 0)) {
            if ((!heatMap[y][x + 1].getChecked()) && (map[y][x + 1] < unknown)) {
                heatMap[y][x + 1].setChecked(true);
                heatMap[y][x + 1].setDir(HeatmapTile.Dir.WEST);
                waveFront.add(heatMap[y][x + 1]);
            } else {
                heatMap[y][x + 1].setChecked(true);
            }
        }

        //Check South
        if (((y - 1) >= 0) && (x < map[0].length) && (x >= 0)) {
            if ((!heatMap[y - 1][x].getChecked()) && (map[y - 1][x] < unknown)) {
                heatMap[y - 1][x].setChecked(true);
                heatMap[y - 1][x].setDir(HeatmapTile.Dir.NORTH);
                waveFront.add(heatMap[y - 1][x]);
            } else {
                heatMap[y - 1][x].setChecked(true);
            }
        }

        //Check West
        if (((x - 1) >= 0) && (y < map.length) && (y >= 0)){
            if ((!heatMap[y][x - 1].getChecked()) && (map[y][x - 1] < unknown)) {
                heatMap[y][x - 1].setChecked(true);
                heatMap[y][x - 1].setDir(HeatmapTile.Dir.EAST);
                waveFront.add(heatMap[y][x - 1]);
            } else {
                heatMap[y][x - 1].setChecked(true);
            }
        }
    }


    /**
     * This method starts at the goal point and safes the element in a stack and checks which direction is safed in this element.
     * Then the next element of the heatmap is safed in the stack. This is repeated until the start point (the robot position)
     * is reached.
     * In the next step the elements are pushed one by one from the stack and for each element a corresponding point is
     * generated (the middle of the grid tile) and safed in the path array which is then returned.
     * @param s The position of the robot in world coordinates
     * @param g The goal point in world coordinates
     * @return An array of points which build the path to the goal point.
     */
    private Path createPath(Point s, Point g) {
        int x = x2Grid(g.getX());
        int y = y2Grid(g.getY());
        int goal_x = x2Grid(s.getX());
        int goal_y = y2Grid(s.getY());
        Stack<HeatmapTile> heatStack = new Stack<>();

        while (((goal_x != x) || (goal_y != y))) {
            heatStack.push(heatMap[y][x]);
            switch (heatMap[y][x].getDir()) {
                case NORTH:
                    y = y + 1;
                    break;
                case EAST:
                    x = x + 1;
                    break;
                case SOUTH:
                    y = y - 1;
                    break;
                case WEST:
                    x = x - 1;
                    break;
            }
        }

        int i = 0;
        Point[] path = new Point[heatStack.size()];
        while (!heatStack.empty()) {
            path[i] = new Point(grid2x(heatStack.peek().getX()) - Math.abs(x_min), grid2y(heatStack.peek().getY()) - Math.abs(y_min));
            heatStack.pop();
            i++;
        }
        return new Path(path);
    }

    /**
     * Computes a frontier based goal point by saving all tiles forming one frontier in a list and saving all those frontier lists in another list.
     * Then the frontiers are compared by size and the middlepoint of the longest frontier is the goalpoint.
     * @param map The grid wich contains the knowledge about the world
     * @return Middlepoint of the longest Frontier.
     */
    private Point getGoal(float[][] map) {
        LinkedList<LinkedList<HeatmapTile>> frontiers = new LinkedList<>();

        for (int i = 0; i < map.length; i ++) {
            for (int j = 0; j < map[0].length; j++) {
                if ( (map[i][j] == unknown) && (neighborObstacle(i, j, map)) && (neighborKnown(i, j, map)) && (!heatMap[i][j].getChecked()) ) {
                    LinkedList<HeatmapTile> frontline = new LinkedList<>();
                    heatMap[i][j].setChecked(true);
                    frontline.add(heatMap[i][j]);
                    ListIterator<HeatmapTile> iterator = frontline.listIterator();
                    while (iterator.hasNext()) {
                        addCandidates(iterator.next(), frontline, map);
                    }
                    frontiers.add(frontline);
                }
            }
        }

        int longest = 0;
        int max_length = 0;
        for (int i = 0; i < frontiers.size(); i++) {
            if (frontiers.get(i).size() > max_length) {
                max_length = frontiers.get(i).size();
                longest = i;
            }
        }

        HeatmapTile gt = frontiers.get(longest).get(max_length/2);
        return new Point(grid2x(gt.getX()), grid2y(gt.getY()));
    }

    /**
     * Checks if the neighbors of the <code>start</code> tile have at least one neighbor that is already known. If so the
     * new <code>HeatmapTile</code> is checked and added to the <code>frontline</code>.
     * @param start The tile whose neighbors are to be checked.
     * @param frontline List that contains the tiles the frontier consists of
     * @param map The grid wich contains the knowledge about the world
     */
    private void addCandidates(HeatmapTile start, LinkedList<HeatmapTile> frontline, float[][] map) {
        int y = start.getY();
        int x = start.getX();

        //Check north
        if ( ((y + 1) < map.length) && (!heatMap[y + 1][x].getChecked()) && (map[y + 1][x] == unknown) && (neighborKnown(y + 1, x, map)) ) {
            heatMap[y + 1][x].setChecked(true);
            frontline.add(heatMap[y + 1][x]);
            return;
        }

        //Check East
        if ( ((x + 1) < map[0].length) && (!heatMap[y][x + 1].getChecked()) && (map[y][x + 1] == unknown) && (neighborKnown(y, x + 1, map)) ) {
            heatMap[y][x + 1].setChecked(true);
            frontline.add(heatMap[y][x + 1]);
            return;
        }

        //Check South
        if ( ((y - 1) >= 0) && (!heatMap[y - 1][x].getChecked()) && (map[y - 1][x] == unknown) && (neighborKnown(y - 1, x, map)) ) {
            heatMap[y - 1][x].setChecked(true);
            frontline.add(heatMap[y - 1][x]);
            return;
        }

        //Check West
        if ( ((x - 1) >= 0) && (!heatMap[y][x - 1].getChecked()) && (map[y][x - 1] == unknown) && (neighborKnown(y, x - 1, map)) ) {
            heatMap[y][x - 1].setChecked(true);
            frontline.add(heatMap[y][x - 1]);
        }
    }

    /**
     * Checks if one of the neighbors of tile <code>map[y][x]</code> is an obstacle and returns true if so.
     * @param y
     * @param x
     * @param map The grid wich contains the knowledge about the world
     * @return
     */
    private boolean neighborObstacle(int y, int x, float[][] map) {
        //Check north
        if ( ((y + 1) < map.length) && (map[y + 1][x] > unknown) ) {return true;}
        
        //Check northeast
        if ( ((y + 1) < map.length) && ((x + 1) < map[0].length) && (map[y + 1][x + 1] > unknown)) {return true;}
        
        //Check east
        if ( ((x + 1) < map[0].length) && (map[y][x + 1] > unknown) ) {return true;}
        
        //Check southeast
        if ( ((y - 1) >= 0) && ((x + 1) < map[0].length) && (map[y - 1][x + 1] > unknown)) {return true;}
        
        //Check south
        if ( ((y - 1) >= 0) && (map[y - 1][x] > unknown) ) {return true;}
        
        //Check southwest
        if ( ((y - 1) >= 0) && ((x - 1) >= 0) && (map[y - 1][x - 1] > unknown)) {return true;}
        
        //Check west
        if ( ((x - 1) >= 0) && (map[y][x - 1] > unknown) ) {return true;}
        
        //Check northwest
        if ( ((y + 1) < map.length) && ((x - 1) >= 0) && (map[y + 1][x - 1] > unknown)) {return true;}
        
        return false;
    }

    /**
     * Checks if one of the neighbors of tile <code>map[y][x]</code> is unknown and returns true if so.
     * @param y
     * @param x
     * @param map The grid wich contains the knowledge about the world
     * @return
     */
    private boolean neighborKnown(int y, int x, float[][] map) {
    //Check north
        if ( ((y + 1) < map.length) && (map[y + 1][x] < unknown) ) {return true;}

    //Check northeast
        if ( ((y + 1) < map.length) && ((x + 1) < map[0].length) && (map[y + 1][x + 1] < unknown)) {return true;}

    //Check east
        if ( ((x + 1) < map[0].length) && (map[y][x + 1] < unknown) ) {return true;}

    //Check southeast
        if ( ((y - 1) >= 0) && ((x + 1) < map[0].length) && (map[y - 1][x + 1] < unknown)) {return true;}

    //Check south
        if ( ((y - 1) >= 0) && (map[y - 1][x] < unknown) ) {return true;}

    //Check southwest
        if ( ((y - 1) >= 0) && ((x - 1) >= 0) && (map[y - 1][x - 1] < unknown)) {return true;}

    //Check west
        if ( ((x - 1) >= 0) && (map[y][x - 1] < unknown) ) {return true;}

    //Check northwest
        if ( ((y + 1) < map.length) && ((x - 1) >= 0) && (map[y + 1][x - 1] < unknown)) {return true;}

        return false;
}



    //Computes grids X number out of X-value
    private int x2Grid(double xValue) {
        return (int)(xValue / tileWidth);
    }

    //Computes grids Y number out of Y-value
    private int y2Grid(double yValue) {
        return (int)(yValue / tileHeight);
    }

    //Computes X-value (middle off grid) out of grids X number
    private double grid2x(int column) {
        return (column * tileWidth + (tileWidth / 2));
    }

    //Computes Y-value (middle of grid) out of grids Y number
    private double grid2y(int row) {
        return (row * tileHeight + (tileHeight / 2));
    }

}
