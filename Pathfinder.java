import java.util.IllegalFormatCodePointException;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Stack;

public class Pathfinder {

    private HeatmapTile[][] heatMap;
    private double tileWidth;
    private double tileHeight;
    private static double occupied = 0.5;

    public Pathfinder(double width, double height, double[][] map) {
        this.tileWidth = width;
        this.tileHeight = height;
        this.heatMap = new HeatmapTile[map.length][map[0].length];
        for (int i = 0; i < map.length; i++ ) {
            for (int j = 0; j < map[0].length; j++) {
                this.heatMap[i][j] = new HeatmapTile(i, j);
            }
        }
    }

    public Path findPath(Point start, Point goal, double[][] map) {
        resetHeatmap(map);
        heatWave(start, goal, map);
        if (heatMap[x2Grid(goal.getX())][y2Grid(goal.getY())].getChecked()) {
            return createPath(start, goal);
        }
        Point[] self = {start};
        return new Path(self);
    }

    //Resets grids of the Heatmap to their initial values
    private void resetHeatmap(double[][] map) {
        for (int i = 0; i < map.length; i++ ) {
            for (int j = 0; j < map[0].length; j++) {
                heatMap[i][j].reset();
            }
        }
    }

    private void heatWave(Point s, Point g, double[][] map) {
        Queue<HeatmapTile> waveFront = new LinkedList<>();

        //Setting tile from the start point on checked and adding it to the queue
        heatMap[x2Grid(s.getX())][y2Grid(s.getY())].setChecked(true);
        waveFront.add(heatMap[x2Grid(s.getX())][y2Grid(s.getY())]);

        //while the there are still unexplored tiles in the queue and the goal tile is not reached yet the heat expands
        //to the adjacent tiles and those are added to the queue
        while (waveFront.size() != 0) {
            if (!((waveFront.peek().getX() == x2Grid(g.getX())) && (waveFront.peek().getY() == y2Grid(g.getY())))) {
                expandWavefront(waveFront, map, waveFront.peek().getX(), waveFront.peek().getY());
                waveFront.remove();
            } else {
                break;
            }
        }
    }

    private Path createPath(Point s, Point g) {
        int x = x2Grid(g.getX());
        int y = y2Grid(g.getY());
        Stack<HeatmapTile> heatStack = new Stack<>();

        while (!((x2Grid(s.getX()) == x) && (y2Grid(s.getY()) == y))) {
            heatStack.push(heatMap[x][y]);
            switch (heatMap[x][y].getDir()) {
                case NORTH:
                    y++;
                    break;
                case EAST:
                    x++;
                    break;
                case SOUTH:
                    y--;
                    break;
                case WEST:
                    x--;
                    break;
            }
        }


        int i = 0;
        Point[] path = new Point[heatStack.size()];
        while (!heatStack.empty()) {
            path[i] = new Point(grid2x(heatStack.peek().getX()), grid2y(heatStack.peek().getY()));
            heatStack.pop();
            i++;
        }
        return new Path(path);
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

    private void expandWavefront(Queue<HeatmapTile> waveFront, double[][] map, int x, int y) {
        //Check North
        if ((y + 1) < map[0].length) {
            if ((!heatMap[x][y + 1].getChecked()) && (map[x][y + 1] < occupied)) {
                heatMap[x][y + 1].setChecked(true);
                heatMap[x][y + 1].setDir(HeatmapTile.Dir.SOUTH);
                waveFront.add(heatMap[x][y + 1]);
            } else {
                heatMap[x][y + 1].setChecked(true);
            }
        }

        //Check East
        if ((x + 1) < map.length) {
            if ((!heatMap[x + 1][y].getChecked()) && (map[x + 1][y] < occupied)) {
                heatMap[x + 1][y].setChecked(true);
                heatMap[x + 1][y].setDir(HeatmapTile.Dir.WEST);
                waveFront.add(heatMap[x + 1][y]);
            } else {
                heatMap[x + 1][y].setChecked(true);
            }
        }

        //Check South
        if ((y - 1) >= 0) {
            if ((!heatMap[x][y - 1].getChecked()) && (map[x][y - 1] < occupied)) {
                heatMap[x][y - 1].setChecked(true);
                heatMap[x][y - 1].setDir(HeatmapTile.Dir.NORTH);
                waveFront.add(heatMap[x][y - 1]);
            } else {
                heatMap[x][y - 1].setChecked(true);
            }
        }

        //Check West
        if ((x - 1) >= 0) {
            if ((!heatMap[x - 1][y].getChecked()) && (map[x - 1][y] < occupied)) {
                heatMap[x - 1][y].setChecked(true);
                heatMap[x - 1][y].setDir(HeatmapTile.Dir.EAST);
                waveFront.add(heatMap[x - 1][y]);
            } else {
                heatMap[x - 1][y].setChecked(true);
            }
        }
    }

}
