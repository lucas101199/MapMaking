/**
 * This class represents one element of a Heatmap. It stores the direction from which it was heated, its x and y value and
 * a boolean to determine if it is already heated.
 */
public class HeatmapTile {
    //Direction from where the heat cam
    enum Dir {
        NULL,
        NORTH,
        EAST,
        SOUTH,
        WEST
    }

    private Dir dir;
    private boolean checked;
    private int x;
    private int y;

    public HeatmapTile(int X, int Y) {
        this.dir = Dir.NULL;
        this.checked = false;
        this.x = X;
        this.y = Y;
    }

    public void reset() {
        this.dir = Dir.NULL;
        this.checked = false;
    }

    public void setDir(Dir direction) {dir = direction;}

    public void setChecked(boolean check) {checked = check;}

    public void setX(int X) {x = X;}

    public void setY(int Y) {y = Y;}

    public Dir getDir() {return dir;}

    public boolean getChecked() {return checked;}

    public int getX() {return x;}

    public int getY() {return y;}
}
