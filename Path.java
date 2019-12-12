import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Collection;
import java.util.Map;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

// Simple class that to read a path file and convert the points
// to objects of the type Point

// Thomas Johansson 2019 thomasj@cs.umu.se

public class Path
{
   public Point[] path;
   public int nextPoint = 0;

   @SuppressWarnings("unchecked")
   public Path(String fileName) {
      try {
          File pathFile = new File(fileName);

          BufferedReader in = new BufferedReader(new InputStreamReader(
                new FileInputStream(pathFile)));

          ObjectMapper mapper = new ObjectMapper();

          // read file and convert to Collection
          Collection <Map<String, Object>> data =
                (Collection<Map<String, Object>>) mapper.readValue(in, Collection.class);

          int nPoints = data.size();
          path = new Point[nPoints];

          // Loop through the Collection and extract pose, X, Y
          // make a new Point and put in list
          int index = 0;
          for (Map<String, Object> point : data)
          {
             Map<String, Object> pose = (Map<String, Object>)point.get("Pose");
             Map<String, Object> aPosition = (Map<String, Object>)pose.get("Position");

             double x = (Double)aPosition.get("X");
             double y = (Double)aPosition.get("Y");
             path[index++] = new Point(x, y);
          }
      } catch (Exception e) {
          throw new RuntimeException(e);
      }
   }

    public Point getGoalPoint(Point pos, double targetDistance) {
        while(!pathFinished() && path[nextPoint].getDistance(pos) < targetDistance) {
            nextPoint++;
            //System.out.println("Point #"+ nextPoint +" too close!");
        }
        return getNextPoint();
    }

    public boolean pathFinished() {
        return nextPoint+1 >= path.length;
    }

    public Point getNextPoint() {
        return path[nextPoint];
    }
   
   // Return the path as a list of Point objects
   public Point[] getPath()
   {
      return path;
   }
}
