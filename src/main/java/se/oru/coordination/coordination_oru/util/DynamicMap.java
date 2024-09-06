package se.oru.coordination.coordination_oru.util;

import com.vividsolutions.jts.geom.Coordinate;
import se.oru.coordination.coordination_oru.motionplanning.OccupancyMap;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.BitSet;

public class DynamicMap {
    protected String filenameYAML = null;
    protected String textYAML = null;
    protected String imageFilename = null;
    
    public BufferedImage mapImage = null;
    public double resolution = -1;
    public double threshold = 0.3;
    public Coordinate origin = null;

    public static void thresholdImage(BufferedImage bimg, double threshold) {
        for (int y = 0; y < bimg.getHeight(); y++) {
            for (int x = 0; x < bimg.getWidth(); x++) {
                Color color = new Color(bimg.getRGB(x, y));
                int graylevel = (color.getRed() + color.getGreen() + color.getBlue()) / 3;
                int r = graylevel;
                int g = graylevel;
                int b = graylevel;
                int rgb = 0xff000000 | (r << 16) | (g << 8) | b;
                bimg.setRGB(x, y, rgb);
            }
        }
    }
    
    public DynamicMap(String mapYAMLFile) {
        try {
            filenameYAML = mapYAMLFile;
            File file = new File(mapYAMLFile);
            BufferedReader br = new BufferedReader(new FileReader(file));
            String st;
            textYAML = "";
            while((st=br.readLine()) != null){
                if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
                    textYAML += st+"\n";
                    String key = st.substring(0, st.indexOf(":")).trim();
                    String value = st.substring(st.indexOf(":")+1).trim();
                    if (key.equals("image")) imageFilename = file.getParentFile()+File.separator+value;
                    else if (key.equals("resolution")) resolution = Double.parseDouble(value);
                    else if (key.equals("occupied_thresh")) threshold = Double.parseDouble(value);
                    else if (key.equals("origin")) {
                        String x = value.substring(1, value.indexOf(",")).trim();
                        String y = value.substring(value.indexOf(",")+1, value.indexOf(",", value.indexOf(",")+1)).trim();
                        origin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
                    }
                }
            }
            br.close();
            mapImage = ImageIO.read(new File(imageFilename));
            thresholdImage(mapImage, threshold); // TODO
        }
        catch (IOException e) { e.printStackTrace(); throw new RuntimeException(); }
    }

    public DynamicMap(BufferedImage mapImage, double resolution, Coordinate origin) {
        this.mapImage = mapImage;
        this.resolution = resolution;
        this.origin = origin;
    }
}
