package se.oru.coordination.coordination_oru.util;

import com.vividsolutions.jts.geom.Coordinate;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class DynamicMap {
    protected String filenameYAML = null;
    protected String textYAML = null;
    protected String imageFilename = null;

    public BufferedImage mapImageGrayscale = null;
    public BufferedImage mapImageBlackAndWhite = null; // only two colors
    public double resolution = -1;
    public double threshold = 0.3;
    public Coordinate origin = null;

    public static BufferedImage deepCopy(BufferedImage bi) {
        // Source: https://stackoverflow.com/a/3514297.
        ColorModel cm = bi.getColorModel();
        boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
        WritableRaster raster = bi.copyData(null);
        return new BufferedImage(cm, raster, isAlphaPremultiplied, null).getSubimage(
                0, 0, bi.getWidth(), bi.getHeight()
        );
    }

    public static void grayscaleImage(BufferedImage bimg) {
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

    public static void thresholdImage(BufferedImage bimg, double threshold) {
        assert 0.0 < threshold && threshold <= 1.0;
        for (int y = 0; y < bimg.getHeight(); y++) {
            for (int x = 0; x < bimg.getWidth(); x++) {
                Color color = new Color(bimg.getRGB(x, y));
                int graylevel = (color.getRed() + color.getGreen() + color.getBlue()) / 3;
                int level = graylevel < threshold * 255 ? 0 : 255;
                int r = level;
                int g = level;
                int b = level;
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
            while ((st = br.readLine()) != null) {
                if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
                    textYAML += st + "\n";
                    String key = st.substring(0, st.indexOf(":")).trim();
                    String value = st.substring(st.indexOf(":") + 1).trim();
                    if (key.equals("image")) imageFilename = file.getParentFile() + File.separator + value;
                    else if (key.equals("resolution")) resolution = Double.parseDouble(value);
                    else if (key.equals("occupied_thresh")) threshold = Double.parseDouble(value);
                    else if (key.equals("origin")) {
                        String x = value.substring(1, value.indexOf(",")).trim();
                        String y = value.substring(
                                value.indexOf(",") + 1,
                                value.indexOf(",", value.indexOf(",") + 1)
                        ).trim();
                        origin = new Coordinate(Double.parseDouble(x), Double.parseDouble(y));
                    }
                }
            }
            br.close();

            mapImageGrayscale = ImageIO.read(new File(imageFilename));
            grayscaleImage(mapImageGrayscale);

            mapImageBlackAndWhite = deepCopy(mapImageGrayscale);
            thresholdImage(mapImageBlackAndWhite, threshold); // TODO
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException();
        }
    }

    public DynamicMap(BufferedImage mapImageBlackAndWhite, double resolution, Coordinate origin) {
        this.mapImageBlackAndWhite = mapImageBlackAndWhite;
        this.resolution = resolution;
        this.origin = origin;
    }

    /**
     * Get the coordinates in pixel space corresponding to a given {@link Coordinate} in the workspace.
     *
     * @param coord A {@link Coordinate} within the workspace.
     * @return The coordinates in pixel space corresponding to the given {@link Coordinate} in the workspace.
     */
    public int[] toPixels(Coordinate coord) {
        return new int[]{
                (int) ((coord.x - origin.x) / resolution),
                mapImageBlackAndWhite.getHeight() - ((int) ((coord.y - origin.y) / resolution))
        };
    }

    /**
     * Get the {@link Coordinate}s in workspace corresponding to given coordinates in pixel space.
     *
     * @param x The x coordinate of the pixel in the occupancy map.
     * @param y The y coordinate of the pixel in the occupancy map.
     * @return The {@link Coordinate}s in workspace corresponding to given coordinates in pixel space.
     */
    public Coordinate toWorldCoordinates(int x, int y) {
        return new Coordinate(
                origin.x + (x + 0.5) * resolution,
                origin.y + (mapImageBlackAndWhite.getHeight() - y + 0.5) * resolution
        );
    }

    public void cleanCircle(Coordinate center, double radius) {
        int[] pixel = this.toPixels(center);
        // TODO
    }
}
