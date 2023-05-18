package se.oru.coordination.coordination_oru.utility;

import com.vividsolutions.jts.geom.Coordinate;
import se.oru.coordination.coordination_oru.motionplanner.OccupancyMap;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

/**
 * The {@code MapInspector} class is a graphical tool to inspect an {@code OccupancyMap}.
 * It allows the user to view the map and interact with it using mouse and keyboard inputs.
 *
 * @author fpa
 */
public class MapInspector extends JPanel implements MouseListener, MouseMotionListener, KeyListener {

    private static final long serialVersionUID = 906863784669776526L;
    private OccupancyMap om = null;
    private boolean occ = false;
    private Point p;

    /**
     * Creates a new {@code MapInspector} instance for the specified {@code OccupancyMap}.
     *
     * @param om The {@code OccupancyMap} to be inspected.
     */
    public MapInspector(OccupancyMap om) {
        super();
        addMouseMotionListener(this);
        addKeyListener(this);
        addMouseListener(this);
        setFocusable(true);
        this.om = om;
        this.setLayout(new BorderLayout());
        this.setPreferredSize(new Dimension(om.getPixelWidth(), om.getPixelHeight()));
        createFrame();
    }

    public static void main(String[] args) {
        String map = "maps/mine-map-paper-2023.yaml";
        var om = new OccupancyMap(map);
        var p = new MapInspector(om);
    }

    /**
     * Creates and displays the main frame of the application.
     */
    private void createFrame() {
        var f = new JFrame("Map inspector");
        f.setContentPane(this);
        f.setSize(1280, 1024);
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.setVisible(true);
    }

    /**
     * Paints the component on the screen.
     *
     * @param g The graphics context to be used for painting.
     */
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (!occ) g.drawImage(om.asBufferedImage(), 0, 0, null);
        else g.drawImage(om.asThresholdedBufferedImage(), 0, 0, null);
    }

    // MouseListener and MouseMotionListener methods
    @Override
    public void mouseDragged(MouseEvent e) {
        p = new Point(e.getX(), e.getY());
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        p = new Point(e.getX(), e.getY());
    }

    @Override
    public void mouseClicked(MouseEvent arg0) {
    }

    @Override
    public void mouseEntered(MouseEvent arg0) {
    }

    @Override
    public void mouseExited(MouseEvent arg0) {
    }

    @Override
    public void mousePressed(MouseEvent arg0) {
    }

    @Override
    public void mouseReleased(MouseEvent arg0) {
    }

    // KeyListener methods
    @Override
    public void keyPressed(KeyEvent arg0) {
    }

    @Override
    public void keyReleased(KeyEvent arg0) {
    }

    /**
     * Handles keyTyped events to provide interaction with the map.
     *
     * @param arg0 The {@code KeyEvent} representing the key typed event.
     */
    @Override
    public void keyTyped(KeyEvent arg0) {
        if (arg0.getKeyChar() == 'c') {
            try {
                var color = new Color(om.asBufferedImage().getRGB(p.x, p.y));
                Coordinate position = om.toWorldCoordiantes(p.x, p.y);
                System.out.println("--");
                System.out.println("Pixel (x,y) = (" + p.x + "," + p.y + ")");
                System.out.println("Position (x,y) = (" + position.x + "," + position.y + ")");
                System.out.println("Color (r,g,b,a) = (" + color.getRed() + "," + color.getGreen() + "," + color.getBlue() + "," + color.getAlpha() + ")");
                System.out.println("Occupancy map bit: " + om.asByteArray()[(p.y) * om.getPixelWidth() / 8 + (p.x) / 8]);
                System.out.println("Occupancy value: " + om.getOccupancyValue(p.x, p.y));
                System.out.println("State: " + (om.isOccupied(p.x, p.y) ? "occupied" : "free") + " (threshold is " + om.getThreshold() + ")");
            } catch (java.lang.ArrayIndexOutOfBoundsException e) {
                e.printStackTrace();
            }
        } else if (arg0.getKeyChar() == 'o') {
            occ = !occ;
            System.out.println("Showing " + (occ ? "threshold (" + om.getThreshold() + ")" : "original") + " map");
        }
        repaint();
    }

}