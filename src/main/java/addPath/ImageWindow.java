package addPath;
//
//import java.awt.*;
//import java.awt.event.*;
//import java.awt.geom.AffineTransform;
//import java.awt.geom.Point2D;
//import java.awt.image.BufferedImage;
//import java.io.*;
//import javax.imageio.ImageIO;
//import javax.swing.*;
//
//public class ImageWindow extends JFrame {
//
//    private BufferedImage image;
//    private Point clickedPoint;
////    private Point originalPoint;
//    int width1 = 600;
//    int height1 = 400;
//    int widthOrginal;
//    int heightOrginal;
//    double k_width;
//    double k_height;
//    String fileName="";
//    public static JTextArea textArea=null;
//
//    public ImageWindow(File file) {
//        try {
//            image = ImageIO.read(file);
//            widthOrginal = image.getWidth();
//            heightOrginal = image.getHeight();
//            k_width = widthOrginal/width1;
//            k_height = heightOrginal/height1;
//            
//            ImageIcon img = new ImageIcon(fileName);
//            Image ii = img.getImage().getScaledInstance(450, 300, Image.SCALE_SMOOTH);
//
////            image = resize(image, width1, height1); // تصغير الصورة إلى حجم النافذة
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//
//        ImagePanel imagePanel = new ImagePanel(image);
//        setContentPane(imagePanel);
//
//        imagePanel.addMouseListener(new MouseAdapter() {
//            @Override
//            public void mouseClicked(MouseEvent e) {
//                clickedPoint = e.getPoint();
//                // تحويل نقطة الفأرة إلى نظام إحداثيات الصورة الأصلي
//                AffineTransform transform = new AffineTransform();
//                transform.scale(1.0 * image.getWidth() / imagePanel.getWidth(),
//                        1.0 * image.getHeight() / imagePanel.getHeight());
//                Point2D originalPoint2D = transform.transform(clickedPoint, null);
//                double widthOrginal1 = originalPoint2D.getX()*k_width;
//                double heightOrginal1 = originalPoint2D.getY()*k_height;
////                originalPoint = new Point((int) originalPoint2D.getX(), (int) originalPoint2D.getY());
//                imagePanel.repaint();
//                
//                double x=(widthOrginal1)/10;
//                double y=(heightOrginal - heightOrginal1)/10;
//                
//                System.out.println("x = "+x+", y = "+y);
//                
//                if(textArea.getText().isEmpty())
//                {
//                    textArea.setText("("+x+","+y+",0.0)");
//                }
//                else
//                {
//                    textArea.setText(textArea.getText()+" -> ("+x+","+y+",0.0)");
//                }
//            }
//        });
//
//        setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
//        setTitle(file.getName());
//        setSize(width1, height1+37);
//        setLocationRelativeTo(null);
//        setVisible(true);
//    }
//
//    public static void addPath(String fileName) {
//        File file = new File(fileName);
//        new ImageWindow(file);
//    }
//
//    // تصغير الصورة إلى الحجم الجديد
//    private BufferedImage resize(BufferedImage img, int width, int height) {
//        BufferedImage resizedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
//        Graphics2D g = resizedImage.createGraphics();
//        g.drawImage(img, 0, 0, width, height, null);
//        g.dispose();
//        return resizedImage;
//    }
//
//    private class ImagePanel extends JPanel {
//        private BufferedImage image;
//
//        public ImagePanel(BufferedImage image) {
//            this.image = image;
//            setPreferredSize(new Dimension(image.getWidth(), image.getHeight()));
//        }
//
//        @Override
//        protected void paintComponent(Graphics g) {
//            super.paintComponent(g);
//            g.drawImage(image, 0, 0, null);
//
//            // if (clickedPoint != null) {
//            //     // تحويل النقطة المحددة إلى نظام إحداثيات الصورة الأصلي
//            //     AffineTransform transform = new AffineTransform();
//            //     transform.scale(1.0 * getWidth() / image.getWidth(),
//            //             1.0 * getHeight() / image.getHeight());
//            //     Point2D scaledPoint2D = transform.transform(originalPoint, null);
//            //     Point scaledPoint = new Point((int) scaledPoint2D.getX(), (int) scaledPoint2D.getY());
//    
//            //     // رسم النقطة الحمراء على الصورة في الموقع المحدد
//            //     g.setColor(Color.RED);
//            //     g.fillOval(scaledPoint.x - 5, scaledPoint.y - 5, 10, 10);
//            // }
//            if (clickedPoint != null) {
//                g.setColor(Color.RED);
//                int radius = 10;
//                int x = (int) clickedPoint.getX() - radius / 2;
//                int y = (int) clickedPoint.getY() - radius / 2;
//                g.drawOval(x, y, radius, radius);
//            }
//        }
//    }
//}
//    



import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.io.*;
import javax.imageio.ImageIO;
import javax.swing.*;

public class ImageWindow extends JFrame {

    private BufferedImage image;
    private Point clickedPoint;
    int width1 = 600;
    int height1 = 400;
    int widthOrginal;
    int heightOrginal;
    double k_width;
    double k_height;
    String fileName="";
    public static JTextArea textArea=null;

    public static void addPath(String fileName1) {
        ImageWindow obj = new ImageWindow();
        
        obj.ImageWindow2(new File(fileName1));
    }

    public void ImageWindow2(File file) {
        try {
            image = ImageIO.read(file);
            widthOrginal = image.getWidth();
            heightOrginal = image.getHeight();
            k_width = widthOrginal/width1;
            k_height = heightOrginal/height1;
            

        } catch (Exception e) {
            e.printStackTrace();
        }

        ImagePanel imagePanel = new ImagePanel(image);
        JScrollPane scrollPane = new JScrollPane(imagePanel);
        scrollPane.getViewport().setPreferredSize(new Dimension(image.getWidth(), image.getHeight()));

        imagePanel.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                clickedPoint = e.getPoint();
                // تحويل نقطة الفأرة إلى نظام إحداثيات الصورة الأصلي
                AffineTransform transform = new AffineTransform();
                transform.scale(1.0 * image.getWidth() / imagePanel.getWidth(),
                        1.0 * image.getHeight() / imagePanel.getHeight());
                Point2D originalPoint2D = transform.transform(clickedPoint, null);
                double widthOrginal1 = originalPoint2D.getX();
                double heightOrginal1 = originalPoint2D.getY();
                imagePanel.repaint();
                
                double x=(widthOrginal1)/10;
                double y=(heightOrginal-heightOrginal1)/10;
                
                System.out.println("x = "+x+", y = "+y +">>>>>>"+heightOrginal);
                
                 if(textArea.getText().isEmpty())
                 {
                     textArea.setText("("+x+","+y+",0.0)");
                 }
                 else
                 {
                     textArea.setText(textArea.getText()+" -> ("+x+","+y+",0.0)");
                 }
            }
        });

        getContentPane().add(scrollPane);

        setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        setTitle(file.getName());
        setSize(width1, height1+37);
        setLocationRelativeTo(null);
        setVisible(true);
        setResizable(false); // منع تكبير النافذة

    }

    // تصغير الصورة إلى الحجم الجديد
    private BufferedImage resize(BufferedImage img, int width, int height) {
        BufferedImage resizedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = resizedImage.createGraphics();
        g.drawImage(img, 0, 0, width, height, null);
        g.dispose();
        JScrollPane scrollPane = new JScrollPane(new JLabel(new ImageIcon(resizedImage)));
        getContentPane().removeAll();
        getContentPane().add(scrollPane, BorderLayout.CENTER);
        setSize(new Dimension(width, height));
        setLocationRelativeTo(null);
        revalidate();
        repaint();
        return resizedImage;
    }
    
    private class ImagePanel extends JPanel {
        private BufferedImage image;
    
        public ImagePanel(BufferedImage image) {
            this.image = image;
            setPreferredSize(new Dimension(image.getWidth(), image.getHeight()));
        }
    
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            g.drawImage(image, 0, 0, null);
    
            if (clickedPoint != null) {
                g.setColor(Color.RED);
                int radius = 10;
                int x = (int) clickedPoint.getX() - radius / 2;
                int y = (int) clickedPoint.getY() - radius / 2;
                g.drawOval(x, y, radius, radius);
            }
        }
    }
}    