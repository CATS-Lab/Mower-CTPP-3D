package ImageProcess;

import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;

public class MatrixRotator {

    public int[][][] rotate(int[][][] pixels, int angleDegrees) {
         
        int width = pixels.length;
        int height = pixels[0].length;

        double angleRadians = Math.toRadians(angleDegrees);  

        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        WritableRaster raster = image.getRaster();
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                raster.setPixel(x, y, pixels[x][y]);
            }
        }

         
        int newWidth = (int) Math.ceil(Math.sqrt(width * width + height * height));
        int newHeight = newWidth;  
        BufferedImage rotatedImage = new BufferedImage(newWidth, newHeight, BufferedImage.TYPE_INT_RGB);

        Graphics2D g2d = rotatedImage.createGraphics();

         
        g2d.setBackground(java.awt.Color.BLACK);
        g2d.clearRect(0, 0, newWidth, newHeight);

         
        AffineTransform transform = new AffineTransform();
        transform.translate(newWidth / 2.0, newHeight / 2.0);  
        transform.rotate(angleRadians);  
        transform.translate(-width / 2.0, -height / 2.0);  
        g2d.setTransform(transform);

        g2d.drawImage(image, 0, 0, null);
        g2d.dispose();

        int[][][] rotatedPixels = new int[newHeight][newWidth][3];
        raster = rotatedImage.getRaster();
        for (int y = 0; y < newHeight; y++) {
            for (int x = 0; x < newWidth; x++) {
                int[] rgb = new int[3];
                raster.getPixel(x, y, rgb);
                rotatedPixels[y][x] = rgb;
            }
        }
        return rotatedPixels;
    }
}
