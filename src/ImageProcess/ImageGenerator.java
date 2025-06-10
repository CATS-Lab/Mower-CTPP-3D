package ImageProcess;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;

public class ImageGenerator {
    public void ImageFromPixels(int[][][] pixels, String path) {
        int width = pixels.length;
        int height = pixels[0].length;

        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        WritableRaster raster = image.getRaster();

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int[] rgb = new int[3];
                rgb[0] = pixels[x][y][0];     // Red
                rgb[1] = pixels[x][y][1]; // Green
                rgb[2] = pixels[x][y][2]; // Blue
                raster.setPixel(x, y, rgb);
            }
        }

        try {
            File output = new File(path);
            ImageIO.write(image, "png", output);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
