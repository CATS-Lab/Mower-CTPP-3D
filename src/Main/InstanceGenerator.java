package Main;

import Common.Data;
import Common.Input;
import Common.Solution;
import Algorithms.MILP;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Random;

public class InstanceGenerator {

    public static void main(String[] args) throws IOException {
        int width = 150;
        int height = 150;
        double highest = 1.0;
        double obstacleProbability = 0.32;

        for (int i = 0; i < 5; i++) {
            InstanceGenerator mapGenerator = new InstanceGenerator();
            mapGenerator.generateMaps(width, height, obstacleProbability, highest);
            String data_path = "./data/" + width + "_" + height + "_" + obstacleProbability + "_" + highest + "_" + i + ".txt";
            mapGenerator.saveToFile(data_path);
            System.out.println("generate: " + data_path);
        }
    }


    public int width;
    public int height;
    public double obstacleProbability;
    public int[][] binaryMap;
    public double[][] heightMap;

    public void generateMaps(int width, int height, double obstacleProbability, double highest) {
        this.width = width;
        this.height = height;
        this.obstacleProbability = obstacleProbability;
        generateBinaryMap();
        generateHeightMap(highest);
        roundHeightMap();
    }

    public void generateBinaryMap() {
        Random rand = new Random();
        binaryMap = new int[height][width];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                binaryMap[y][x] = rand.nextDouble() < obstacleProbability ? 1 : 0;
            }
        }

        for (int i = 0; i < 4; i++) {
            smoothBinaryMap();
        }
    }

    public void generateHeightMap(double highest) {
        Random rand = new Random();
        heightMap = new double[height][width];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (binaryMap[y][x] == 1) {
                    heightMap[y][x] = highest;
                } else {
                    heightMap[y][x] = rand.nextDouble() * highest;
                }
            }
        }

        smoothHeightMap();
    }

    public void smoothBinaryMap() {
        int[][] newMap = new int[height][width];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int neighborWallTiles = getSurroundingWallCount(x, y);

                if (neighborWallTiles > 4)
                    newMap[y][x] = 1;
                else if (neighborWallTiles < 4)
                    newMap[y][x] = 0;
                else
                    newMap[y][x] = binaryMap[y][x];
            }
        }

        binaryMap = newMap;
    }

    public int getSurroundingWallCount(int gridX, int gridY) {
        int wallCount = 0;
        for (int neighborX = gridX - 1; neighborX <= gridX + 1; neighborX++) {
            for (int neighborY = gridY - 1; neighborY <= gridY + 1; neighborY++) {
                if (neighborX >= 0 && neighborX < width && neighborY >= 0 && neighborY < height) {
                    if (neighborX != gridX || neighborY != gridY) {
                        wallCount += binaryMap[neighborY][neighborX];
                    }
                } else {
                    wallCount++;
                }
            }
        }
        return wallCount;
    }

    public void smoothHeightMap() {
        double[][] newHeightMap = new double[height][width];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                double totalHeight = 0.0;
                int count = 0;

                for (int offsetY = -1; offsetY <= 1; offsetY++) {
                    for (int offsetX = -1; offsetX <= 1; offsetX++) {
                        int newY = y + offsetY;
                        int newX = x + offsetX;

                        if (newY >= 0 && newY < height && newX >= 0 && newX < width) {
                            totalHeight += heightMap[newY][newX];
                            count++;
                        }
                    }
                }

                newHeightMap[y][x] = totalHeight / count;
            }
        }

        heightMap = newHeightMap;
    }

    private void roundHeightMap() {
        DecimalFormat df = new DecimalFormat("#.0");
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                heightMap[y][x] = Double.parseDouble(df.format(heightMap[y][x]));
            }
        }
    }

    public void printBinaryMap() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                System.out.print(binaryMap[y][x] + " ");
            }
            System.out.println();
        }
    }

    public void printHeightMap() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                System.out.printf("%.2f ", heightMap[y][x]);
            }
            System.out.println();
        }
    }

    public void saveToFile(String filename) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename))) {
            writer.write(height + "\n");
            writer.write(width + "\n");
            writer.write("\n");

            boolean start = false;

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (binaryMap[y][x] == 0 && !start){
                        binaryMap[y][x] = 2;
                        start = true;
                    }
                    writer.write(binaryMap[y][x] + " ");
                }
                writer.write("\n");
            }

            writer.write("\n");

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    writer.write(heightMap[y][x] + " ");
                }
                writer.write("\n");
            }
        }
    }
}

