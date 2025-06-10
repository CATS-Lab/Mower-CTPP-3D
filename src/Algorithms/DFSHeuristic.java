package Algorithms;

import Common.Data;
import Common.Path;
import Common.Point;
import Common.Solution;

import java.util.ArrayList;
import java.util.List;

public class DFSHeuristic {
    public boolean[][] binary_map; // 0 = obstacle, -1 = free
    public double[][] height_map;
    private Data data;
    private int rows, cols;
    private boolean[][] visited;
    private ArrayList<Point> path = new ArrayList<>();
    private static final int TURN_COST = 100; // 可以根据需要调节转弯惩罚


    private static final int[] dx = {-1, 0, 1, 0};
    private static final int[] dy = {0, 1, 0, -1};

    public DFSHeuristic(Data data) {
        this.data =data;
        this.binary_map = data.binary_graph;
        this.height_map = data.height_graph;
        this.rows = binary_map.length;
        this.cols = binary_map[0].length;
        this.visited = new boolean[rows][cols];
    }

    public Solution run() {
        Point start = findStart();
        if (start != null) {
            dfs(start.x, start.y, -1);  // 初始无方向
        }

        Path _path = new Path();
        for (Point p: path)
            _path.addPoint(p);
        _path.getSpeed(data, data.getPixelMatrix());
        return new Solution(_path);
    }

    private Point findStart() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (binary_map[i][j]) {
                    return new Point(i, j); // x = row, y = col
                }
            }
        }
        return null; // no start found
    }

    private void dfs(int x, int y, int prevDir) {
        visited[x][y] = true;
        path.add(new Point(x, y));

        List<int[]> neighbors = new ArrayList<>();

        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (inBounds(nx, ny) && !visited[nx][ny] && isSlopeAcceptable(x, y, nx, ny) && binary_map[nx][ny]) {
                int turnPenalty = (prevDir == -1 || prevDir == d) ? 0 : TURN_COST;
                int heuristic = countFreeNeighbors(new Point(nx, ny)) - turnPenalty;
                neighbors.add(new int[]{nx, ny, d, heuristic});
            }
        }

        // 按照启发式排序（越大优先）
        neighbors.sort((a, b) -> Integer.compare(b[3], a[3]));

        for (int[] neighbor : neighbors) {
            int nx = neighbor[0];
            int ny = neighbor[1];
            int dir = neighbor[2];
            dfs(nx, ny, dir);
            path.add(new Point(x, y)); // 回退
        }
    }


    private boolean isSlopeAcceptable(int x1, int y1, int x2, int y2) {
        double h1 = height_map[x1][y1];
        double h2 = height_map[x2][y2];
        return data.is_feasible_slope(h1 - h2);
    }

    private boolean inBounds(int x, int y) {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    }

    private int countFreeNeighbors(Point p) {
        int count = 0;
//        for (int d = 0; d < 4; d++) {
//            int nx = p.x + dx[d];
//            int ny = p.y + dy[d];
//            if (inBounds(nx, ny) && !visited[nx][ny] && binary_map[nx][ny]) {
//                count++;
//            }
//        }

        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                if (dx == 0 && dy == 0) continue; // 忽略自己
                int nx = p.x + dx;
                int ny = p.y + dy;
                if (inBounds(nx, ny) && !visited[nx][ny] && binary_map[nx][ny]) {
                    count++;
                }
            }
        }
        return count;
    }
}
