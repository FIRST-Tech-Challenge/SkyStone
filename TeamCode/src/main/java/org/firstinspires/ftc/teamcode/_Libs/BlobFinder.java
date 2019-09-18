package org.firstinspires.ftc.teamcode._Libs;

// image processing utility class that finds the biggest contiguous blob of pixels of a given color in an image

import android.graphics.Bitmap;
import android.graphics.Point;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

public class BlobFinder {

    Bitmap image;
    boolean visited[][];
    int bbCount;            // pixel count of biggest connected blob found so far
    Point bbOrigin;         // origin of biggest connected blob found so far
    Point bMin, bMax;       // limits of connected blob at "current" pixel
    Point bbMin, bbMax;     // limits of biggest connected blob found so far
    int color;              // color we're looking for
    Queue<Point> pending;   // queue of pending pixel locations to check

    public BlobFinder(Bitmap bitmap) {
            image = bitmap;
            visited = new boolean[image.getHeight()][image.getWidth()];
            bbOrigin = new Point();
            bbMin = new Point();
            bbMax = new Point();
            bMin = new Point();
            bMax = new Point();
            pending = new ArrayBlockingQueue<Point>(image.getHeight()*image.getWidth());    // worst case ...
    }

    // return number of pixels in biggest blob of given color
    public int find(int c) {
        return find(c, 1);
    }

    public int find(int c, int sample) {

        // save color we're looking for so we don't have to pass it recursively
        color = c;

        // Initialize the visited flag array
        for (int x=0; x<image.getWidth(); x++)
        for (int y=0; y<image.getHeight(); y++)
            visited[y][x] = false;

        // Initialize biggest blob count and min/max values
        bbCount = 0;
        final int infinity = 100000;
        bbMin.set(infinity, infinity);
        bbMax.set(-infinity, -infinity);

        // Search for blob starting at each Nth pixel
        for (int x=0; x<image.getWidth(); x+=sample) {
            for (int y = 0; y < image.getHeight(); y+=sample) {
                // Deep travel that pixel
                bMin.set(infinity, infinity);
                bMax.set(-infinity, -infinity);
                int count = deepCount(x, y);

                // Check if it's the biggest so far ...
                if (count > bbCount) {
                    bbOrigin.x = x;
                    bbOrigin.y = y;
                    bbCount = count;
                    bbMin.set(bMin.x, bMin.y);
                    bbMax.set(bMax.x, bMax.y);
                }
            }
        }

        // Return the position and the size of the blob
        return bbCount;
    }

    // get x and y of blob centroid and limit points
    public Point getCentroid() {
        return new Point((bbMin.x+bbMax.x)/2, (bbMin.y+bbMax.y)/2);
    }
    public Point getBoundsMin() {
        return bbMin;
    }
    public Point getBoundsMax() {
        return bbMax;
    }

    // This is the deep search function - compute the number of connected pixels of
    // the search color starting at location x,y.
    // The natural way to do this would be recursive but we run out of stack space at about 300 pixels so
    // we'll do it the hard way using an array of Points to keep track of pixels to be searched.
    private int deepCount(int ox, int oy) {
        int count = 0;

        pending.add(new Point(ox,oy));

        while(!pending.isEmpty()) {

            // get the next element from the queue
            Point p = pending.remove();
            int x = p.x;
            int y = p.y;

            // Ignore the pixel if it's out of bounds
            if (p.x < 0 || p.x >= image.getWidth() || p.y < 0 || p.y >= image.getHeight())
                continue;

            // Ignore the pixel if it's already visited
            if (visited[y][x] == true)
                continue;

            // Mark the pixel as visited
            visited[y][x] = true;

            // Skip if the pixel is not the requested color
            if (image.getPixel(x,y) != color)
                continue;

            // Expand min/max to include this pixel
            if (x < bMin.x) bMin.x = x;
            if (x > bMax.x) bMax.x = x;
            if (y < bMin.y) bMin.y = y;
            if (y > bMax.y) bMax.y = y;

            // Increment count by 1
            count++;

            // Search for adjacent pixels (up, left, right, down)
            pending.add(new Point(x, y-1));
            pending.add(new Point(x-1, y));
            pending.add(new Point(x+1, y));
            pending.add(new Point(x, y+1));

        }

        // return the total count
        return count;
    }
}
