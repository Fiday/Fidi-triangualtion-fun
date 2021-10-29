package org.delaunay;

import org.delaunay.algorithm.Triangulation;
import org.delaunay.algorithm.Triangulation.InvalidVertexException;
import org.delaunay.algorithm.Triangulations;
import org.delaunay.dtfe.painters.PaintTransform;
import org.delaunay.dtfe.painters.TriangulationPainter;
import org.delaunay.dtfe.painters.TriangulationPainterModel;
import org.delaunay.model.GifFrame;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import static org.delaunay.model.ImageUtil.convertRGBAToGIF;
import static org.delaunay.model.ImageUtil.saveAnimatedGIF;

public class TriangulationDemo {

    public static void makeGIF(String filename, int delay) throws Exception {

        List<GifFrame> gifFrames = new ArrayList<>();
        OutputStream output = new FileOutputStream(filename + ".gif");

        String[] filenames = new File("outfiles").list();
        List<BufferedImage> images = new LinkedList<>();

        for (String name : filenames) {

            BufferedImage next = ImageIO.read(new File("outfiles/" + name));
            images.add(next);
        }

        for (BufferedImage image : images) {
            int transparantColor = 0xFF00FF; // purple
            BufferedImage gif = convertRGBAToGIF(image, transparantColor);

            String disposal = GifFrame.RESTORE_TO_BGCOLOR;

            gifFrames.add(new GifFrame(gif, delay, disposal));
        }

        int loopCount = 0; // loop indefinitely
        saveAnimatedGIF(output, gifFrames, loopCount);
    }


    public static void drawTriangulation(Triangulation t, int w, int h, String filename, BufferedImage image)
            throws IOException {

        TriangulationPainter painter = new TriangulationPainter(new TriangulationPainterModel()
                .setEdgeColor(new Color(0xFFFFFF))
                .setEdgeStrokeWidth(1.5f));


        BufferedImage img = painter.paint(t, new PaintTransform(w, h));
        Graphics2D g = (Graphics2D) img.getGraphics();

        g.setStroke(new BasicStroke(1.0f));
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        t.getPixelsOfTriangles().forEach(vectors -> {
            g.setColor(t.getAvgPixelColor(vectors));
            vectors.forEach(v -> g.drawLine((int) v.x, (int) v.y, (int) v.x, (int) v.y));
        });


        ImageIO.write(img, "png", new File("outfiles/" + filename));
    }

    public static Triangulation createTriangulation(Integer vertices, char number, String filename) throws InvalidVertexException, IOException {
        BufferedImage image;
        File file = new File(filename + ".png");
        image = ImageIO.read(file);

        Triangulation t = new Triangulation(filename + ".png", image.getWidth(), image.getHeight());


        t.addAllVertices(Triangulations.randomVertices(vertices, image.getWidth(), image.getHeight()));
        t.triangulate();

        System.out.println("Creating images");
        drawTriangulation(t, image.getWidth(), image.getHeight(), filename + number + ".png", image);

        System.out.println("Done");

        return t;
    }

    public static void deleteDirectory(File file) {
        for (File subfile : file.listFiles()) {
            if (subfile.isDirectory()) {
                deleteDirectory(subfile);
            }

            subfile.delete();
        }
    }

    public static void main(String[] args) throws Exception {
        deleteDirectory(new File("outfiles"));
        new File("outfiles").mkdirs();

        String filename = "FidiStuff";

        Triangulation t = createTriangulation(10000, 'A', filename);
        FileWriter myWriter = new FileWriter("result.txt");

        t.getVertexArrayNoIbo().forEach(v -> {
            try {
                myWriter.write(v + "\n");
            } catch (IOException e) {
                e.printStackTrace();
            }


        });
        myWriter.close();
/*
        IntFunction<Integer> incrementation = (a) -> a + a / 2;

        int maxVertices = 50000;
        int minVertices = 3;
        int gifTimeBetween = 300;


        int x = minVertices;
        for (int i = 0; x < maxVertices; i++) {
            createTriangulation(x, (char) (i + 65), filename);
            x = incrementation.apply(x);
        }
        makeGIF(filename, gifTimeBetween);*/
    }
}
