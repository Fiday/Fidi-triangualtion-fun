package org.delaunay.algorithm;

import java.awt.*;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.List;

import org.delaunay.model.*;

import com.google.common.base.Function;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import org.delaunay.model.Vector;

import javax.imageio.ImageIO;

/**
 * A fast Delaunay Triangulation implementation.
 */
@SuppressWarnings("serial")
public strictfp class Triangulation {
    private static final double EPS = 0.001;
    private static final double EPS_SQUARE = EPS * EPS;
    private int width;
    private int height;

    BufferedImage image;

    public Triangulation() {
    }

    public Triangulation(String filename, int width, int height) throws IOException {
        this.width = width;
        this.height = height;
        File file = new File(filename);
        image = ImageIO.read(file);

    }

    public static class NonDelaunayException extends RuntimeException {
    }

    public static class InvalidVertexException extends Exception {
    }

    public static interface DebugLogger {
        public void debug(String str);
    }

    public static enum VertexExceptionStrategy {
        THROW_EXCEPTION,
        CATCH_AND_DROP_VERTEX,
        ;
    }

    private Vertex[] superVerts = new Vertex[]{};
    private LinkedHashSet<Triangle> triangles = Sets.newLinkedHashSet();
    private LinkedHashSet<Vertex> inputVertices = Sets.newLinkedHashSet();
    private LinkedHashSet<Vertex> vertices = Sets.newLinkedHashSet();
    private Triangle lastLocatedTriangle = null;
    private int hopCount = 0;
    private int locateCount = 0;
    private boolean keepSuperTriangle = false;
    private VertexExceptionStrategy vertexExceptionStrategy = VertexExceptionStrategy.THROW_EXCEPTION;

    /*
     * The hilbert order determines the granularity of the hilbert curve. For
     * example, a value of 16 produces a square with with length and width 2^16,
     * resulting in 2^16 * 2^16 = 2^32 cells. This is typically good enough for
     * a triangulation up to 4 Billion vertices. Running time for coordinate
     * conversion would be O(16).
     */
    private int hilbertOrder = 16;

    /*
     * Determines the scale of the super triangle. Increase this number if you
     * need to vertex locates from farther out from the bounding box of the
     * vertices.
     */
    private double superTriangleScale = 2.0;

    private DebugLogger log = new DebugLogger() {
        public void debug(String str) {
            // null implementation
        }
    };

    public int getHopCount() {
        return hopCount;
    }

    public int getLocateCount() {
        return locateCount;
    }

    public void setDebugLogger(DebugLogger log) {
        this.log = log;
    }

    public Vertex addVertex(double x, double y) {
        Vertex vertex = new Vertex(x, y);
        inputVertices.add(vertex);
        return vertex;
    }

    public void addVertex(Vertex v) {
        inputVertices.add(v);
    }

    public void addAllVertices(Iterable<Vertex> vs) {
        Iterables.addAll(inputVertices, vs);
    }

    public LinkedHashSet<Vertex> getInputVertices() {
        return inputVertices;
    }

    public LinkedHashSet<Vertex> getVertices() {
        return vertices;
    }

    public List<Vertex> getVerticesInBounds(final Rectangle2D rect) {
        return Lists.newArrayList(Iterables.filter(getVertices(), new Predicate<Vertex>() {
            public boolean apply(Vertex v) {
                return rect.contains(v.toPoint());
            }
        }));
    }

    public LinkedHashSet<Triangle> getTriangles() {
        return triangles;
    }

    /**
     * If set to true, the supertriangle will not be removed at the end of the
     * triangulation method. This allows points outside the convex hull of
     * vertices to be located.
     */
    public void setKeepSuperTriangle(boolean keepSuperTriangle) {
        this.keepSuperTriangle = keepSuperTriangle;
    }

    public Vertex locateNearestVertex(Vector v) {
        Triangle located = locate(v);
        if (located == null) {
            return null;
        }

        Vertex bestVertex = null;
        double dist = Double.MAX_VALUE;

        for (Triangle tri : getCircumcircleTriangles(v, located)) {
            for (Vertex vert : tri.getVertices()) {
                double d = vert.subtract(v).lengthSquared();
                if (d < dist) {
                    bestVertex = vert;
                    dist = d;
                }
            }
        }
        return bestVertex;
    }

    public Set<Vertex> getVerticesInRadius(Vertex v, double radius) {
        Set<Vertex> checked = Sets.newHashSet(v);
        Set<Vertex> inRadius = Sets.newHashSet(v);
        Set<Vertex> toCheck = Sets.newHashSet(v.getNeighborVertices());

        while (toCheck.size() > 0) {
            Vertex check = Iterables.getFirst(toCheck, null);
            toCheck.remove(check);
            checked.add(check);

            if (v.subtract(check).length() < radius) {
                inRadius.add(check);
                toCheck.addAll(check.getNeighborVertices());
                toCheck.removeAll(checked);
            }
        }

        return inRadius;
    }

    public void triangulate() throws InvalidVertexException {
        /*
         * Reset triangulation state
         */
        resetTriangulation();

        if (Iterables.isEmpty(inputVertices)) {
            return;
        }

        /*
         * Determine the supertriangle.
         */
        createSuperTriangle(inputVertices);

        /*
         * Sort vertices using hilbert curve to linearize triangulation
         * performance.
         */
        log.debug("Linearizing with Hilbert Space-filling Curve");
        List<Vertex> sortedVertices = getHilbertSortedVertices(inputVertices);

        /*
         * Add vertices one at a time, updating the triangulation as we go.
         */
        log.debug("Building Triangulation");
        for (Vertex vertex : sortedVertices) {
            try {
                addVertexToTriangulation(vertex);
            } catch (InvalidVertexException e) {
                if (vertexExceptionStrategy == VertexExceptionStrategy.THROW_EXCEPTION) {
                    throw e;
                } else {
                    // ignore
                }
            }
        }

        /*
         * Cleanup
         */
        clearLocator();
        if (!keepSuperTriangle) {
            removeSuperTriangle();
        }

        log.debug("Triangulation Complete");
    }

    public void clear() {
        resetTriangulation();
        inputVertices = Sets.newLinkedHashSet();
    }

    private void resetTriangulation() {
        triangles = Sets.newLinkedHashSet();
        vertices = Sets.newLinkedHashSet();
        clearLocator();
    }

    private List<Vertex> getHilbertSortedVertices(Iterable<? extends Vertex> verts) {
        Rectangle2D bbox = Vectors.boundingBox(Lists.newArrayList(superVerts));
        ScaledHilbertIndex hilbert = new ScaledHilbertIndex(hilbertOrder, bbox);
        for (Vertex v : verts) {
            v.setHilbertIndex(hilbert.toIndex(v.x, v.y));
        }
        List<Vertex> sortedVertices = Lists.newArrayList(verts);
        Collections.sort(sortedVertices, new Comparator<Vertex>() {
            public int compare(Vertex v1, Vertex v2) {
                return v1.getHilbertIndex().compareTo(v2.getHilbertIndex());
            }
        });
        return sortedVertices;
    }

    public void addVertexToTriangulation(Vertex vertex) throws InvalidVertexException {
        Collection<Triangle> toRemove = null, toAdd = null;

        try {
            /*
             * Get the set of triangles for which the vertex lies in its
             * circumcircle.
             */
            toRemove = getCircumcircleTriangles(vertex);
        } catch (NonDelaunayException e) {
            /*
             * Unfortunately, we cannot recover from this state since the
             * triangulation is already non-delaunay. It was probably caused
             * by overlapping vertices, so we throw an invalid vertex
             * exception.
             */
            throw new InvalidVertexException();
        } catch (InvalidVertexException e) {
            log.debug(String.format("Dropping vertex %s because it outside the triangulation!\nMaybe something went wrong when computing the super triangle?", vertex));
            return;
        }

        /*
         * Compute the set of edges that represent the convex hull of the
         * cavity left by removing the triangles.
         */
        List<Edge> edgeSet = getEdgeSet(toRemove);

        /*
         * Remove the triangles.
         */
        removeTriangles(toRemove);

        try {
            /*
             * Create and add triangles created from the cavity convex hull
             * edges and the vertex.
             */
            toAdd = createTriangles(edgeSet, vertex);
            addTriangles(toAdd);
            vertices.add(vertex);
        } catch (NonDelaunayException e) {
            log.debug(String.format("Dropping vertex %s because it causes degeneracy.\nYou may need to use exact math on this vertex.", vertex));
            removeTriangles(toAdd);
            addTriangles(toRemove);
        }
    }

    public void createSuperTriangle(Iterable<? extends Vertex> verts) {
        createSuperTriangle(Vectors.boundingBox(verts));
    }

    public void createSuperTriangle(Rectangle2D rect) {
        double dmax = Math.max(rect.getWidth(), rect.getHeight());
        double xmid = rect.getCenterX();
        double ymid = rect.getCenterY();

        superVerts = new Vertex[]{
                new Vertex(xmid - dmax * superTriangleScale, ymid - dmax),
                new Vertex(xmid, ymid + dmax * superTriangleScale),
                new Vertex(xmid + dmax * superTriangleScale, ymid - dmax)
        };

        triangles = Sets.newLinkedHashSet();
        triangles.add(new Triangle(superVerts[0], superVerts[1], superVerts[2]));
    }

    public void removeSuperTriangle() {
        Set<Triangle> touching = Sets.newHashSet();
        for (Vertex v : superVerts) {
            touching.addAll(v.getNeighborTriangles());
        }
        removeTriangles(touching);
        superVerts = new Vertex[]{};
    }

    public boolean touchesSuperVertex(Triangle tri) {
        for (Vertex v : superVerts) {
            if (tri.getVertices().contains(v)) {
                return true;
            }
        }
        return false;
    }

    public boolean neighborsSuperVertex(Vertex vert) {
        for (Triangle tri : vert.getNeighborTriangles()) {
            if (touchesSuperVertex(tri)) {
                return true;
            }
        }
        if (Sets.newHashSet(superVerts).contains(vert)) {
            return true;
        }
        return false;
    }

    private void clearLocator() {
        lastLocatedTriangle = null;
    }

    public Collection<Triangle> getCircumcircleTriangles(Vector vertex) throws InvalidVertexException, NonDelaunayException {
        Triangle t = locate(vertex);
        if (t == null) {
            throw new InvalidVertexException();
        }
        return getCircumcircleTriangles(vertex, t);
    }

    private Collection<Triangle> getCircumcircleTriangles(Vector vertex, Triangle t) {
        Set<Triangle> checked = Sets.newHashSet(t);
        Set<Triangle> inCircum = Sets.newHashSet(t);

        // Initialize "to check" set with neighbors
        Set<Triangle> toCheck = Sets.newHashSet(t.getNeighbors());

        // For first triangle in "to check" set, check if
        // the vertex is in its circum circle.
        while (toCheck.size() > 0) {
            t = Iterables.getFirst(toCheck, null);
            toCheck.remove(t);

            if (t.isInCircum(vertex)) {
                inCircum.add(t);
                // If it is, add *its* neighbors to the "to check" set.
                toCheck.addAll(t.getNeighbors());
                toCheck.removeAll(checked);
            }
            checked.add(t);
        }

        return inCircum;
    }

    /*
     * Walks the triangulation toward the vertex. Returns the triangle in which
     * the vertex resides. If the vertex is outside the current triangulation,
     * nil is returned.
     *
     * It is possible that if the triangulation breaks due to floating point
     * errors, it will cause errors during locate. In this case, we throw a
     * NonDelaunayException.
     *
     * If the vertices are near each other, such as when iterating over a
     * hilbert linearization or running a scanline of locations, this should be
     * pretty fast.
     */
    public Triangle locate(Vector v) throws NonDelaunayException {
        locateCount += 1;
        Triangle t = lastLocatedTriangle == null ? Iterables.getFirst(triangles, null) : lastLocatedTriangle;
        if (t == null) {
            return null;
        }
        boolean done = false;

        Set<Triangle> seen = Sets.<Triangle>newHashSet();
        while (!done) {
            hopCount += 1;
            lastLocatedTriangle = t;
            if (seen.contains(t)) {
                throw new NonDelaunayException();
            }
            seen.add(t);
            Triangle tNext = t.nextWalk(v);
            if (tNext == null) {
                return null;
            }
            done = (tNext == t);
            t = tNext;
        }

        /*
         * During triangulation the located triangle is immediately removed.
         * But, it can be useful to store this if we are locating points in the
         * triangulation after it's constructed.
         */
        lastLocatedTriangle = t;
        return t;
    }

    public List<Edge> getEdgeSet(Collection<Triangle> tris) {
        List<Edge> edges = Lists.newArrayList();
        for (Triangle t : tris) {
            for (Edge e : t.getEdges()) {
                if (edges.contains(e)) {
                    edges.remove(e);
                } else {
                    edges.add(e);
                }
            }
        }
        return edges;
    }

    public List<Triangle> createTriangles(Iterable<Edge> edgeSet, final Vertex vertex) {
        return Lists.newArrayList(Iterables.transform(edgeSet, new Function<Edge, Triangle>() {
            public Triangle apply(Edge e) {
                return new Triangle(vertex, e.a, e.b);
            }
        }));
    }

    private void addTriangles(Iterable<Triangle> tris) throws NonDelaunayException {
        for (Triangle t : tris) {
            for (Vertex v : t.getVertices()) {
                v.addTriangle(t);
            }
            triangles.add(t);
        }
        lastLocatedTriangle = Iterables.getFirst(tris, null);
    }

    private void removeTriangles(Iterable<Triangle> tris) {
        for (Triangle t : tris) {
            for (Vertex v : t.getVertices()) {
                v.removeTriangle(t);
            }
            triangles.remove(t);
        }
        lastLocatedTriangle = null;
    }

    public HashSet<HashSet<Vector>> getPixelsOfTriangles() {
        HashSet<HashSet<Vector>> trianglePixels = new HashSet<>();

        for (Triangle triangle :
                triangles) {
            trianglePixels.add(getPixelsOfTriangle(triangle));
        }
        return trianglePixels;
    }

    public HashSet<Vector> getPixelsOfTriangle(Triangle triangle) {
        HashSet<Vector> subTriangle = new HashSet<>();

        Pair<Vector, Vector> boundingBox = getBoundingBox(triangle);

        for (int i = -3; i < getWidthBoundingBox(boundingBox) + 3; i++) {
            for (int j = -3; j < getHeightBoundingBox(boundingBox) + 3; j++) {
                if (within(triangle, new Vector((int) boundingBox.getKey().x + i, (int) boundingBox.getKey().y + j))) {
                    subTriangle.add(new Vector(boundingBox.getKey().x + i, boundingBox.getKey().y + j));
                }
            }
        }
        return subTriangle;
    }

    private boolean checkIfPointsAreOnTheSameSideOfLine(Edge edge, Vector p1, Vector p2) {
        return ((edge.a.y - edge.b.y) * (p1.x - edge.a.x) + (edge.b.x - edge.a.x) * (p1.y - edge.a.y)) *
                ((edge.a.y - edge.b.y) * (p2.x - edge.a.x) + (edge.b.x - edge.a.x) * (p2.y - edge.a.y)) > 0;
    }

    private boolean isOnInside(Triangle triangle, int x, int y) {

        boolean result = true;
        for (Edge edge : triangle.getEdges()) {
            result = result && checkIfPointsAreOnTheSameSideOfLine(edge, new Vector(x, y), edge.subtractVertices(triangle.getVertices()));
        }
        return result;
    }

    public Pair<Vector, Vector> getBoundingBox(Triangle triangle) {

        return new Pair<>(new Vector(Math.min(triangle.a.x, Math.min(triangle.b.x, triangle.c.x)), Math.min(triangle.a.y, Math.min(triangle.b.y, triangle.c.y)))
                , new Vector(Math.max(triangle.a.x, Math.max(triangle.b.x, triangle.c.x)),
                Math.max(triangle.a.y, Math.max(triangle.b.y, triangle.c.y))));
    }

    public double getWidthBoundingBox(Pair<Vector, Vector> boundingBox) {
        return Math.abs(boundingBox.getKey().x - boundingBox.getValue().x);
    }

    public double getHeightBoundingBox(Pair<Vector, Vector> boundingBox) {
        return Math.abs(boundingBox.getKey().y - boundingBox.getValue().y);
    }

    private Vector centerOfTriangle(Triangle triangle) {
        return new Vector((triangle.a.x + triangle.b.x + triangle.c.x) / 3, (triangle.a.y + triangle.b.y + triangle.c.y) / 3);
    }

    private boolean pointInTriangleBoundingBox(Triangle triangle, Vector v) {
        var xMin = Math.min(triangle.a.x, Math.min(triangle.b.x, triangle.c.x)) - EPS;
        var xMax = Math.max(triangle.a.x, Math.max(triangle.b.x, triangle.c.x)) + EPS;
        var yMin = Math.min(triangle.a.y, Math.min(triangle.b.y, triangle.c.y)) - EPS;
        var yMax = Math.max(triangle.a.y, Math.max(triangle.b.y, triangle.c.y)) + EPS;
        return !(v.x < xMin || xMax < v.x || v.y < yMin || yMax < v.y);
    }


    private static double side(Vector v1, Vector v2, Vector v) {
        return (v2.y - v1.y) * (v.x - v1.x) + (-v2.x + v1.x) * (v.y - v1.y);
    }

    private boolean nativePointInTriangle(Triangle triangle, Vector v) {
        boolean checkSide1 = side(triangle.a, triangle.b, v) >= 0;
        boolean checkSide2 = side(triangle.b, triangle.c, v) >= 0;
        boolean checkSide3 = side(triangle.c, triangle.a, v) >= 0;
        return checkSide1 && checkSide2 && checkSide3;
    }

    private double distanceSquarePointToSegment(Vector v1, Vector v2, Vector v) {
        double p1_p2_squareLength = (v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y);
        double dotProduct = ((v.x - v1.x) * (v2.x - v1.x) + (v.y - v1.y) * (v2.y - v1.y)) / p1_p2_squareLength;
        if (dotProduct < 0) {
            return (v.x - v1.x) * (v.x - v1.x) + (v.y - v1.y) * (v.y - v1.y);
        }
        if (dotProduct <= 1) {
            double p_p1_squareLength = (v1.x - v.x) * (v1.x - v.x) + (v1.y - v.y) * (v1.y - v.y);
            return p_p1_squareLength - dotProduct * dotProduct * p1_p2_squareLength;
        }
        return (v.x - v2.x) * (v.x - v2.x) + (v.y - v2.y) * (v.y - v2.y);
    }

    private boolean accuratePointInTriangle(Triangle triangle, Vector v) {
        if (!pointInTriangleBoundingBox(triangle, v)) {
            return false;
        }
        if (nativePointInTriangle(triangle, v)) {
            return true;
        }
        if (distanceSquarePointToSegment(triangle.a, triangle.b, v) <= EPS_SQUARE) {
            return true;
        }
        if (distanceSquarePointToSegment(triangle.b, triangle.c, v) <= EPS_SQUARE) {
            return true;
        }
        return distanceSquarePointToSegment(triangle.c, triangle.a, v) <= EPS_SQUARE;
    }

    public boolean within(Triangle triangle, Vector p) {
        Objects.requireNonNull(p);
        return accuratePointInTriangle(triangle, p);
    }

    public Color getAvgPixelColor(HashSet<Vector> pixels) {
        long sumr = 0, sumg = 0, sumb = 0;
        for (Vector vector : pixels
        ) {
            Color pixel = new Color(image.getRGB((int) vector.x, (int) vector.y));
            sumr += pixel.getRed();
            sumg += pixel.getGreen();
            sumb += pixel.getBlue();

        }
        float sizeOfHashSet = pixels.size();
        float red = sumr / sizeOfHashSet;
        float green = sumg / sizeOfHashSet;
        float blue = sumb / sizeOfHashSet;

        return new Color(red / 255, green / 255, blue / 255);
    }

    public static double calculateRelativePosition(double s, int maxRange) {
        return -1.0 + ((s - 0.0) * (1.0 - (-1.0))) / (maxRange - 0.0);
    }


    public List<String> getVertexArrayIbo() {
        List<String> returnHashset = new ArrayList<>();
        StringBuilder vertices = new StringBuilder("GLfloat vertices[] = {\n");
        StringBuilder colors = new StringBuilder("GLfloat colors[] = {\n");
        StringBuilder indices = new StringBuilder("GLuint indices[] = {");
        HashMap<Vertex, Integer> usedVertices = new HashMap<>();

        Integer j;
        Integer counter = 0;
        for (Triangle triangle :
                triangles) {

            j = usedVertices.putIfAbsent(triangle.a, counter);

            if (j == null) {
                vertices.append(calculateRelativePosition(triangle.a.x, width))
                        .append("f,")
                        .append(calculateRelativePosition(triangle.a.y, height) * -1)
                        .append("f, 0.0f")
                        .append(",")
                        .append("\n");
                indices.append(counter);
                counter++;
            } else {
                indices.append(j);
            }
            indices.append(",");

            j = usedVertices.putIfAbsent(triangle.b, counter);

            if (j == null) {
                vertices.append(calculateRelativePosition(triangle.b.x, width))
                        .append("f,")
                        .append(calculateRelativePosition(triangle.b.y, height) * -1)
                        .append("f, 0.0f")
                        .append(",")
                        .append("\n");
                indices.append(counter);
                counter++;
            } else {
                indices.append(j);
            }
            indices.append(",");

            j = usedVertices.putIfAbsent(triangle.c, counter);

            if (j == null) {
                vertices.append(calculateRelativePosition(triangle.c.x, width))
                        .append("f,")
                        .append(calculateRelativePosition(triangle.c.y, height) * -1)
                        .append("f, 0.0f")
                        .append(",")
                        .append("\n");
                indices.append(counter);
                counter++;
            } else {
                indices.append(j);
            }
            indices.append(",");


            Color color = getAvgPixelColor(getPixelsOfTriangle(triangle));
            colors.append(color.getRed() / 255.0).append("f,").append(color.getGreen() / 255.0).append("f,").append(color.getBlue() / 255.0).append("f, 1.0f,").append("\n");
            colors.append(color.getRed() / 255.0).append("f,").append(color.getGreen() / 255.0).append("f,").append(color.getBlue() / 255.0).append("f, 1.0f,").append("\n");
            colors.append(color.getRed() / 255.0).append("f,").append(color.getGreen() / 255.0).append("f,").append(color.getBlue() / 255.0).append("f, 1.0f,").append("\n");

            //vertices.append("\n");


        }
        colors.append("};\n");
        vertices.append("};\n");
        indices.append("};\n");


        returnHashset.add("#define NUM_VERTICES " + usedVertices.size());
        returnHashset.add("#define NUM_INDICES " + triangles.size() * 3);
        returnHashset.add("const unsigned int WIDTH = " + width + ";");
        returnHashset.add("const unsigned int HEIGHT = " + height + ";");
        returnHashset.add(vertices.toString());
        returnHashset.add(colors.toString());
        returnHashset.add(indices.toString());


        return returnHashset;
    }

    public List<String> getVertexArrayNoIbo() {

        List<String> returnHashset = new ArrayList<>();
        StringBuilder vertices = new StringBuilder("GLfloat vertices[] = {\n");
        StringBuilder colors = new StringBuilder("GLfloat colors[] = {\n");

        for (Triangle triangle :
                triangles) {
            vertices.append(calculateRelativePosition(triangle.b.x, width)).append("f,").append(calculateRelativePosition(triangle.b.y, height) * -1).append("f, 0.0f,").append("\n");
            vertices.append(calculateRelativePosition(triangle.a.x, width)).append("f,").append(calculateRelativePosition(triangle.a.y, height) * -1).append("f, 0.0f,").append("\n");
            vertices.append(calculateRelativePosition(triangle.c.x, width)).append("f,").append(calculateRelativePosition(triangle.c.y, height) * -1).append("f, 0.0f,").append("\n");
            Color color = getAvgPixelColor(getPixelsOfTriangle(triangle));

            colors.append(color.getRed() / 255.0).append("f,").append(color.getGreen() / 255.0).append("f,").append(color.getBlue() / 255.0).append("f, 1.0f,").append("\n");
            colors.append(color.getRed() / 255.0).append("f,").append(color.getGreen() / 255.0).append("f,").append(color.getBlue() / 255.0).append("f, 1.0f,").append("\n");
            colors.append(color.getRed() / 255.0).append("f,").append(color.getGreen() / 255.0).append("f,").append(color.getBlue() / 255.0).append("f, 1.0f,").append("\n");

        }
        vertices.append("};\n");
        colors.append("};\n");


        returnHashset.add("#define NUM_VERTICES " + triangles.size() * 3);
        returnHashset.add("const unsigned int WIDTH = " + width + ";");
        returnHashset.add("const unsigned int HEIGHT = " + height + ";\n");
        returnHashset.add(vertices.toString());
        returnHashset.add(colors.toString());


        return returnHashset;
    }


}



