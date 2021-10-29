package org.delaunay.model;


import java.util.Collection;

public class Edge {
    public final Vertex a, b;

    public Edge(Vertex a, Vertex b) {
        this.a = a;
        this.b = b;
    }

    public Vertex subtractVertices(Collection<Vertex> vertices) {
        vertices.remove(a);
        vertices.remove(b);

        return vertices.stream().findFirst().orElse(new Vertex(0, 0));
    }


    @Override
    public int hashCode() {
        return a.hashCode() ^ b.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Edge) {
            Edge o = (Edge) obj;
            return (a == o.a && b == o.b) || (a == o.b && b == o.a);
        }
        return false;
    }
}