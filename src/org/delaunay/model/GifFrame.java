package org.delaunay.model;

import org.w3c.dom.Node;

import javax.imageio.*;
import javax.imageio.metadata.IIOInvalidTreeException;
import javax.imageio.metadata.IIOMetadata;
import javax.imageio.metadata.IIOMetadataNode;
import javax.imageio.stream.ImageOutputStream;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.IndexColorModel;
import java.awt.image.WritableRaster;
import java.io.OutputStream;
import java.util.List;

public class GifFrame {
    public static final String NONE = "none";
    public static final String DO_NOT_DISPOSE = "doNotDispose";
    public static final String RESTORE_TO_BGCOLOR = "restoreToBackgroundColor";
    public static final String RESTORE_TO_PREVIOUS = "restoreToPrevious";

    public final BufferedImage img;
    public final long delay; // in millis
    public final String disposalMethod;

    public GifFrame(BufferedImage img, long delay) {
        this(img, delay, NONE);
    }

    public GifFrame(BufferedImage img, long delay, String disposalMethod) {
        this.img = img;
        this.delay = delay;
        this.disposalMethod = disposalMethod;
    }
}




