package org.firstinspires.ftc.teamcode.subSystems;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

public class CamSub implements AutoCloseable {

    private final VisionPortal portal;
    private final Map<ColorRange, ColorBlobLocatorProcessor> blobProcessors = new HashMap<>();
    private final Map<ColorRange, List<Point>> blobCenters = new HashMap<>();
    private final Set<ColorBlobLocatorProcessor> initializedBlobProcessors = new HashSet<>();
    private CameraCalibration calibration;

    public CamSub(HardwareMap hm, VisionProcessor... processors) {
        portal = new VisionPortal.Builder()
                .setCamera(hm.get(WebcamName.class, "cam"))
                .setCameraResolution(new Size(640, 480)) // 480p
                .enableLiveView(false)
                .addProcessor(new VisionProcessor() {
                    @Override
                    public void init(int width, int height, CameraCalibration cameraCalibration) {
                        synchronized (CamSub.this) {
                            calibration = cameraCalibration;
                            initializedBlobProcessors.clear();
                        }
                    }

                    @Override
                    public Object processFrame(Mat frame, long captureTimeNanos) {
                        synchronized (CamSub.this) {
                            for (Map.Entry<ColorRange, ColorBlobLocatorProcessor> entry : blobProcessors.entrySet()) {
                                ColorBlobLocatorProcessor processor = entry.getValue();
                                if (initializedBlobProcessors.add(processor)) {
                                    processor.init(frame.cols(), frame.rows(), calibration);
                                }
                                processor.processFrame(frame, captureTimeNanos);
                                List<Point> centers = new ArrayList<>();
                                for (ColorBlobLocatorProcessor.Blob blob : processor.getBlobs()) {
                                    Point center = blob.getBoxFit().center;
                                    centers.add(new Point(center.x, center.y));
                                }
                                blobCenters.put(entry.getKey(), centers);
                            }
                        }
                        return null;
                    }

                    @Override
                    public void onDrawFrame(Canvas canvas, int width, int height,
                                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                            Object userContext) {
                    }
                })
                .addProcessors(processors)
                .build();
    }

    public void startStreaming() {
        portal.resumeStreaming();
    }

    public void stopStreaming() {
        portal.stopStreaming();
    }

    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        portal.setProcessorEnabled(processor, enabled);
    }

    public VisionPortal.CameraState getCameraState() {
        return portal.getCameraState();
    }

    public double getFps() {
        return portal.getFps();
    }

    @Override
    public void close() {
        portal.close();
    }

    // returns x,y center points of blobs of the color given
    public synchronized List<Point> scanBlobs(ColorRange color) {
        Objects.requireNonNull(color, "color");
        ColorBlobLocatorProcessor processor = blobProcessors.get(color);
        if (processor == null) {
            processor = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(color)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(false)
                    .build();
            blobProcessors.put(color, processor);
        }
        List<Point> centers = new ArrayList<>();
        for (Point center : blobCenters.getOrDefault(color, Collections.emptyList())) {
            centers.add(new Point(center.x, center.y));
        }
        return centers;
    }
}
