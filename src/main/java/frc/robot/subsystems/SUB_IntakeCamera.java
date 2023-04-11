package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.Scalar;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ArrayList;

public class SUB_IntakeCamera extends Thread {
    private int camWidth = 320, camHeight = 240;
    private double ts;

    @Override
    public void run() {
        ts = Timer.getFPGATimestamp();
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(camWidth, camHeight);
        
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Rectangle", camWidth, camHeight);

        Mat mat = new Mat();

        System.out.println("AAA: SETUP CAMERA COMPLETE");

        while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
                /* Error grabbing frame, ignore */
                System.out.println("AAA: ERROR GRABBING FRAMES");
                continue;
            }

            Mat org = mat.clone();

            Imgproc.GaussianBlur(mat, mat, new Size(11, 11), 0);
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
            Core.inRange(mat, new Scalar(100, 0, 0), new Scalar(130,255,255), mat);

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(mat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Scalar color = new Scalar(0, 255, 0);

            if (Timer.getFPGATimestamp() - ts > 0.5) {
                System.out.println("AAA: contour_size = " + contours.size());
                ts = Timer.getFPGATimestamp();
            }

            for (int i = 0; i < contours.size(); i++) {
                RotatedRect r = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
                Size s = r.size;
                double ratio = s.height / s.width;

                if (ratio >= 0.6 && ratio <= 1.4) {
                    Imgproc.drawContours(org, contours, i, color, 1);
                }
            }

            outputStream.putFrame(org);
        }
    }
}
