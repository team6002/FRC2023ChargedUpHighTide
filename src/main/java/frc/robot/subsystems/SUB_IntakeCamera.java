package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class SUB_IntakeCamera extends SubsystemBase {
    private Thread camThread;
    private OpenCVThread cvThread;

    public SUB_IntakeCamera() {
        cvThread = new OpenCVThread();
        camThread = new Thread(cvThread);
    }

    public void init() {
        /* Start separate thread */
        camThread.setDaemon(true);
        camThread.start();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    /* THREAD FOR RUNNING CAMERASERVER AND OPENCV PROCESSING */
    class OpenCVThread implements Runnable {
        private int camWidth = 320, camHeight = 240;
        private int CB_CHAN_IDX = 1;

        public void run() {
            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setResolution(camWidth, camHeight);

            CvSink cvSink = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("Rectangle", camWidth, camHeight);

            Mat mat = new Mat();
            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
            Mat org;

            System.out.println("AAA: SETUP CAMERA COMPLETE");

            while (!Thread.interrupted()) {
                try {
                    if (cvSink.grabFrame(mat) == 0) {
                        /* Error grabbing frame, ignore */
                        System.out.println("AAA: ERROR GRABBING FRAMES");
                        continue;
                    }

                    org = mat.clone();
    
                    // Imgproc.GaussianBlur(mat, mat, new Size(11, 11), 0);
                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2YCrCb);
                    Core.extractChannel(mat, mat, CB_CHAN_IDX);
                    Imgproc.threshold(mat, mat, 175, 255, Imgproc.THRESH_BINARY);

                    Imgproc.dilate(mat, mat, dilateElement);
                    Imgproc.dilate(mat, mat, dilateElement);
                    Imgproc.dilate(mat, mat, dilateElement);
                    Imgproc.dilate(mat, mat, dilateElement);
                    Imgproc.dilate(mat, mat, dilateElement);
                    Imgproc.dilate(mat, mat, dilateElement);
    
                    List<MatOfPoint> contours = new ArrayList<>();
                    Imgproc.findContours(mat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
    
                    Scalar color = new Scalar(0, 255, 0);
    
                    for (int i = 0; i < contours.size(); i++) {
                        Imgproc.drawContours(org, contours, i, color, 1);
                        // RotatedRect r = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
                    }
    
                    outputStream.putFrame(org);

                    /* Sleep for 10ms to avoid overruning the RoboRio */
                    Thread.sleep(10);
                } catch (Exception e) {
                    System.out.println("OpenCVThread Exception: " + e);
                }
            }
        }
    }
}
