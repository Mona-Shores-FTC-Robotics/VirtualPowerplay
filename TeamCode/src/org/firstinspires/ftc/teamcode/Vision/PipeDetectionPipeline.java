package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PipeDetectionPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public boolean poleCenter = false;
    public boolean poleLeft = false;
    public boolean poleRight = false;
    public int poleCenterCounter = 0;

    public PipeDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    //Outputs
    private final Mat hsvThresholdOutput = new Mat();
    private final Mat hsvThresholdInput = new Mat();

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, hsvThresholdInput, Imgproc.COLOR_RGB2HSV);

        double[] hsvThresholdHue = {16.7, 23};
        double[] hsvThresholdSaturation = {30, 255.0};
        double[] hsvThresholdValue = {0, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

           // Step Find_Contours0:
            Mat findContoursInput = hsvThresholdOutput;
            boolean findContoursExternalOnly = false;
            findContours(hsvThresholdOutput, findContoursExternalOnly, findContoursOutput);

            // Step Filter_Contours0:
            ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
            double filterContoursMinArea = 0;
            double filterContoursMinPerimeter = 0.0;
            double filterContoursMinWidth = 0;
            double filterContoursMaxWidth = 1000;
            double filterContoursMinHeight = 160;
            double filterContoursMaxHeight = 1000;
            double[] filterContoursSolidity = {0, 100};
            double filterContoursMaxVertices = 1000000;
            double filterContoursMinVertices = 0;
            double filterContoursMinRatio = 0;
            double filterContoursMaxRatio = 1000;
            filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

            Mat output = new Mat(input.rows(), input.cols(), CvType.CV_8UC4);

            //-1 fills in
            for (int i = 0; i < filterContoursOutput.size(); i++) {
                Imgproc.drawContours(output, filterContoursOutput, i, new Scalar(255, 255, 255), -1);
            }

            Rect leftRect = new Rect(0,0, 375, 448);
            Rect rectOfInterest = new Rect(375,0, 50, 448);
            Rect rightRect = new Rect(425,0, 375, 448);
        Imgproc.rectangle(output, rectOfInterest, new Scalar (255,255,255));
        Imgproc.rectangle(output, leftRect, new Scalar (255,0,0));
        Imgproc.rectangle(output, rightRect, new Scalar (0, 0,255));

            Mat zoneOfInterest = output.submat(rectOfInterest);
            Mat leftZone  = output.submat(leftRect);
            Mat rightZone  = output.submat(rightRect);

            double percentZoneOfInterest = (Core.sumElems(zoneOfInterest).val[1] / rectOfInterest.area() / 255) * 100 ;
            double percentLeftZone = (Core.sumElems(leftZone).val[1] / leftRect.area() / 255) * 100 ;
            double percentRightZone= (Core.sumElems(rightZone).val[1] / rightRect.area() / 255) * 100 ;

            //telemetry.addData("Percent of white pixels in Zone of Interest (center)", (int) percentZoneOfInterest );
            //telemetry.addData("Percent of white pixels in Left Zone", (int) percentLeftZone);
            //telemetry.addData("Percent of white pixels in Right Zone", (int) percentRightZone);

            if (percentZoneOfInterest > 50)
            {
                    poleCenter = true;
                    //telemetry.addLine("Junction Pole Centered for 10 frames");

            }
            else {
                poleCenter = false;
                poleCenterCounter = 0;
            }

            if (percentLeftZone > percentRightZone)
            {
                poleLeft = true;
                poleRight = false;
                //telemetry.addLine("Pole on Robot's left, turn right to center pole");
            }

            else if (percentRightZone >= percentLeftZone) {
                poleRight = true;
                poleLeft = false;
                //telemetry.addLine("Pole on Robot's right, turn left to center pole");
            }
        //telemetry.update();
        return output;
        }

        public boolean isPoleCenter()
        {
            return poleCenter;
        }

        public boolean isPoleLeft()
        {
            return poleLeft;
        }

        public boolean isPoleRight()
        {
            return poleRight;
        }

        public Mat hsvThresholdOutput() {
            return hsvThresholdOutput;
        }

        public ArrayList<MatOfPoint> findContoursOutput() {
            return findContoursOutput;
        }

        public ArrayList<MatOfPoint> filterContoursOutput() {
            return filterContoursOutput;
        }

        private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                                  Mat out) {
            Core.inRange(input, new Scalar(hue[0], sat[0], val[0]),
                    new Scalar(hue[1], sat[1], val[1]), out);
        }

        private void findContours(Mat input, boolean externalOnly,
                                  List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            }
            else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);


        }

        private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                    double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                            maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                            minRatio, double maxRatio, List<MatOfPoint> output) {
            final MatOfInt hull = new MatOfInt();
            output.clear();
            //operation
            for (int i = 0; i < inputContours.size(); i++) {
                final MatOfPoint contour = inputContours.get(i);
                final Rect bb = Imgproc.boundingRect(contour);
                if (bb.width < minWidth || bb.width > maxWidth) continue;
                if (bb.height < minHeight || bb.height > maxHeight) continue;
                final double area = Imgproc.contourArea(contour);
                if (area < minArea) continue;
                if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++) {
                    int index = (int)hull.get(j, 0)[0];
                    double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                final double solid = 100 * area / Imgproc.contourArea(mopHull);
                if (solid < solidity[0] || solid > solidity[1]) continue;
                if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
                final double ratio = bb.width / (double)bb.height;
                if (ratio < minRatio || ratio > maxRatio) continue;
                output.add(contour);
            }
        }

    }

