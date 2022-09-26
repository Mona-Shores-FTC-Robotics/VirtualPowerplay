package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVWebcam {
    //public varibles
    public OpenCvCamera webcam;
    public int TeamEleLoc;
    public int FinalTeamEleLoc;
    public HardwareMap hwMap = null;
    public Mat outPut = new Mat();
    public double LeftMax;
    public double MiddleMax;
    public double RightMax;
    //public double AdjustedThreshold;
    //public static final double UnadjustedThreshold = 210;

    //private variables

    private final int RectLX = 5;
    private final int RectLY = 150;
    private final int RectMX = 113;
    private final int RectMY = 150;
    private final int RectRX = 230;
    private final int RectRY = 150;

    private final int RectLwidth = 85;
    private final int RectLheight = 85;
    private final int RectMwidth = 40;
    private final int RectMheight = 85;
    private final int RectRwidth = 80;
    private final int RectRheight = 85;

    private final  Rect rectL = new Rect(RectLX, RectLY, RectLwidth, RectLheight);
    private final Rect rectM = new Rect(RectMX, RectMY, RectMwidth, RectMheight);
    private final Rect rectR = new Rect(RectRX, RectRY, RectRwidth, RectRheight);
    private final Scalar rectanglecolor = new Scalar(0, 0, 0);

    private Mat LeftCrop = new Mat();
    private Mat MiddleCrop = new Mat();
    private Mat RightCrop = new Mat();

    //private Core.MinMaxLocResult GreenMinMax;
    //private Mat InputGreenChannel = new Mat();
    //private Mat QROutput = new Mat();
    private Core.MinMaxLocResult LeftMinMax;
    private Core.MinMaxLocResult MiddleMinMax;
    private Core.MinMaxLocResult RightMinMax;

    /* Constructor */
    public OpenCVWebcam() {

    }
    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //webcam.setPipeline(new SamplePipeline());

        webcam.setPipeline(new SamplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {

            input.copyTo(outPut);


            //Draw Three Rectangles
            //Imgproc.rectangle(outPut, rectL, rectanglecolor, 2);
            Imgproc.rectangle(outPut, rectM, rectanglecolor, 2);
            //Imgproc.rectangle(outPut, rectR, rectanglecolor, 2);
            //LeftCrop = input.submat(rectL);
            MiddleCrop = input.submat(rectM);
            //RightCrop = input.submat(rectR);

            // channel 1 = green

            //Core.extractChannel(LeftCrop, LeftCrop, 1);



            Core.extractChannel(MiddleCrop, MiddleCrop, 1);
            //Core.extractChannel(RightCrop, RightCrop, 1);

            //LeftMinMax = Core.minMaxLoc(LeftCrop);



            MiddleMinMax = Core.minMaxLoc(MiddleCrop);
            //RightMinMax = Core.minMaxLoc(RightCrop);

            //LeftMax = LeftMinMax.maxVal;
            MiddleMax = MiddleMinMax.maxVal;
            //RightMax = RightMinMax.maxVal;

            //Core.extractChannel(input, InputGreenChannel, 1);
            //GreenMinMax = Core.minMaxLoc(InputGreenChannel);
            //AdjustedThreshold = UnadjustedThreshold * (GreenMinMax.maxVal / 255.0);

            //QR Code detection
            //QRCodeDetector QR = new QRCodeDetector();
            //QR.detect(input, QROutput);

            //points for drawing two vertical lines to split the viewing area into thirds
            //Point LeftLineTop = new Point(input.size().width / 3,  0 );
          //  Point LeftLineBottom = new Point (input.size().width / 3, input.size().height);
          //  Point RightLineTop = new Point(2 * (input.size().width / 3),  0 );
        //    Point RightLineBottom = new Point (2 * (input.size().width / 3), input.size().height);

            //draws the two vertical lines on the viewing area
          //  Imgproc.line(outPut, LeftLineTop, LeftLineBottom, new Scalar(0,0,0));
            //Imgproc.line(outPut, RightLineTop, RightLineBottom, new Scalar (0,0,0));

            //draw lines to form rectangle around QR code

                //for (int i = 0; i < QROutput.cols(); i++) {
                  //  Point pt1 = new Point(QROutput.get(0, i));
                   // Point pt2 = new Point(QROutput.get(0, (i + 1) % 4));
                   // Imgproc.line(outPut, pt1, pt2, new Scalar(255, 0, 0), 3);
               // }
            //}
            //top left corner of QR code can be used for TeamEleLoc update if this works
            //double[] TopLeftQRCodePoint = QROutput.get(0, 0);

//            if (TopLeftQRCodePoint[0] < input.size().width / 3){
//                TeamEleLoc = 0;
//            }
//            else if (TopLeftQRCodePoint[0] < 2 * (input.size().width / 3)){
//                TeamEleLoc = 1;
//            }
//            else {
//                TeamEleLoc = 2;
//            }
/*            if (LeftMax < AdjustedThreshold && RightMax < AdjustedThreshold) {
                //team element = left
                TeamEleLoc = 0;
            }

            if (LeftMax > AdjustedThreshold){
                //team element = center
                TeamEleLoc = 1;
            }
            if (RightMax > AdjustedThreshold){
                //team element = right
                TeamEleLoc = 2;
            }
*/

            if (LeftMax > MiddleMax && LeftMax > RightMax){
            // team element = left
            TeamEleLoc = 0;
             }

            if (MiddleMax > LeftMax && MiddleMax > RightMax){
                // team element = Middle
                TeamEleLoc = 1;
            }

            if (RightMax > MiddleMax && RightMax > LeftMax){
                // team element = Right
                TeamEleLoc = 2;
            }

            return outPut;
            // write crop code for third square
            // write two square method code for comparing two of the squares
            // white three square method for determinng team element locationat highest value of three squares

        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
