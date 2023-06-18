package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config

@Autonomous
public class CamTest1 extends OpMode {
    OpenCvCamera webcam1 = null;
    FtcDashboard dashboard;

    public static double Pivot;
    public static double presentvalue = 100;



    @Override
    public void init(){
        WebcamName webcamName =hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            public void onOpened(){
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){

            }
        });

    }
    @Override
    public void loop(){



    }
    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        double pivot;

        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        Scalar midrectColor = new Scalar(255.0,255.0,255.0);


        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,219,359);
            Rect rightRect = new Rect(419,1,219,359);
            Rect midRect = new Rect (220,1,199,359);

            input.copyTo(output);
            Imgproc.rectangle(output,leftRect,rectColor,2);
            Imgproc.rectangle(output, rightRect,rectColor,2);
            Imgproc.rectangle(output, midRect,rectColor,2);


            leftCrop =  YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);


            Core.extractChannel(leftCrop, leftCrop,2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin =rightavg.val[0];
            midavgfin = midavg.val[0];



                if ((midavgfin < leftavgfin) && (midavgfin < rightavgfin)) {
                    telemetry.addLine("Mid");
                } else if (leftavgfin > rightavgfin) {
                    telemetry.addLine("Right");
                } else {
                    telemetry.addLine("Left");
                }


            pivot = leftavgfin -rightavgfin;
            telemetry.addData("pivot",pivot);
            telemetry.addData("mid",midavgfin);
            telemetry.addData("right",rightavgfin);
            telemetry.addData("left", leftavgfin);
            dashboard.startCameraStream(webcam1,100);


            telemetry.update();


            return(output);



        }
    }
}

