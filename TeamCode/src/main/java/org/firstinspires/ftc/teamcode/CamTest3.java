package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Config

@Autonomous
public class CamTest3 extends OpMode {
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

        Mat aCrop;
        Mat bCrop;
        Mat cCrop;
        Mat dCrop;
        Mat eCrop;
        Mat fCrop;
        Mat gCrop;
        Mat hCrop;
        Mat iCrop;
        Mat jCrop;
        Mat kCrop;
        Mat lCrop;
        Mat mCrop;
        Mat nCrop;
        Mat oCrop;
        Mat pCrop;
        Mat qCrop;
        Mat rCrop;
        Mat sCrop;
        Mat tCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        double pivot;
        double aavgfin;
        double bavgfin;
        double cavgfin;
        double davgfin;
        double eavgfin;
        double favgfin;
        double gavgfin;
        double havgfin;
        double iavgfin;
        double javgfin;
        double kavgfin;
        double lavgfin;
        double mavgfin;
        double navgfin;
        double oavgfin;
        double pavgfin;
        double qavgfin;
        double ravgfin;
        double savgfin;
        double tavgfin;

        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        Scalar midrectColor = new Scalar(255.0,255.0,255.0);


        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,219,359);
            Rect rightRect = new Rect(419,1,219,359);
            Rect midRect = new Rect (220,1,199,359);
            Rect aRect = new Rect(640/20,1,640/20,359);
            Rect bRect = new Rect((640/20)*2,1,640/20,359);
            Rect cRect = new Rect((640/20)*3,1,640/20,359);
            Rect dRect = new Rect((640/20)*4,1,640/20,359);
            Rect eRect = new Rect((640/20)*5,1,640/20,359);
            Rect fRect = new Rect((640/20)*6,1,640/20,359);
            Rect gRect = new Rect((640/20)*7,1,640/20,359);
            Rect hRect = new Rect((640/20)*8,1,640/20,359);
            Rect iRect = new Rect((640/20)*9,1,640/20,359);
            Rect jRect = new Rect((640/20)*10,1,640/20,359);
            Rect kRect = new Rect((640/20)*11,1,640/20,359);
            Rect lRect = new Rect((640/20)*12,1,640/20,359);
            Rect mRect = new Rect((640/20)*13,1,640/20,359);
            Rect nRect = new Rect((640/20)*14,1,640/20,359);
            Rect oRect = new Rect((640/20)*15,1,640/20,359);
            Rect pRect = new Rect((640/20)*16,1,640/20,359);
            Rect qRect = new Rect((640/20)*17,1,640/20,359);
            Rect rRect = new Rect((640/20)*18,1,640/20,359);
            Rect sRect = new Rect((640/20)*19,1,640/20,359);
            Rect tRect = new Rect((640/20)*19,1,640/20,359);

            input.copyTo(output);
            Imgproc.rectangle(output,leftRect,rectColor,2);
            Imgproc.rectangle(output, rightRect,rectColor,2);
            Imgproc.rectangle(output, midRect,rectColor,2);
            Imgproc.rectangle(output,aRect,rectColor,2);
            Imgproc.rectangle(output,bRect,rectColor,2);
            Imgproc.rectangle(output,cRect,rectColor,2);
            Imgproc.rectangle(output,dRect,rectColor,2);
            Imgproc.rectangle(output,eRect,rectColor,2);
            Imgproc.rectangle(output,fRect,rectColor,2);
            Imgproc.rectangle(output,gRect,rectColor,2);
            Imgproc.rectangle(output,hRect,rectColor,2);
            Imgproc.rectangle(output,iRect,rectColor,2);
            Imgproc.rectangle(output,jRect,rectColor,2);
            Imgproc.rectangle(output,kRect,rectColor,2);
            Imgproc.rectangle(output,lRect,rectColor,2);
            Imgproc.rectangle(output,mRect,rectColor,2);
            Imgproc.rectangle(output,nRect,rectColor,2);
            Imgproc.rectangle(output,oRect,rectColor,2);
            Imgproc.rectangle(output,pRect,rectColor,2);
            Imgproc.rectangle(output,qRect,rectColor,2);
            Imgproc.rectangle(output,rRect,rectColor,2);
            Imgproc.rectangle(output,sRect,rectColor,2);
            Imgproc.rectangle(output,tRect,rectColor,2);


            leftCrop =  YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);
            aCrop =  YCbCr.submat(aRect);
            bCrop =  YCbCr.submat(bRect);
            cCrop =  YCbCr.submat(cRect);
            dCrop =  YCbCr.submat(dRect);
            eCrop =  YCbCr.submat(eRect);
            fCrop =  YCbCr.submat(fRect);
            gCrop =  YCbCr.submat(gRect);
            hCrop =  YCbCr.submat(hRect);
            iCrop =  YCbCr.submat(iRect);
            jCrop =  YCbCr.submat(jRect);
            kCrop =  YCbCr.submat(kRect);
            lCrop =  YCbCr.submat(lRect);
            mCrop =  YCbCr.submat(mRect);
            nCrop =  YCbCr.submat(nRect);
            oCrop =  YCbCr.submat(oRect);
            pCrop =  YCbCr.submat(pRect);
            qCrop =  YCbCr.submat(qRect);
            rCrop =  YCbCr.submat(rRect);
            sCrop =  YCbCr.submat(sRect);
            tCrop =  YCbCr.submat(tRect);


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

