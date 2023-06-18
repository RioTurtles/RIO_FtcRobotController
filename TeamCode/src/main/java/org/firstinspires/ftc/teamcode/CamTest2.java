package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

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
public class CamTest2 extends OpMode {
    OpenCvCamera webcam1 = null;
    FtcDashboard dashboard;

    public static double Pivot;
    public static double presentvalue = 100;
    int maxpos = 0;
    double maxval=0;



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





       // double pivot;

        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        Scalar midrectColor = new Scalar(255.0,255.0,255.0);


        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

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






            Core.extractChannel(aCrop, aCrop,2);
            Core.extractChannel(bCrop, bCrop,2);
            Core.extractChannel(cCrop, cCrop,2);
            Core.extractChannel(dCrop, dCrop,2);
            Core.extractChannel(eCrop, eCrop,2);
            Core.extractChannel(fCrop, fCrop,2);
            Core.extractChannel(gCrop, gCrop,2);
            Core.extractChannel(hCrop, hCrop,2);
            Core.extractChannel(iCrop, iCrop,2);
            Core.extractChannel(jCrop, jCrop,2);
            Core.extractChannel(kCrop, kCrop,2);
            Core.extractChannel(lCrop, lCrop,2);
            Core.extractChannel(mCrop, mCrop,2);
            Core.extractChannel(nCrop, nCrop,2);
            Core.extractChannel(oCrop, oCrop,2);
            Core.extractChannel(pCrop, pCrop,2);
            Core.extractChannel(qCrop, qCrop,2);
            Core.extractChannel(rCrop, rCrop,2);
            Core.extractChannel(sCrop, sCrop,2);
            Core.extractChannel(tCrop, tCrop,2);




            Scalar aavg = Core.mean(aCrop);
            Scalar bavg = Core.mean(bCrop);
            Scalar cavg = Core.mean(cCrop);
            Scalar davg = Core.mean(dCrop);
            Scalar eavg = Core.mean(eCrop);
            Scalar favg = Core.mean(fCrop);
            Scalar gavg = Core.mean(gCrop);
            Scalar havg = Core.mean(hCrop);
            Scalar iavg = Core.mean(iCrop);
            Scalar javg = Core.mean(jCrop);
            Scalar kavg = Core.mean(kCrop);
            Scalar lavg = Core.mean(lCrop);
            Scalar mavg = Core.mean(mCrop);
            Scalar navg = Core.mean(nCrop);
            Scalar oavg = Core.mean(oCrop);
            Scalar pavg = Core.mean(pCrop);
            Scalar qavg = Core.mean(qCrop);
            Scalar ravg = Core.mean(rCrop);
            Scalar savg = Core.mean(sCrop);
            Scalar tavg = Core.mean(tCrop);




            aavgfin = aavg.val[0];
            bavgfin = bavg.val[0];
            cavgfin = cavg.val[0];
            davgfin = davg.val[0];
            eavgfin = eavg.val[0];
            favgfin = favg.val[0];
            gavgfin = gavg.val[0];
            havgfin = havg.val[0];
            iavgfin = iavg.val[0];
            javgfin = javg.val[0];
            kavgfin = kavg.val[0];
            lavgfin = lavg.val[0];
            mavgfin = mavg.val[0];
            navgfin = navg.val[0];
            oavgfin = oavg.val[0];
            pavgfin = pavg.val[0];
            qavgfin = qavg.val[0];
            ravgfin = ravg.val[0];
            savgfin = savg.val[0];
            tavgfin = tavg.val[0];


            double[] pos = {aavgfin,bavgfin,cavgfin,davgfin,eavgfin,favgfin,gavgfin,havgfin,iavgfin,javgfin,kavgfin,lavgfin,mavgfin,navgfin,oavgfin,pavgfin,qavgfin,ravgfin,savgfin,tavgfin};

            maxval=120.0;
            for (int i = 0; i < 19; i++) {

                if (pos[i] < maxval){
                    maxval = pos[i];
                    maxpos = i;
                }
                telemetry.addData("Pos",pos[i]);
            }











            telemetry.addData("maxpos",maxpos);
            telemetry.addData("maxval",maxval);

            dashboard.startCameraStream(webcam1,100);



            telemetry.update();


            return(output);



        }
    }
}

