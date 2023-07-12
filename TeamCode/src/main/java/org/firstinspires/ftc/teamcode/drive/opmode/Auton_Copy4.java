
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Project1Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="auton_right")
@Config

public class Auton_Copy4 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        timer.reset();
        double cur = timer.milliseconds();
        double cur2 = timer2.milliseconds();
        boolean bucketcheck = (((robot.bucketColor.red() > 90) || (robot.bucketColor.blue() > 90)) && ((robot.bucketColor.green() < robot.bucketColor.blue()) || (robot.bucketColor.green() < robot.bucketColor.red())));
        boolean bucketCheck = ((robot.bucketColor.red() > 100) || (robot.bucketColor.blue() > 100));
        boolean loaded = true;
        boolean realse = false;
        boolean raise = false;
        boolean ground = false;
        boolean farming = false;
        boolean farmingReady = true;
        boolean scoring = false;
        int vertTarget = 1500;// high pole
        int horzTarget = 0;
        int stage = 0;
        int horzwait = 0;
        int autonStage = 0;
        int scoredCones = 0;
        int intakeHorz[] = {1600, 1600, 1700, 1800, 2000, 0};
        int loops = 0;
        double toggleTimer = 0, toggleOrig1 = 0, toggleInterval = 50;
        double toggleTimer2 = 0;
        double intakeArm[] = {0.24, 0.22, 0.18, 0.16, 0.12, 0.10};
        timer.reset();
        timer2.reset();
        //boolean bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 0.05) || (robot.bucketColor.getNormalizedColors().blue>0.05));
        boolean clawcheck = ((robot.clawColor.red() > 200) || (robot.clawColor.blue() > 200));
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-32.00, 62.63, Math.toRadians(270.00)))
                .lineTo(new Vector2d(-36.00, 18.00))
                .splineToSplineHeading(new Pose2d(-34, 3.00, Math.toRadians(345)), Math.toRadians(310.00))
                .addTemporalMarker(1,() -> {
            robot.setVert(vertTarget);
        })
                .build();
        drive.setPoseEstimate(untitled0.start());

        Trajectory forward = drive.trajectoryBuilder(new Pose2d(-33.00, 62.63, Math.toRadians(270.00)))
                .forward(10)
                .build();
        robot.setArm(0.83);
        robot.clawOpen();
        robot.clawAngle.setPosition(0.3);
        robot.yGuide.setPosition(0.2);
        robot.bucketAngle.setPosition(0.68);

        waitForStart();
        robot.lVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.followTrajectorySequence(untitled0);
        robot.setHorz(1000);

        while (opModeIsActive()) {
            toggleTimer += 1;
            toggleTimer2 += 1;
            bucketcheck = (((robot.bucketColor.red() > 90) || (robot.bucketColor.blue() > 90)) && ((robot.bucketColor.green() < robot.bucketColor.blue()) || (robot.bucketColor.green() < robot.bucketColor.red())));
            Pose2d myPose = drive.getPoseEstimate();
            drive.update();
            Trajectory stay = drive.trajectoryBuilder(myPose)
                    .lineToLinearHeading(new Pose2d(-35, 3, Math.toRadians(340)))
                    .build();

            if (autonStage ==-1){ //moving right

                scoring = false;

                if (timer.milliseconds()>1600){
                    autonStage += 1;

                }
            }



            if (autonStage ==0){ //moving right

                scoring = false;

                if (timer.milliseconds()>2400){
                    autonStage += 1;

                }
            }


            if (autonStage ==1){ //turning
                scoring = false;

                if (timer.milliseconds()>3000) {
                    autonStage += 1;
                    loaded = true;
                }
            }
            if (autonStage == 2){
                if (stage==0) {
                    autonStage += 1;
                }
                robot.clawAngle.setPosition(0.3);
                robot.setHorz(1100);
                robot.setArm(intakeArm[1]);
                scoring = true;
            }
            if (autonStage == 3){
                if ((scoredCones == 6) || (cur >27000)){
                    autonStage += 1;

                }
            }
            if (autonStage == 4){
                if (robot.lHorz.getCurrentPosition()<50) {
                    autonStage += 1;
                    loaded = false;
                }
                robot.clawAngle.setPosition(0.6);
                robot.setHorz(0);
                robot.setVert(0);
                stage=0;

                scoring=false;
                loaded=false;
            }
            if (autonStage == 5){
                robot.setVert(0);
                robot.setHorz(0);// Car stop
            }
            //***************************************************************
            //TRANSFER

            if (stage == 0 && scoring) { //grab
                robot.setArm(intakeArm[scoredCones]);


                //robot.clawOpen();
                sleep(1000);
                horzTarget=intakeHorz[scoredCones]-100;
                robot.setHorz(horzTarget);
                if ((robot.lHorz.getCurrentPosition() > horzTarget-300)||clawcheck) {
                    robot.lHorz.setPower(0);
                    robot.rHorz.setPower(0);
                    robot.clawClose();
                    sleep(400);
                    robot.setArm(0.7);
                    sleep(400);//half lifted
                    robot.setHorz(0);
                    toggleTimer=0;
                    timer2.reset();
                    stage += 1;
                }


            }
            if (stage==1) { //retract
                if (robot.lHorz.getCurrentPosition()<150) {
                    robot.setArm(0.85);
                    robot.clawAngle.setPosition(0.65);

                    scoredCones+=1;
                    stage += 1;
                    toggleTimer=0;
                    timer2.reset();

                }
            }
            if (stage == 2) { //open claw

                //if (toggleTimer > 3){
                robot.clawAngle.setPosition(0.85);
                // }

                if (timer2.milliseconds()>300){
                    robot.clawOpen();
                    stage +=1 ;
                    toggleTimer = 0;
                    timer2.reset();
                }
            }

            if (stage == 3) { //release claw


                if (timer2.milliseconds()>300) {
                    robot.clawAngle.setPosition(0.3);
                    if (scoredCones <5){
                        robot.setHorz(1300);

                    }
                    robot.setArm(intakeArm[scoredCones]);

                    //if (bucketcheck){

                    loaded  = true;

                    //}else{
                    // stage = 0;
                    //loaded = false;
                    //}

                }
            }
            if (loaded){ //score and extend
                robot.setVert(vertTarget);
                //robot.yGuide.setPosition(0.35);





                if (robot.lVert.getCurrentPosition()>(vertTarget-500)){
                    robot.yGuide.setPosition(0.35);


                    robot.bucketAngle.setPosition(0);

                    sleep(500);
                    robot.bucketAngle.setPosition(0.65);
                    sleep(200);
                    robot.setVert(0);
                    robot.yGuide.setPosition(0.2);
                    loaded=false;
                    toggleTimer=0;
                    stage = 0;


                    //farmingReady = true;



                }


            }
            drive.followTrajectory(stay);
            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());

            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addLine("Staying");
            telemetry.addData("stage", stage);
            telemetry.addData("loaded", loaded);
            telemetry.update();
        }

    }
}
