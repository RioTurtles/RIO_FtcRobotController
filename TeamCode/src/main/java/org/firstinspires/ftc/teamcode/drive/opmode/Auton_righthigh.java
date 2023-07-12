
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Project1Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="auton_righthigh")
@Config

public class Auton_righthigh extends LinearOpMode {
    public static int horztest = 1500;
    public static double armtest = 0.26;
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
        int intakeHorz[] = {1150, 1300, 1300, 1350, 1450, 0};
        int loops = 0;
        double toggleTimer = 0, toggleOrig1 = 0, toggleInterval = 50;
        double toggleTimer2 = 0;
        double intakeArm[] = {0.36, 0.36, 0.30, 0.26, 0.18, 0.18};
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
                .addTemporalMarker(2,() -> {
                    robot.clawOpen();
                    robot.setHorz(intakeHorz[1]);
                })
                .build();
        drive.setPoseEstimate(untitled0.start());

        Trajectory forward = drive.trajectoryBuilder(new Pose2d(-33.00, 62.63, Math.toRadians(270.00)))
                .forward(10)
                .build();
        robot.setArm(0.83);
        robot.clawOpen();
        robot.clawAngle.setPosition(0.205);
        robot.yGuide.setPosition(0.2);
        robot.bucketAngle.setPosition(0.68);

        waitForStart();
        robot.lVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.followTrajectorySequence(untitled0);
        robot.setArm(intakeArm[1]);

        while (opModeIsActive()) {
            toggleTimer += 1;
            toggleTimer2 += 1;
            bucketcheck = (((robot.bucketColor.red() > 90) || (robot.bucketColor.blue() > 90)) && ((robot.bucketColor.green() < robot.bucketColor.blue()) || (robot.bucketColor.green() < robot.bucketColor.red())));
            Pose2d myPose = drive.getPoseEstimate();
            drive.update();
            Trajectory stay = drive.trajectoryBuilder(myPose)
                    .lineToLinearHeading(new Pose2d(-35, 3, Math.toRadians(347)))
                    .build();

            if ((stage == 0) && (loaded)) {
                robot.setVert(vertTarget);
                if (robot.lVert.getCurrentPosition() > (vertTarget - 500)) {
                    robot.yGuide.setPosition(0.35);
                    robot.bucketAngle.setPosition(0);
                    sleep(500);
                    robot.bucketAngle.setPosition(0.65);
                    sleep(200);
                    scoredCones +=1;
                    robot.setVert(0);
                    robot.yGuide.setPosition(0.2);
                    robot.setHorz(intakeHorz[scoredCones-1]-100);
                    robot.setArm(intakeArm[scoredCones-1]);
                    loaded = false;
                    toggleTimer = 0;
                    stage +=1;
                }
            }

            if (stage == 1) {//transfer
                if (scoredCones<6) {

                        robot.clawClose();
                        sleep(200);
                        robot.setArm(0.7);
                        sleep(400);
                        robot.setHorz(0);
                        if (robot.lHorz.getCurrentPosition() < 200) {
                            robot.setArm(0.85);
                            robot.clawAngle.setPosition(0.65);

                            stage += 1;
                            toggleTimer = 0;
                            timer2.reset();

                        }

                }
                else {
                    telemetry.addLine("done");
                }
            }
            if (stage ==2 ){
                robot.clawAngle.setPosition(0.85);
                    if (timer2.milliseconds() > 350) {
                        robot.clawOpen();
                        stage +=1 ;
                        loaded = true;
                        toggleTimer = 0;
                        timer2.reset();
                    }
            }

            if (stage == 3){
                if (timer2.milliseconds()>450) {
                    robot.clawAngle.setPosition(0.205);
                    if(scoredCones<5){
                        robot.setHorz(intakeHorz[scoredCones]-200);
                        robot.setArm(intakeArm[scoredCones]);
                    }
                    stage = 0;
                    loaded = true;
                    toggleTimer = 0;
                    timer2.reset();
                }
            }
           if (scoredCones<6){
               drive.followTrajectory(stay);
           }
            TrajectorySequence midpoint1 = drive.trajectorySequenceBuilder(myPose)
                    .turn(Math.toRadians(90))
                    .splineTo(new Vector2d(-60.34, 12.57), Math.toRadians(270.00))
                    .build();
            Trajectory midpoint2 = drive.trajectoryBuilder(myPose)
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                    .build();
            TrajectorySequence midpoint3 = drive.trajectorySequenceBuilder(myPose)
                    .turn(Math.toRadians(90))
                    .splineTo(new Vector2d(-23.77, 13.71), Math.toRadians(53.13))
                    .splineTo(new Vector2d(-11.43, 12.34), Math.toRadians(270.00))
                    .build();

            if (scoredCones==6){
                robot.setArm(0.05);
                robot.clawAngle.setPosition(0.6);
               drive.followTrajectorySequence(midpoint1);
           }
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
