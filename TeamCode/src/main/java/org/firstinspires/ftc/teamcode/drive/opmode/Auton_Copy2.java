
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

@Autonomous (name="auton_Copy2")
@Config

public class Auton_Copy2 extends LinearOpMode {

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
        double intakeArm[] = {0.38, 0.32, 0.26, 0.18, 0.05, 0.7};
        timer.reset();
        timer2.reset();
        //boolean bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 0.05) || (robot.bucketColor.getNormalizedColors().blue>0.05));
        boolean clawcheck = ((robot.clawColor.red() > 200) || (robot.clawColor.blue() > 200));
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-33.00, -62.63, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.00, -18.00))
                .splineToSplineHeading(new Pose2d(-34, -0.00, Math.toRadians(17.00)), Math.toRadians(50.00))
                .build();
        drive.setPoseEstimate(untitled0.start());
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(-33.00, -62.63, Math.toRadians(90.00)))
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
                    .lineToLinearHeading(new Pose2d(-33, 1, Math.toRadians(17)))
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
                    //robot.setHorz(intakeHorz[scoredCones-1]-100);
                    //robot.setArm(intakeArm[scoredCones-1]);
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
                    if (timer2.milliseconds() > 500) {
                        robot.clawOpen();
                        stage +=1 ;
                        loaded = true;
                        toggleTimer = 0;
                        timer2.reset();
                    }
            }

            if (stage == 3){
                if (timer2.milliseconds()>300) {
                    robot.clawAngle.setPosition(0.3);
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
