
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous (name="jerry2")

public class Auton2 extends LinearOpMode {
    final static double ARM_HOME = 0.0;
    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;
    final static double HEXMOTOR_TPR = 288;
    final static double GOBILDA_TPR = 751.8; //5202
    final static double GEAR_REDUCTION = 1;
    final static double TICKS_PER_DEGREE = (HEXMOTOR_TPR * GEAR_REDUCTION) / 360;
    //ticks = pulses, cycles = ticks * 4

    final String[] debugModes = {"VERT", "HORZ", "BUCKET", "BUCKET ANGLE", "CLAW", "CLAW ANGLE"};
    int dModesIndex = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.imu1.resetYaw();

        //variables
        double direction_y=0, direction_x=0, pivot=0, heading = 0;
        boolean clawOpen = true;
        double toggleTimer = 0, toggleOrig1 = 0, toggleInterval = 50;


        int vertPos;
        String mode = "OFF";

        robot.setArm(0.83);
        robot.clawOpen();
        robot.clawAngle.setPosition(0.3);
        robot.yGuide.setPosition(0.2);
        robot.bucketAngle.setPosition(0.68);


        waitForStart();
        drivetrain.remote(0, 0, 0, 0);
        //robot.flip1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //((DcMotorEx) robot.flip1).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        //robot.flip1.setDirection(DcMotorEx.Direction.REVERSE);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        timer.reset();
        double cur = timer.milliseconds();
        double cur2 = timer2.milliseconds();
        double targetDirection = 0;
        double yTarget = 0;
        double xTarget = 0;
        double scoringDirection = -0.3;
        double parkingDistance = 10;
        //double backOrgin = robot.backDis.getDistance(DistanceUnit.CM);
        double intakeArm[] = {0.34,0.32,0.25,0.18,0.05,0};





        boolean bucketCheck = ((robot.bucketColor.getNormalizedColors().red > 100) || (robot.bucketColor.getNormalizedColors().blue > 100));
        boolean loaded = false;
        boolean realse = false;
        boolean raise = false;
        boolean ground = false;
        boolean farming = false;
        boolean farmingReady = true;
        boolean scoring = false;
        boolean bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 0.05) || (robot.bucketColor.getNormalizedColors().blue>0.05));

        int vertTarget = 2400;// high pole
        int horzTarget = 0;
        int stage = -1;
        int horzwait = 0;
        int autonStage = 0;
        int scoredCones = 0;
        int intakeHorz[] = {2000,1900,2100,2200,2400,0};

        timer.reset();




        while (opModeIsActive()) {
            heading = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            pivot = targetDirection - heading;
            drivetrain.remote(direction_y, direction_x, pivot, heading);
            bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 0.05) || (robot.bucketColor.getNormalizedColors().blue>0.05));
            toggleTimer +=1;

            if (autonStage ==0){ //moving right
                direction_y = (83-robot.backDis.getDistance(DistanceUnit.CM))*0.1;//put distance code here
                direction_x = 0;
                targetDirection = 0;

                scoring = false;

                if (robot.rightDis.getDistance(DistanceUnit.CM) >20){
                    autonStage += 1;

                }
            }
            if (autonStage ==1){ //moving right
                direction_y = 0;//put distance code here
                direction_x = 1;
                targetDirection = 0;

                scoring = false;

                if ((robot.leftDis.getDistance(DistanceUnit.CM) < 800) && ( robot.leftDis.getDistance(DistanceUnit.CM) >100)){
                    autonStage += 1;

                }
            }
            if (autonStage ==2){
                direction_y = 0;
                direction_x = 0.5;
                scoring = false;
                targetDirection = 0;//

                if (robot.fRightDis.getDistance(DistanceUnit.CM) <20) {
                    autonStage += 1;
                }
            }
            if (autonStage ==3){ //turning
                direction_y = 0;
                direction_x = 0;
                scoring = false;
                targetDirection = scoringDirection;//

                if (heading > scoringDirection*0.95) {
                    autonStage += 1;
                    loaded = true;
                }
            }
            if (autonStage == 4){
                if (stage==0){
                    autonStage+=1;
                }
                scoring = true;
                direction_y = 0;
                direction_x = 0;

                targetDirection = scoringDirection;


            }
            if (autonStage == 5){
                if ((scoredCones == 6) || (cur >200000000)){
                    autonStage += 1;
                }
                direction_y = 0;
                direction_x = 0;
                targetDirection = scoringDirection;
            }
            if (autonStage == 6){
                if (robot.rightDis.getDistance(DistanceUnit.CM) < 120){
                    autonStage += 1;
                }
                direction_y = 0;//put distance code here
                direction_x = -1;
                targetDirection = 0;
                scoring=false;
                loaded=false;
            }
            if (autonStage == 7){ // Car stop

                direction_y = 0;//put distance code here
                direction_x = 0;
                targetDirection = 0;
            }
























            /*if (robot.lVert.getCurrentPosition() > 800) {  //Brakes/slow down car
                direction_x = direction_x / 2;
                direction_y = direction_y / 2;
                pivot = pivot / 2;
            } // brake*///brake




            //***************************************************************
            //TRANSFER

            if (stage == 0 && scoring) {
                robot.setArm(intakeArm[scoredCones]);

                //robot.clawOpen();
                horzTarget=intakeHorz[scoredCones];
                robot.setHorz(horzTarget);
                if (robot.lHorz.getCurrentPosition() > horzTarget-200) {
                    drivetrain.remote(0, 0, 0, 0);
                    robot.clawClose();
                    sleep(600);
                    robot.setArm(0.7);
                    sleep(300);//half lifted
                    robot.setHorz(0);
                    stage += 1;
                }

            }
            if (stage==1) {
                if (robot.rHorz.getCurrentPosition() < 5) {
                    robot.setArm(0.8);
                    toggleTimer=0;
                    scoredCones+=1;
                    stage += 1;

                }
            }
            if (stage == 2) {

                //if (toggleTimer > 20){
                robot.clawAngle.setPosition(0.88);
                //}

                if (toggleTimer > 3){
                    robot.clawOpen();
                    stage +=1 ;
                    toggleTimer = 0;
                }
            }

            if (stage == 3) {


                if (toggleTimer > 20) {
                    robot.clawAngle.setPosition(0.3);
                    robot.setHorz(1300);
                    robot.setArm(intakeArm[scoredCones]);

                    if (bucketcheck){
                        toggleTimer=0;
                        loaded  = true;

                    }else{
                        stage = 0;
                        loaded = false;
                    }

                }
            }
            if (loaded){
                robot.setVert(vertTarget);
                //robot.yGuide.setPosition(0.35);





                if (robot.lVert.getCurrentPosition()>(vertTarget-200)){
                    robot.yGuide.setPosition(0.35);


                    robot.bucketAngle.setPosition(0);

                    sleep(500);
                    robot.bucketAngle.setPosition(0.7);
                    sleep(300);
                    robot.setVert(0);
                    robot.yGuide.setPosition(0.2);
                    loaded=false;
                    stage = 0;


                    //farmingReady = true;



                }


            }
            //End of transfer
            //*************************************************************************************************************

            // TELEMETRY
            telemetry.addData("LF", robot.frontLeft.getCurrentPosition());
            telemetry.addData("RF", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR", robot.backRight.getCurrentPosition());
            telemetry.addData("vert", robot.lVert.getCurrentPosition());
            telemetry.addData("toggletime ", toggleTimer);
            telemetry.addData("loaded", loaded);
            telemetry.addData("timer", timer);
            telemetry.addData("Auton Stage",autonStage);
            telemetry.addData("Stage",stage);
            telemetry.addData("right",robot.rightDis.getDistance(DistanceUnit.CM));
            telemetry.addData("left",robot.leftDis.getDistance(DistanceUnit.CM));
            telemetry.addData("back",robot.backDis.getDistance(DistanceUnit.CM));
            telemetry.addData("Front right",robot.fRightDis.getDistance(DistanceUnit.CM));
            telemetry.addData("heading",heading);
            telemetry.update();




        }
    }
}