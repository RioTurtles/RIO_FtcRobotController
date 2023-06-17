
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="tommy3")

public class Project3 extends LinearOpMode
{
    final static double ARM_HOME = 0.0;
    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;
    final static double HEXMOTOR_TPR = 288;
    final static double GOBILDA_TPR = 751.8; //5202
    final static double GEAR_REDUCTION = 1;
    final static double TICKS_PER_DEGREE = (HEXMOTOR_TPR * GEAR_REDUCTION) / 360;
    //ticks = pulses, cycles = ticks * 4

    final String[] debugModes =  {"VERT", "HORZ", "BUCKET", "BUCKET ANGLE", "CLAW", "CLAW ANGLE"};
    int dModesIndex = 0;
    int stage = 0;
    int vertTarget = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");


        robot.imu1.resetYaw();

        //variables
        double direction_y, direction_x, pivot, heading;
        boolean clawOpen = true;
        double toggleTimer = 0, toggleOrig1 = 0, toggleInterval = 50;
        double toggleTimer2 =0;
        double toggleTimer3 =0;
        double toggleTimer4 =0;

        int vertPos;
        int vertTarget =0 ;

        int horzTarget =0;
        int farmingHorz = 1800;


        String mode = "OFF";
        robot.setArm(0.9);
        robot.clawAngle.setPosition(0.3);
        robot.yGuide.setPosition(0.2);
        robot.clawOpen();
        waitForStart();
        drivetrain.remote(0,0,0,0);



        ElapsedTime timer = new ElapsedTime();
        //timer.reset();

        double cur = timer.milliseconds();
        //boolean bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 100) || (robot.bucketColor.getNormalizedColors().blue>100));
        boolean ground = false;
        boolean loaded = false;
        boolean realse = false;
        boolean raise = false;
        boolean lowcontrol = false;
        boolean farming =false;
        boolean farmingReady =true;


        boolean bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 0.05) || (robot.bucketColor.getNormalizedColors().blue>0.05));

        robot.lHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        while (opModeIsActive()) {
            direction_y   = gamepad1.left_stick_y;
            direction_x = -gamepad1.left_stick_x;
            pivot    =  gamepad1.right_stick_x;
            heading = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 0.01) || (robot.bucketColor.getNormalizedColors().blue>0.01));

            toggleTimer +=1 ;
            toggleTimer2 +=1 ;
            toggleTimer3 +=1;
            toggleTimer4 +=1;
            //vertPos = robot.vert.getCurrentPosition();

            //stage for transfer
            if (stage == 0) {
                robot.bucketAngle.setPosition(0.65);
                robot.clawAngle.setPosition(0.3);
                //robot.setArm(0.03);
                robot.yGuide.setPosition(0.2);
                if ((!loaded) && (gamepad1.right_bumper)){
                    drivetrain.remote(0,0,0,0);
                    robot.clawClose();
                    robot.lHorz.setPower(0);
                    robot.rHorz.setPower(0);
                    sleep(200);
                    stage +=1;
                }
            }
            if (stage == 1){
                if(robot.lHorz.getCurrentPosition()<20){
                    robot.setArm(0.8);
                    toggleTimer = 0;
                    stage += 1;
                }
                else {
                    robot.setArm(0.7);
                }
            }
            if (stage == 2){
                if (toggleTimer > 10){
                    robot.clawAngle.setPosition(0.9);
                }
                if (toggleTimer > 15){
                    robot.clawOpen();


                    stage += 1;
                    toggleTimer = 0;
                }
            }
            if (stage == 3){
                if (toggleTimer > 5){
                    robot.clawAngle.setPosition(0.3);

                }
            }


            //put cone on junction
            if (gamepad1.cross){
                vertTarget = 0;
                loaded = true;
            }
            else if (gamepad1.circle){
                vertTarget = 1100;
                loaded = true;
            }
            else if (gamepad1.triangle){
                vertTarget = 2300;
                loaded = true;
            }

            if (loaded){
                robot.setVert(vertTarget);
                if (robot.lVert.getCurrentPosition() > vertTarget/2){
                    robot.yGuide.setPosition(0.35);
                }
            }

            if ((loaded) && (gamepad1.right_bumper)){
                drivetrain.remote(0,0,0,0);
                robot.bucketAngle.setPosition(0);
                sleep(600);
                robot.bucketAngle.setPosition(0.65);
                sleep(300);
                robot.setVert(0);
                stage = 0;
                loaded=false;

            }

            //horizontal slider
            if (gamepad1.left_bumper){
                robot.setArm(0.03);
                toggleTimer = 0;
                if (toggleTimer4 > 10){
                    robot.clawAngle.setPosition(0.28);//horizontal position
                    if((toggleTimer2>10)&&(robot.lHorz.getCurrentPosition()<3000)){
                        horzTarget =robot.lHorz.getCurrentPosition()+400;//speed of horizontal
                        robot.setHorz(horzTarget);
                    }

                }
            }
            else{
                horzTarget=0;
                robot.setHorz(horzTarget);
                telemetry.addLine("retract");
            }

            //farming
            if (gamepad1.dpad_up){
                farming = true;

            }




            /*telemetry.addData("LF", robot.frontLeft.getCurrentPosition());
            telemetry.addData("RF", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR", robot.backRight.getCurrentPosition());*/
            telemetry.addData("bucketcheck", bucketcheck);

            /*if ((toggleTimer-toggleOrig1) > toggleInterval) {
                if (gamepad1.right_bumper ){
                    clawOpen = (!clawOpen);
                    toggleTimer = toggleOrig1;
                }

            }*/
            if(gamepad1.dpad_down){
                robot.imu1.resetYaw();
            }



            //DRIVETRAIN
            if (robot.lVert.getCurrentPosition()>800) {  //Brakes/slow down car
                direction_x = direction_x/2;
                direction_y = direction_y/2;
                pivot = pivot/2;
            } // brake
            if ((gamepad1.left_trigger == 0) && (gamepad1.right_trigger == 0)){
                drivetrain.remote(-direction_y, -direction_x, -pivot, heading);
            } // normal drive
            //else {
            //drivetrain.remote(0, ((-gamepad1.right_trigger+ gamepad1.left_trigger)*0.3), pivot, 0);
            //} // horizontal strafe

            //if (gamepad1.touchpad) {
            //robot.imu1.resetYaw();
            //}

            //if (gamepad1.square || gamepad1.circle || gamepad1.triangle || gamepad1.cross){
                /*switch (mode) {
                    case "OFF": robot.vert.setTargetPosition(0);
                    case "LOW": robot.vert.setTargetPosition(500);
                    case "MID": robot.vert.setTargetPosition(800);
                    case "HIGH": robot.vert.setTargetPosition(1300);
                }*/

            //}


            //***************************************************************










            //TELEMETRY

            //telemetry.addData("servopos1", servopos);
            //telemetry.addData("servopos2", servopos2);
            //telemetry.addData("servopos3", servopos3);
            // telemetry.addData("vert height", robot.lVert.getCurrentPosition());
            telemetry.addData("time", toggleTimer);
            telemetry.addData("stage",stage);
            telemetry.addData("Horz",robot.lHorz.getCurrentPosition());
            telemetry.addData("Horz Target",horzTarget);
            telemetry.addData("Vert",robot.lVert.getCurrentPosition());

            telemetry.addData("red",robot.bucketColor.getNormalizedColors().red );
            telemetry.addData("blue",robot.bucketColor.getNormalizedColors().blue );
            telemetry.addData("farming",farming);
            //telemetry.addData("yaw1", robot.yaw1.getPosition());
            //telemetry.addData("yaw2", robot.yaw2.getPosition());*/
            telemetry.addData("heading",heading);


            telemetry.update();

            //while(gamepad1.left_stick_button){robot.yaw1.setPosition(-0.7);}

        }
    }
}
