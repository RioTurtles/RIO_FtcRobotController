
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="tommy2")

public class Project2 extends LinearOpMode
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

        int vertPos;
        int vertTarget =0 ;

        int horzTarget =0;
        int farmingHorz = 1800;


        String mode = "OFF";
        robot.setArm(0.9);
        robot.clawAngle.setPosition(0.3);
        robot.yGuide.setPosition(0.15);
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
            bucketcheck = ((robot.bucketColor.getNormalizedColors().red> 0.05) || (robot.bucketColor.getNormalizedColors().blue>0.05));

            toggleTimer +=1 ;
            toggleTimer2 +=1 ;
            toggleTimer3 +=1;
            //vertPos = robot.vert.getCurrentPosition();
            /*
            //if (gamepad1.square){
                robot.flip1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               // robot.flip2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //robot.flip1.setDirection(DcMotor.Direction.REVERSE);
                //while (Math.abs(robot.flip1.getCurrentPosition()-pidTarget) <= 10){ // CHANGE WHILE LOOP TO IF STATEMENT after tuning the values to keep the motor running
                    if (gamepad1.circle){
                     pidTarget += 10;
                    }
                    cur = robot.flip1.getCurrentPosition();
                    pidTime1 = timer.milliseconds();
                    lastError = error;
                    error = pidTarget-cur;
                    I += error * (pidTime1-pidTime);
                    D = (error - lastError) / (pidTime1-pidTime);

                    double out = (error * Kp) + (I * Ki) + (D * Kd);

                    ((DcMotorEx) robot.flip1).setPower(out);


                    pidTime = pidTime1;

                    telemetry.addData("Encoder value", robot.flip1.getCurrentPosition());
                    //debug if u want it la
               // }
           // }
           */



            // INTAKE


            /*else if (gamepad1.dpad_up){
                robot.vert.setTargetPosition(vertPos+20);
            }
            else if (gamepad1.dpad_down){
                robot.vert.setTargetPosition(vertPos-20);
            }



            else if (gamepad1.left_stick_button){
                robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/

            telemetry.addData("LF", robot.frontLeft.getCurrentPosition());
            telemetry.addData("RF", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR", robot.backRight.getCurrentPosition());

            /*if ((toggleTimer-toggleOrig1) > toggleInterval) {
                if (gamepad1.right_bumper ){
                    clawOpen = (!clawOpen);
                    toggleTimer = toggleOrig1;
                }

            }*/



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
            //TRANSFER
            if (gamepad1.square){
                vertTarget = 0;
                ground=true;
            }
            else if (gamepad1.cross){
                vertTarget = 0;
                ground=false;
            }
            else if (gamepad1.circle){
                vertTarget = 1100;
                ground=false;
            }
            else if (gamepad1.triangle){
                vertTarget = 2300;
                ground=false;
            }


            if (stage == 0){
                robot.bucketAngle.setPosition(0.68);
                robot.clawAngle.setPosition(0.3);
                //robot.setArm(0.05);
                robot.yGuide.setPosition(0.15);


                if(!farming){
                    if (gamepad1.right_bumper){


                        drivetrain.remote(0,0,0,0);
                        robot.clawClose();
                        robot.lHorz.setPower(0);
                        robot.rHorz.setPower(0);
                        sleep(200);
                        toggleTimer=0;
                        stage += 1;

                    }else{
                        robot.clawOpen();
                    }
                }else{
                    robot.setArm(0.05);
                    robot.setHorz(farmingHorz);

                    if (robot.lHorz.getCurrentPosition()>farmingHorz-200){
                        drivetrain.remote(0, 0, pivot, 0);
                        robot.clawClose();
                        sleep(500);
                        robot.setArm(0.4);
                        sleep(500);//half lifted
                        robot.setHorz(0);
                        stage+=1;
                    }
                }
            }
            if (stage==1){
                if(!ground){
                    if (robot.lHorz.getCurrentPosition()<20){
                        robot.setArm(0.8);

                        toggleTimer=0;
                        stage += 1;

                    }else{
                        robot.setArm(0.4);
                    }

                }else {
                    robot.setArm(0.8);

                    if(gamepad1.right_bumper){
                        robot.clawAngle.setPosition(0.3);//horizontal
                        robot.setArm(0.05);//ground level
                        drivetrain.remote(0,0,0,0);
                        sleep(500);
                        robot.clawOpen();
                        stage=0;

                    }

                }

            }


            if (stage ==2){

                if (toggleTimer > 20){
                    robot.clawAngle.setPosition(0.9);
                }

                if (toggleTimer > 30){
                    robot.clawOpen();
                    stage +=1 ;
                    toggleTimer = 0;
                }
            }

            if (stage ==3){


                if (toggleTimer > 20||bucketcheck) {
                    robot.clawAngle.setPosition(0.3);

                    if (bucketcheck){
                        toggleTimer=0;
                        loaded  = true;

                    }else{
                        stage = 0;
                        loaded = false;



                    }
                }

                if (loaded){
                    //telemetry.addLine("loaded");
                    if (gamepad1.right_bumper){
                        robot.setVert(vertTarget);

                    }
                    if (farming){
                        robot.setVert(vertTarget);
                        robot.setArm(0.05);
                        robot.setHorz(farmingHorz-800);
                        lowcontrol=false;

                    }

                    if(vertTarget ==0){
                        if (gamepad1.right_bumper){
                            robot.yGuide.setPosition(0.35);
                            lowcontrol = true;

                        }
                    }else{
                        lowcontrol =false;
                        if (robot.lVert.getCurrentPosition()>(vertTarget-200)){
                            robot.yGuide.setPosition(0.35);

                            if (!gamepad1.right_bumper||farming){ drivetrain.remote(0,0,0,0);
                                robot.bucketAngle.setPosition(0);

                                sleep(600);
                                robot.bucketAngle.setPosition(0.7);
                                sleep(300);
                                robot.setVert(0);
                                stage = 0;
                                loaded=false;

                                //farmingReady = true;

                            }

                        }
                    }

                }
            }
            if(!gamepad1.right_bumper&& lowcontrol){
                drivetrain.remote(0,0,0,0);
                robot.bucketAngle.setPosition(0);
                sleep(600);
                robot.bucketAngle.setPosition(0.7);
                sleep(200);
                robot.setVert(0);
                lowcontrol = false;
                loaded=false;
                stage = 0;
            }

            //Farming
            if (gamepad1.dpad_up && toggleTimer3 >20){
                farming = !farming;
                toggleTimer3 =0;
            }
            if(gamepad1.dpad_down){
                robot.imu1.resetYaw();
            }




            /*Horizontal*/
            if(!farming){
                if (gamepad1.left_bumper){
                    if(ground&&loaded) {
                        robot.setArm(0.3);//lifting position

                    } else{
                        robot.setArm(0.05);//intake position
                    }
                    robot.clawAngle.setPosition(0.3);//horizontal position
                    if((toggleTimer2>10)&&(robot.lHorz.getCurrentPosition()<3000)){
                        horzTarget =robot.lHorz.getCurrentPosition()+400;//speed of horizontal
                        robot.setHorz(horzTarget);
                    }
                    telemetry.addLine("extend");

                }else{
                    horzTarget=0;
                    robot.setHorz(horzTarget);
                    telemetry.addLine("retract");
                    if(robot.lHorz.getCurrentPosition() <50){
                        toggleTimer2 = 0;

                    }
                }
            }
            /*
            if (gamepad1.dpad_up){
                robot.clawOpen();
                robot.setHorz(500);
                sleep(10000);
                robot.clawClose();
                robot.setArm(0.4);
                robot.setHorz(0);
                sleep(10000);

            }*/









            //TELEMETRY

            //telemetry.addData("servopos1", servopos);
            //telemetry.addData("servopos2", servopos2);
            //telemetry.addData("servopos3", servopos3);
            // telemetry.addData("vert height", robot.lVert.getCurrentPosition());
            //telemetry.addData("time", toggleTimer);
            telemetry.addData("stage",stage);
            telemetry.addData("Horz",robot.lHorz.getCurrentPosition());
            telemetry.addData("Horz Target",horzTarget);
            telemetry.addData("Horz",robot.lVert.getCurrentPosition());

            telemetry.addData("red",robot.bucketColor.getNormalizedColors().red );
            telemetry.addData("blue",robot.bucketColor.getNormalizedColors().blue );
            telemetry.addData("farming",farming);
            //telemetry.addData("yaw1", robot.yaw1.getPosition());
            //telemetry.addData("yaw2", robot.yaw2.getPosition());*/

            telemetry.update();

            //while(gamepad1.left_stick_button){robot.yaw1.setPosition(-0.7);}

        }
    }
}
