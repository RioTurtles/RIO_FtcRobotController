
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
        double xPos = -robot.frontRight.getCurrentPosition();
        double yPos = -robot.frontLeft.getCurrentPosition();
        
       
        int vertPos;
        int vertTarget =0 ;
        
        int horzTarget =0;
        int farmingHorz = 1300;
        
        int loops = 0;
        
    
        String mode = "OFF";
        robot.setArm(0.9);
        robot.clawAngle.setPosition(0.28);
        robot.yGuide.setPosition(0.2);
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
        boolean clawDown =true;
        boolean groundScoring =false;
        
        
        
        boolean bucketcheck = ((robot.bucketColor.red()> 70) || (robot.bucketColor.blue()>70));
        boolean clawcheck = ((robot.clawColor.red()> 100) || (robot.clawColor.blue()>100));
        
        robot.lHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rHorz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.reset();


        
    
        while (opModeIsActive()) {
           direction_y   = gamepad1.left_stick_y;
            direction_x = -gamepad1.left_stick_x;
            pivot    =  gamepad1.right_stick_x * 0.8;
            heading = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            bucketcheck = (((robot.bucketColor.red()> 90) || (robot.bucketColor.blue()>90))&&((robot.bucketColor.green()<robot.bucketColor.blue())||(robot.bucketColor.green()<robot.bucketColor.red())));
            //clawcheck = (((robot.clawColor.red()> 100) || (robot.clawColor.blue()>100))&&((robot.clawColor.green()<robot.clawColor.blue())||(robot.clawColor.green()<robot.clawColor.red())));
            clawcheck = false;
            xPos = -robot.frontRight.getCurrentPosition();
            yPos = -robot.frontLeft.getCurrentPosition();
            cur = timer.milliseconds();
            loops+=1;
            

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
                vertTarget = 730;
                ground=false;
            }
            else if (gamepad1.triangle){
                vertTarget = 1500;
                ground=false;
            }


            if (stage == 0){
                robot.bucketAngle.setPosition(0.65);
                //robot.clawAngle.setPosition(0.28);
                
                robot.yGuide.setPosition(0.2);
                
                
                if(!farming){
                    if ((gamepad1.right_bumper||clawcheck)&&clawDown){
                            
                    
                        drivetrain.remote(0,0,0,0);
                        robot.clawClose();
                        robot.lHorz.setPower(0);
                        robot.rHorz.setPower(0);
                        sleep(200);
                        horzTarget=0;
                        robot.setHorz(horzTarget);
                        toggleTimer=0;
                        stage += 1;

                    }else{
                        robot.clawOpen();
                    }
                }else{
                    robot.setArm(0.05);
                    robot.clawAngle.setPosition(0.28);
                    robot.setHorz(farmingHorz);
                    
                    if (robot.lHorz.getCurrentPosition()>farmingHorz-200||clawcheck){
                        drivetrain.remote(0, 0, pivot, 0);
                        robot.clawClose();
                        sleep(200);
                        robot.setArm(0.4);
                        //sleep(500);//half lifted
                        robot.setHorz(0);
                        stage+=1;
                    }
                }
            }
            if (stage==1){
                if(!ground){
                    clawDown=false;
                   if (((robot.lHorz.getCurrentPosition()<500)||(robot.lHorz.getCurrentPosition()<1800&&robot.lHorz.getCurrentPosition()>1000))&&(toggleTimer>5)){
                        robot.setArm(0.85);
                        robot.clawAngle.setPosition(0.65);
                        
                        toggleTimer=0;
                        stage += 1;
                    
                        }else{
                            robot.setArm(0.7);
                        }   
                    
                }else {
                    robot.setArm(0.85);
                
                    if(gamepad1.right_bumper){
                        robot.clawAngle.setPosition(0.28);//horizontal
                        robot.setArm(0.2);//ground level
                        groundScoring = true;
                    
                    }
                    if(groundScoring&&!gamepad1.right_bumper&&toggleTimer>5){
                        drivetrain.remote(0,0,0,0);
                        robot.clawOpen();
                        sleep(500);
                        groundScoring=false;
                        stage=0;
                        
                    }
                }

            }

            
            if (stage ==2){
                
                if (toggleTimer > 3){
                    robot.clawAngle.setPosition(0.8);
                    
                }
                bucketcheck = (((robot.bucketColor.red()> 80) || (robot.bucketColor.blue()>80))&&((robot.bucketColor.green()<robot.clawColor.blue())||(robot.bucketColor.green()<robot.bucketColor.red())));

                if (toggleTimer > 15){
                    robot.clawOpen();
                    stage +=1 ;
                    toggleTimer = 0;
                }
            }

            if (stage ==3){

                
                if (toggleTimer > 3) {
                    robot.clawAngle.setPosition(0.6);
                    robot.setArm(0.05);
                    

                    if (bucketcheck&&toggleTimer>5){
                       toggleTimer=0;
                        loaded  = true;
                        
                    }//else{
                        //stage = 0;
                       // loaded = false;
                        


                    //}
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
                    robot.clawAngle.setPosition(0.28);
                    robot.setHorz(farmingHorz-800);
                    lowcontrol=false;
                    
                }
                
                if(vertTarget ==0){
                    if (gamepad1.right_bumper){
                        robot.yGuide.setPosition(0.4);
                        lowcontrol = true;
                        
                    }
                }else{
                    lowcontrol =false;
                    if (robot.lVert.getCurrentPosition()>(vertTarget-200)){
                        robot.yGuide.setPosition(0.35);
                        
                        if (!gamepad1.right_bumper||farming){ drivetrain.remote(0,0,0,0);
                            robot.bucketAngle.setPosition(0);
                            
                            sleep(600);
                            robot.bucketAngle.setPosition(0.65);
                            sleep(300);
                            robot.setVert(0);
                            stage = 0;
                            loaded=false;
                            
                            //farmingReady = true;
                           
                        }
                        
                        }
                    }
                
                
            }
            if(!gamepad1.right_bumper&& lowcontrol){
                            drivetrain.remote(0,0,0,0);
                            robot.bucketAngle.setPosition(0);
                            sleep(600);
                            robot.bucketAngle.setPosition(0.65);
                            sleep(200);
                            robot.setVert(0);
                            robot.yGuide.setPosition(0.2);
                            lowcontrol = false;
                            loaded=false;
                            stage = 0;
                        }

            //Farming
            if (gamepad1.dpad_up && toggleTimer3 >20){
                farming = !farming;
                vertTarget = 1500;
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
                        robot.clawAngle.setPosition(0.28);

                    } else{
                    robot.setArm(0.05);//intake position
                    }
                    robot.clawAngle.setPosition(0.28);//horizontal position
                    clawDown = true;
                    if((toggleTimer2>5)&&(robot.lHorz.getCurrentPosition()<2000)){
                        horzTarget =robot.lHorz.getCurrentPosition()+400;//speed of horizontal
                        robot.setHorz(horzTarget);
                    }
                    telemetry.addLine("extend");

                }else{
                    //horzTarget=0;
                    //robot.setHorz(horzTarget);
                    telemetry.addLine("retract");
                    if(robot.lHorz.getCurrentPosition() <50){
                        toggleTimer2 = 0;
                   
                    }
                }
            }
            if(gamepad1.right_trigger>0.2){
                robot.setArm(0.15);
                
            }
            if(gamepad1.left_trigger>0.2){//hardRest
                stage =0;
                loaded=false;
                robot.setHorz(0);
                
                
            }


            

        
            


             //TELEMETRY

            //telemetry.addData("servopos1", servopos);
            //telemetry.addData("servopos2", servopos2);
            //telemetry.addData("servopos3", servopos3);
            // telemetry.addData("vert height", robot.lVert.getCurrentPosition());
            telemetry.addData("time", toggleTimer3);
            telemetry.addData("stage",stage);
            telemetry.addData("Horz",robot.lHorz.getCurrentPosition());
            telemetry.addData("Horz Target",horzTarget);
            telemetry.addData("Vert",robot.lVert.getCurrentPosition());

            telemetry.addData("red",robot.bucketColor.red() );
            telemetry.addData("blue",robot.bucketColor.blue() );
            telemetry.addData("farming",farming);
            //telemetry.addData("yaw1", robot.yaw1.getPosition());
            //telemetry.addData("yaw2", robot.yaw2.getPosition());*/
            telemetry.addData("heading",heading);
            telemetry.addData("Hz",loops/cur*1000);


            telemetry.update();

            //while(gamepad1.left_stick_button){robot.yaw1.setPosition(-0.7);}

        }
    }
}