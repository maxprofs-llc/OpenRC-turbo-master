package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Piter on 12/10/17.
 */

@Autonomous(name="AutonomousIntake_RedCloseMap", group="Pushbot")
public class AutonomousIntake_RedCloseMap extends LinearOpMode {
    NathanPushboat boat = new NathanPushboat(); //Hardware Map
    String teamColor = "RED";
    VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();
    double distance = 0;
    double shift = 0;
    double center = 0;
    double insanity = 0;
    double PI = Math.PI;
    double servo_adjust_last_time = 0;
    String correctCryptoSlot = "Left";
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double scale_factor = 255;
    @Override
    public void runOpMode() {

        telemetry.addData(">", "DONT PRESS THE PLAY BUTTON");
        telemetry.update();
        boat.init(hardwareMap);
        boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reset_drive();
        AutoTransitioner.transitionOnStop(this, "AdwithTeleop");
        telemetry.addData(">", "~ good ~");
        telemetry.update();
        //correctCryptoSlot = detect_init_column();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQ6M0kj/////AAAAGYsMfGvfjkGGlaBJBo84DksXdDgs4AmpEDWNkoag1HAlZ93v7JEK967tDDswHjp6gNrANoJqgPDZCawn6YEnlYzDTzaoufvvImFMlSa94j0OW28rElHTniEc0hbRblm1qEX5pHri02/FTyEAmdbKNW324ljfpHWZnwHp65Bwr8WR9vB6QxkAPJBf+0R3f9H0MuHdKVQkMBC2E97MJVy9fBc3huI5zBOrdEYIvZCf32ktKrw6uTenPZGdpJF4x4VqS4VXFrJ2w+tpWU6pHn2JZM+wLGDy8gYtKKMXKmX2Jfz1U6THFBxlFiXOojOuaFIBN9iPlWvG2AIBRNvbLw8sOW0jmhSWRvOx0bSc/QH3l8By";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        telemetry.addData(">", "~ good ~");
        telemetry.update();
        boat.jewel_arm.setPosition(0);
        //boat.relic_flop.setPosition(.83); //was .83
        boat.relic_noose.setPosition(0);
        while(opModeIsActive() == false){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                correctCryptoSlot = "Right";
            } else if (vuMark == RelicRecoveryVuMark.CENTER) { //7.63
                correctCryptoSlot = "Center";
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                correctCryptoSlot = "Left";
            }
            telemetry.update();
        }

//        if (correctCryptoSlot == "UNKNOWN"){
//            correctCryptoSlot = "Right";
//        }
        int success = 1;
        for(int i = 0; i < 3 && success == 1; i++){
            success = detect_color("RED", "CLOSE");
            telemetry.addData("Success", success);
            telemetry.update();
        }

        switch(correctCryptoSlot){

            case "Center":
                distance = 40.0; //works //works 2/15
                shift = 9.0; //works
                center = 0.0;
                insanity = 16.0;
                break;
            case "Left":
                distance = 40.0 + 5.0; //works add 3 after turn //works 2/15
                shift = -17.0;//works
                center = 10.0;
                insanity = 8.0;
                break;
            case "Right":
                distance = 37.0 - 5.0; //works //WORKS 2/15
                shift = 21.0; //works
                center = -12.0;
                insanity = 22.0;
                break;
        }
        //waitForStart();
        //autoGlyph();
        //boat.glyph_aligner.setPosition(.7);
//        correctCryptoSlot = detect_column();
//        switch(correctCryptoSlot){
//
//            case "Center":
//                distance = 40.0;
//                shift = 7.0;
//                break;
//            case "Left":
//                distance = 37 + 8.0;
//                shift = -14.0;
//                break;
//            case "Right":
//                distance = 37 - 6.0; //used to be 8
//                shift = 14.0;
//                break;
//        }
//autoGlyph();
        //diagonal();
//        drive(PI, 4.0, .6); //3.5 //maybe change to 1.5 worked on 2/14
//sleep(100000);

        drive(PI/2, distance, .7);
        boat.relic_noose.setPosition(1);
        //boat.relic_flop.setPosition(.71); //open up the relic arm //.71
        sleep(500);
        boat.winch.setPower(-.5); //set winch
        boat.glyph_aligner.setPosition(.4);
        //drive(PI/2,-32,.7);
        sleep(100);

        drive(PI, 2.0, .5);
        drive(PI, 2.0, .5);
        drive(PI, 2.0, .5);


        sleep(100);
        rotate_arc(-PI/1.6 + PI *10/180, .7);
        drive(PI/2,5,.7);

        if (correctCryptoSlot == "Left"){
            boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive(0, -3.0, .6); //3.5 //maybe change to 1.5 worked on 2/14
            //drive(PI, 2.0, 1.0); //3.5 //maybe change to 1.5 worked on 2/14
            //drive(PI, 2.0, 1.0);
            //2.5 feb 15 worked ok but was off a little
            boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //sleep(1500);
        diagonal();
        //boat.glyph_aligner.setPosition(.99);
//        autoGlyphInsanity(insanity, center);

        autoGlyph(shift, center); //START OF OLD CODE
        drive(-PI / 2, 6, 1);

    }


    public void autoGlyphInsanity (double distance, double center){
        drive(-PI / 2, 9, 1);
        rotate_arc(PI + PI* 25/180, 1);
        drive(PI, center, .8);
        boat.right_intake.setPower(.6);
        boat.left_intake.setPower(.6);
        drive(PI/2, 8, .7);

        boat.right_intake.setPower(-.3);
        boat.left_intake.setPower(-.3);
        drive(PI/2, 4, .7);

        drive(PI/2, 6, .4);
        rotate_arc(PI/6, .3);
        drive(PI/2, 3, .4);
        drive(-PI/2, 22, .4); //used to be 3
        //rotate_arc(-PI/4, .3);
        rotate_arc(-PI/6, 1);
        drive(PI,5,1);
        //drive(-PI/2, 7, .5); //used to be 11
        rotate_arc(-PI/1.6 - PI *0/180, 1);
        //drive(PI/2, 2, 1);
        drive (PI, 24, .7);
        drive(PI/2, 58, 1);
        drive(0, distance, 1);
        diagonal();
        drive(-PI/2, 6,1);
    }

    public void autoGlyph(double distance, double center){
        drive(-PI / 2, 11, 1);
        rotate_arc(PI + PI* 25/180, 1);
        drive(PI, center, .8);
        boat.right_intake.setPower(.6);
        boat.left_intake.setPower(.6);
        drive(PI/2, 7, 1);

        boat.right_intake.setPower(-.3);
        boat.left_intake.setPower(-.3);
        drive(PI/2, 6, .4);
        rotate_arc(PI/4, .3);
        drive(PI/2, 3, .4);
        drive(-PI/2, 3, .4);
        rotate_arc(-PI/4, .3);
//        rotate_arc(-PI/2, .2);
//        drive(PI/2, 3, .4);
//        drive(-PI/2, 3, .4);
//        rotate_arc(PI/4, .2);

        drive(-PI/2, 6.5, .5);
        rotate_arc(-(PI + PI* 25/180), .7);

        drive(PI, distance+center, .8);
        drive(PI/2, 13, .7);

        diagonal();

    }


    public void autoGlyphAdwith(double distance, double center){
        drive(-PI / 2, 11, 1);
        rotate_arc(PI + PI* 25/180, 1);
        drive(PI, center, .8);
        boat.right_intake.setPower(.6);
        boat.left_intake.setPower(.6);
        drive(PI/2, 7, 1);

        boat.right_intake.setPower(-1);
        boat.left_intake.setPower(-1);
        drive(PI/2, 12, .4);
//        rotate_arc(PI/4, .3);
//        drive(PI/2, 3, .4);
//        drive(-PI/2, 3, .4);
//        rotate_arc(-PI/4, .3);
////        rotate_arc(-PI/2, .2);
////        drive(PI/2, 3, .4);
////        drive(-PI/2, 3, .4);
////        rotate_arc(PI/4, .2);

        drive(-PI/2, 15, .5);
        rotate_arc(-(PI + PI* 25/180), .7);

        drive(PI/2,10,1);

    }
    public void busy(){ //Use to check if motors are running
        //Good to make sure it doesn't continue onto next function while previous function is still running
        double busyStartTime = runtime.milliseconds();

        while(boat.back_left_motor.isBusy() && boat.back_right_motor.isBusy() && boat.front_left_motor.isBusy() && boat.front_right_motor.isBusy() && opModeIsActive()){
            telemetry.addData(">", "Busy " + (runtime.milliseconds() - busyStartTime));
            telemetry.addData(">", "~~~ Motors ~~~");
            telemetry.addData(">", "Front_Left_Motor: " + boat.front_left_motor.getCurrentPosition() / 1120);
            telemetry.addData(">", "Front_Right_Motor: " + boat.front_right_motor.getCurrentPosition() / 1120);
            telemetry.addData(">", "Back_Left_Motor: " + boat.back_left_motor.getCurrentPosition() / 1120);
            telemetry.addData(">", "Back_Right_Motor: " + boat.back_right_motor.getCurrentPosition() / 1120);
            telemetry.addData(">", "Busy List <");
            if(boat.front_left_motor.isBusy()){
                telemetry.addData(">", "Front Left Motor");
            }
            if(boat.front_right_motor.isBusy()){
                telemetry.addData(">", "Front Right Motor");
            }
            if(boat.back_left_motor.isBusy()){
                telemetry.addData(">", "Back Left Motor");
            }
            if(boat.back_right_motor.isBusy()){
                telemetry.addData(">", "Back Right Motor");
            }
            telemetry.update();
        }


    }
    public void rotate_arc (double angle, double power) {
        ///CCW IS POSITIVE

        boat.front_left_motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        boat.back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        boat.front_right_motor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        boat.back_right_motor.setDirection(DcMotor.Direction.FORWARD);//TAKE RADIANS

        double radius_chassis = 21.45/2; //19.5
        double distance = radius_chassis * angle; //distance you want to travel HAS BUILT IN DISTANCE SIDE NEEDED BECAUSE OF THE SIGN OF ANGLE
        double P = -1;
        int conversion_factor = 1120 * 2/35 * 9/8; //9/8 = conversion for ROTATION find THROUGH EXPERIMENTATOIN,  1120*5 rotations should give 2pi4, what does it actually equal?
        int rotations = (int) Math.round(distance * conversion_factor);

        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boat.front_left_motor.setTargetPosition(-rotations + boat.front_left_motor.getCurrentPosition()); //1120 for andy mark
        boat.front_right_motor.setTargetPosition(rotations + boat.front_right_motor.getCurrentPosition());
        boat.back_left_motor.setTargetPosition(-rotations + boat.back_left_motor.getCurrentPosition() );
        boat.back_right_motor.setTargetPosition(rotations + boat.back_right_motor.getCurrentPosition());

        boat.back_left_motor.setPower(-P*power);
        boat.front_right_motor.setPower(P*power);
        boat.front_left_motor.setPower(-P*power);
        boat.back_right_motor.setPower(P*power);
        busy();
    }

    public void diagonal(){
        boat.right_intake.setPower(.25);
        boat.left_intake.setPower(.25);
        drive(-PI / 2, 2.0, .3);
        boat.right_intake.setPower(0.1);
        boat.left_intake.setPower(0.1);
        rotate_arc(-PI/8, .2); // try a bigger angle? used to be PI/8
        drive(-PI/2, 1,1);
        rotate_arc(PI/8, .2);
        drive(0, 3.0, .5);

        drive(PI / 2, 8, .5);
    }
    
    public float getHeading(){
        Orientation angles = boat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;
        float fakeHeading = heading - rotate_angle;
        return fakeHeading;
    }
    
    public void rotate(double angle) {
        double threshold = 2;
        double power = 0;
        double angleDist = 0;
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        while ((Math.get.abs(angle - getHeading()) > threshold) || ((Math.get.abs(angle) > 180 - threshold) && (Math.get.abs(Math.get.abs(angle) - Math.get.abs(getHeading())) > threshold )) {
        angleDist =  Math.abs(angle - getHeading());
        if (angleDist > 180) {
            angleDist = 360 - angleDist;
        }
        power = ((.3 * (angleDist / 180)) + 0.05);
        
        //TURN COUNTER CLOCKWISE
            if (angle - getHeading() > 0 || angle - getHeading() < -180) {
                 boat.back_left_motor.setPower(-1*power);
                 boat.front_right_motor.setPower(power);
                 boat.front_left_motor.setPower(-1*power);
                 boat.back_right_motor.setPower(power);
            } else {
        //TURN CLOCKWISE
                 boat.back_left_motor.setPower(power);
                 boat.front_right_motor.setPower(-1*power);
                 boat.front_left_motor.setPower(power);
                 boat.back_right_motor.setPower(-1*power);
            }
        }
        reset_drive();
        busy();
    }
    
    public void outtake (){
        drive (PI/2, 10, .4);
        sleep(500);

        boat.right_intake.setPower(.2);
        boat.left_intake.setPower(.2);
        sleep(1000);
        boat.right_intake.setPower(0.1);
        boat.left_intake.setPower(0.1);
        drive(PI / 2, 8, .2);
        //rotate_arc(PI + PI* 30/180, .4);
        drive(-PI / 2, 9, .4);


//        boat.right_intake.setPower(.3);
//        boat.left_intake.setPower(.3);
//        drive(-PI/2, 4, .4);
//
//        boat.right_intake.setPower(0.1);
//        boat.left_intake.setPower(0.1);
//        drive(PI/2, 3.5, .3);
//
//
//
//        sleep(100);



        drive(PI / 2, 5, .2);

        drive(-PI / 2, 7, .2);


        //rotate_arc(PI* 30/180, 1); // starting here is trying to intake another glyph
//
//        boat.right_intake.setPower(-.5);
//        boat.left_intake.setPower(-.5);
//        drive(PI/2, 30, 1);
//        boat.right_intake.setPower(0);
//        boat.left_intake.setPower(0);
//        rotate_arc(PI + PI* 30/180, 1);
//
//
//        boat.right_intake.setPower(.2);
//        boat.left_intake.setPower(.5);
//        sleep(750);
//        boat.right_intake.setPower(0.1);
//        boat.left_intake.setPower(0.1);
//        drive(PI / 2, 5, .2);
//        drive(-PI / 2, 10, .2);

    }
    public void drive(double angle, double distance, double power){ //"24 inches" off the balancing stone gives 22.5 inches in actuality
        /*
                 -PI/2
               ==========
               |        |
               |        |
 jewel arm  0  |        | PI
               |        |
               | INTAKE |
                  PI/2

         */

        // DEFINE DIRECTION: front of the ROBOT IS THE INTAKE, define right and left from there
        boat.front_left_motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        boat.back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        boat.front_right_motor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        boat.back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData(">", "Drive Angle: " + angle);
        telemetry.update();
        int Dx = (int) Math.round(Math.sin(angle + Math.PI/4));
        int Dy = (int) Math.round(Math.sin(angle - Math.PI/4));
        double Px = Math.sin(angle + Math.PI/4);
        double Py = Math.sin(angle - Math.PI/4);
        int conversion_factor = 1120 * 2/35; //find THROUGH EXPERIMENTATOIN,  1120*5 rotations should give 2pi4, what does it actually equal?
        int rotations = (int) Math.round(distance * conversion_factor);

        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boat.front_left_motor.setTargetPosition(Dx*rotations + boat.front_left_motor.getCurrentPosition()); //1120 for andy mark
        boat.front_right_motor.setTargetPosition(Dy*rotations + boat.front_right_motor.getCurrentPosition());
        boat.back_left_motor.setTargetPosition(Dy*rotations + boat.back_left_motor.getCurrentPosition());
        boat.back_right_motor.setTargetPosition(Dx*rotations + boat.back_right_motor.getCurrentPosition());

        boat.back_left_motor.setPower(Py*power);
        boat.front_right_motor.setPower(Py*power);
        boat.front_left_motor.setPower(Px*power);
        boat.back_right_motor.setPower(Px*power);


        busy();
    }

    public void reset_drive(){
        telemetry.addData(">", "~ Resetting Encoders ~");
        telemetry.update();

        boat.front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boat.back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(">", "~ Done :D ~");
        telemetry.update();
    }

    public String detect_column(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQ6M0kj/////AAAAGYsMfGvfjkGGlaBJBo84DksXdDgs4AmpEDWNkoag1HAlZ93v7JEK967tDDswHjp6gNrANoJqgPDZCawn6YEnlYzDTzaoufvvImFMlSa94j0OW28rElHTniEc0hbRblm1qEX5pHri02/FTyEAmdbKNW324ljfpHWZnwHp65Bwr8WR9vB6QxkAPJBf+0R3f9H0MuHdKVQkMBC2E97MJVy9fBc3huI5zBOrdEYIvZCf32ktKrw6uTenPZGdpJF4x4VqS4VXFrJ2w+tpWU6pHn2JZM+wLGDy8gYtKKMXKmX2Jfz1U6THFBxlFiXOojOuaFIBN9iPlWvG2AIBRNvbLw8sOW0jmhSWRvOx0bSc/QH3l8By";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        double desiredRunTime = 7000;
        double currentTime = runtime.milliseconds();
        String correctCryptoSlot = "Unknown";

        while(correctCryptoSlot == "Unknown" && opModeIsActive() && runtime.milliseconds() > currentTime && runtime.milliseconds() < currentTime + desiredRunTime) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    correctCryptoSlot = "Right"; //in INCHES 3.8?
                } else if (vuMark == RelicRecoveryVuMark.CENTER) { //7.63
                    correctCryptoSlot = "Center"; //should be 7.63
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    correctCryptoSlot = "Left"; //11.5?
                }
            } else {
                telemetry.addData("VuMark", "not visible " + runtime);
                telemetry.addData("VuMark", ": " + vuMark);
            }
            telemetry.addData(">", "Running VuScan for " + (runtime.milliseconds() - currentTime) + " milliseconds");
            telemetry.update();
        }
        sleep(500);
        if(correctCryptoSlot == "Unknown"){ //If it doesn't see anything it'll go to right one
            correctCryptoSlot = "Center";
        }
        return correctCryptoSlot;
    }

    public String detect_init_column(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQ6M0kj/////AAAAGYsMfGvfjkGGlaBJBo84DksXdDgs4AmpEDWNkoag1HAlZ93v7JEK967tDDswHjp6gNrANoJqgPDZCawn6YEnlYzDTzaoufvvImFMlSa94j0OW28rElHTniEc0hbRblm1qEX5pHri02/FTyEAmdbKNW324ljfpHWZnwHp65Bwr8WR9vB6QxkAPJBf+0R3f9H0MuHdKVQkMBC2E97MJVy9fBc3huI5zBOrdEYIvZCf32ktKrw6uTenPZGdpJF4x4VqS4VXFrJ2w+tpWU6pHn2JZM+wLGDy8gYtKKMXKmX2Jfz1U6THFBxlFiXOojOuaFIBN9iPlWvG2AIBRNvbLw8sOW0jmhSWRvOx0bSc/QH3l8By";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        double currentTime = runtime.milliseconds();
        String correctCryptoSlot = "Unknown";

        while(correctCryptoSlot == "Unknown") {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    correctCryptoSlot = "Right"; //in INCHES 3.8?
                } else if (vuMark == RelicRecoveryVuMark.CENTER) { //7.63
                    correctCryptoSlot = "Center"; //should be 7.63
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    correctCryptoSlot = "Left"; //11.5?
                }
            } else {
                telemetry.addData("VuMark", "not visible " + runtime);
                telemetry.addData("VuMark", ": " + vuMark);
            }
            telemetry.addData(">", "Running VuScan for " + (runtime.milliseconds() - currentTime) + " milliseconds");
            telemetry.update();
        }
        sleep(500);
        return correctCryptoSlot;
    }
    public int detect_color(String teamColor, String location){
        //Assumes color sensor is on the back of jewel arm
        boat.jewel_arm.setPosition(0.65);
        sleep(600);
        double higherPosition = 0.62;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double scale_factor = 255;
        Color.RGBToHSV((int) (boat.color_sensor.red() * scale_factor),
                (int) (boat.color_sensor.green() * scale_factor),
                (int) (boat.color_sensor.blue() * scale_factor),
                hsvValues);
        telemetry.addData("Values", hsvValues[0]);
        if (hsvValues[0] <= 250 && hsvValues[0] >= 90) {
            telemetry.addData(">", "Blue");
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLUE);
                }
            });
            switch(teamColor){
                case "BLUE":
                    if(location == "FAR"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(400);
                    }
                    rotate_arc(PI / 8, 0.4);
                    sleep(100);
                    boat.jewel_arm.setPosition(0);
                    sleep(200);
                    rotate_arc(-PI / 8, 0.4);
                    break;
                case "RED":
                    if(location == "FAR"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(400);
                    }
                    rotate_arc(-PI / 8, 0.4);
                    sleep(100);
                    boat.jewel_arm.setPosition(0);
                    sleep(200);
                    rotate_arc(PI / 8, 0.4);
                    break;
            }
        } else if (hsvValues[0] <= 25 || (hsvValues[0] >= 300 && hsvValues[0] <= 400)) {
            telemetry.addData(">", "Red");
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.RED);
                }
            });
            switch (teamColor) {
                case "BLUE":
                    if(location == "CLOSE"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(400);
                    }
                    rotate_arc(-PI / 8, 0.4);
                    sleep(100);
                    boat.jewel_arm.setPosition(0);
                    sleep(200);
                    rotate_arc(PI / 8, 0.4);
                    break;
                case "RED":
                    if(location == "CLOSE"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(400);
                    }
                    rotate_arc(PI / 8, 0.4);
                    sleep(100);
                    boat.jewel_arm.setPosition(0);
                    sleep(200);
                    rotate_arc(-PI / 8, 0.4);
                    break;
            }
        } else {
            telemetry.addData(">", "Unknown");
            telemetry.update();
            boat.jewel_arm.setPosition(0);
            sleep(200);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLACK);
                }
            });
            return 1;
        }
        telemetry.update();
        return 0;
    }

    public void colorTest() {
        while (opModeIsActive()) {
            float hsvValues[] = {0F, 0F, 0F};
            final float values[] = hsvValues;
            final double scale_factor = 255;
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            Color.RGBToHSV((int) (boat.color_sensor.red() * scale_factor),
                    (int) (boat.color_sensor.green() * scale_factor),
                    (int) (boat.color_sensor.blue() * scale_factor),
                    hsvValues);
            telemetry.addData("Values", hsvValues[0]);
            if(hsvValues[0] <= 250 && hsvValues[0] >= 120){
                telemetry.addData(">", "Blue");
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.BLUE);
                    }
                });
            }
            else if(hsvValues[0] <= 25 || (hsvValues[0] >= 300 && hsvValues[0] <= 400)){
                telemetry.addData(">", "Red");
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.RED);
                    }
                });
            }
            else{
                telemetry.addData(">", "Unknown");
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.BLACK);
                    }
                });
            }
            telemetry.update();
        }
    }
}
