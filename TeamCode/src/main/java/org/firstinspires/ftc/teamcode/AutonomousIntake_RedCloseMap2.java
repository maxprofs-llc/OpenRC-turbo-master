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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    double rotateAngle = 0;
    double lastErrorGlyph = 0;
    double shovel_time = 0;

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
        //AutoTransitioner.transitionOnStop(this, "AdwithTeleop");
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
        relicTrackables.activate(); //Vuforia Initialization

        telemetry.addData(">", "~ good ~");
        telemetry.update();
        boat.jewel_arm.setPosition(0.35);

        while(opModeIsActive() == false){

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); // Detect VuMark during init() period of the match
            telemetry.addData("VuMark", "%s visible", vuMark);
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                correctCryptoSlot = "Right";
            } else if (vuMark == RelicRecoveryVuMark.CENTER) { //7.63
                correctCryptoSlot = "Center";
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                correctCryptoSlot = "Left";
            }
            telemetry.addData("Distance (mm)",
                    String.format( "%.02f", boat.distance_sensor.getDistance(DistanceUnit.MM)));
            telemetry.update();

        }

        autoDetectColumn();
        drive(PI, 5.0, .5);
        rotate_arc(-PI/1.6 + PI *10/180, .7);
        boat.winch.setPower(-.2); //set winch
        boat.glyph_aligner.setPosition(.08);
        drive(-PI/2,5,1);

        drive(PI/2,5,1);
        pushMe();
        drive(-PI/2,6,.5);
        sleep(10000);

        int success = 1;
        for(int i = 0; i < 3 && success == 1; i++){ //detect and hit off the right jewel
            success = detect_color("RED", "CLOSE");
            telemetry.addData("Success", success);
            telemetry.update();
        }

        switch(correctCryptoSlot){ //switch to correct cryptobox distance

            case "Center":
                distance = 40.0; //works //works 2/15
                shift = 9.0; //works
                center = 0.0;
                insanity = 16.0;
                break;
            case "Left":
                distance = 40.0 + 8.0; //works add 3 after turn //works 2/15
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



        drive(PI/2, distance, .7); //drive to cryptobox column
        //boat.relic_flop.setPosition(.71); //open up the relic arm //.71

        //drive(PI/2,-32,.7);
        sleep(100);

        //drive(PI, 2.0, .5);
        drive(PI, 4.0, .5);
        //drive(PI, 2.0, .5);


        sleep(100);
        autoDetectColumn(); //self correct within column using distance sensor
        rotate_arc(-PI/1.6 + PI *10/180, .7);
        boat.winch.setPower(-.2); //set winch
        boat.glyph_aligner.setPosition(.08); //move shovel out of the way

        drive(PI/2,10,1);
        drive(PI/2,5,.5);
        diagonal(); //score glyph diagonally


        autoGlyph(shift, center); //try and score autonomous glyph
        drive(-PI / 2, 6, 1); //move out of the way and park

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

    public void autoDetectColumn(){
        boat.jewel_arm.setPosition(.35);
        sleep(500);
        double column_distance = boat.distance_sensor.getDistance(DistanceUnit.MM);
        double target_distance = 160.0; //this distance is how far you want the distance sensor to get to the column
        while(Math.abs(column_distance - target_distance)>2.0 && opModeIsActive()){
            boat.front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boat.front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boat.back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boat.back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (column_distance-target_distance < 0) {
                boat.back_left_motor.setPower(.2);
                boat.front_right_motor.setPower(.2);
                boat.front_left_motor.setPower(.2);
                boat.back_right_motor.setPower(.2);
            }
            else if (column_distance-target_distance > 0){
                boat.back_left_motor.setPower(-.2);
                boat.front_right_motor.setPower(-.2);
                boat.front_left_motor.setPower(-.2);
                boat.back_right_motor.setPower(-.2);
            }
            column_distance = boat.distance_sensor.getDistance(DistanceUnit.MM);
        }
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.jewel_arm.setPosition(0);
        sleep(500);
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

    public void pushMe(){
        boat.right_intake.setPower(.25);
        boat.left_intake.setPower(.25);
        drive(-PI / 2, 3.0, .3);
        sleep(300);
        drive(0, 4.0, .5);
        drive(PI / 2, 8, .5);
    }
    public void diagonal(){
        boat.right_intake.setPower(.25);
        boat.left_intake.setPower(.25);
        drive(-PI / 2, 2.0, .3);
        boat.right_intake.setPower(0.0);
        boat.left_intake.setPower(0.0);
        rotate_arc(-PI/8, .2); // try a bigger angle? used to be PI/8
        drive(-PI/2, 1,1);
        rotate_arc(PI/8, .2);
        drive(0, 3.0, .5);

        drive(PI / 2, 8, .5);
    }
    
    public float getHeading(){
        Orientation angles = boat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;
        float fakeHeading = heading;// - rotate_angle;
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

        angle = convertAngle(angle);


        while ((Math.abs(angle - getHeading()) > threshold) || ((Math.abs(angle) > 180 - threshold) && (Math.abs(Math.abs(angle) - Math.abs(getHeading())) > threshold )) {
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

    public double convertAngle(double angle) {

        rotateAngle = rotateAngle + angle;

        if (rotateAngle > 180) {
         rotateAngle = rotateAngle - 360;
        } else if (rotateAngle < -180) {
         rotateAngle = rotateAngle + 360;
        }

        angle = angle + rotateAngle;

        if (angle > 180) {
         angle = angle - 360;
        } else if (angle < -180) {
         angle = angle + 360;
        }
        return angle;
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

    public void glyph_low(){

        while (Math.abs(boat.armPotentiometer.getVoltage() - .27) > .1 && opModeIsActive()) { // you need to change the .1 for accuracy. Smaller == more accurate but my PID isn't that accurate
            //.1 is the threshold of that this loop will exit. the pot ranges from 0-3.3 over the course of 270 degrees
            if (boat.armPotentiometer.getVoltage() < .50) {
                stallShovel(.23);
            } else if (boat.glyph_aligner.getPosition() == .7 || boat.glyph_aligner.getPosition() == .01) { // CHANGE THE IF STATEMENT DIM WIT
                Playglyph_flipper_runToPosition(.27, .8); //.3 3/6/18 OLD ONE FROM FEB 15 THAT WORKS //.438 SHOULD BE RIGHT 3/5
                //Playglyph_flipper_runToPosition(.25, 1.0); // OLD ONE FROM FEB 15 THAT WORKS
            }

            boat.glyph_grabber.setPosition(.65);
            if (boat.armPotentiometer.getVoltage() < 1.7) {
                boat.glyph_aligner.setPosition(.01);    // CHANGE THE IF STATEMENT DIM WIT             // .01 3/6 march
            } else if (boat.armPotentiometer.getVoltage() > 1.8) {
                boat.glyph_aligner.setPosition(.70);
            }
        }
    }

    public void glyph_release(){
        boat.glyph_grabber.setPosition(.4);
    }

    

    public void glyph_load(){
        while (Math.abs(boat.armPotentiometer.getVoltage()-2.3)>.1 && opModeIsActive()) { // i think the PID thinks it's going to 2.4, but the ideal position is 2.2 so you gotta toy with 2.3 and .1 here


            glyph_flipper_runToPosition(2.40, .3);//2.29 3/6/18

            if (boat.armPotentiometer.getVoltage() > 2.0) {
                boat.glyph_grabber.setPosition(.36); //.28 3/6/18
                boat.glyph_aligner.setPosition(.32); //.38 3/6/18

            } else {
                boat.glyph_aligner.setPosition(.5); //.38 3/6/18

            }
        }
    }

    public void stallShovel( double target){
        double currentPos = boat.armPotentiometer.getVoltage();

        boat.glyph_flipper.setPower(Math.signum(currentPos - target)*((-.25/Math.sqrt(1+2000*(currentPos-target)*(currentPos-target)))+.25));

    }

    public void glyph_flipper_runToPosition(double desiredPosition, double coefficient){ //Replace later with Josh's PID loop
        double desiredTime = 20;
        double potValue = boat.armPotentiometer.getVoltage();
        double kp = .8;   //.8 3/6/18         // proportional konstant
        double ki = 0.345;    //.345 3/6/18         //konstant of integration
        double kd = 0.2;     //.2 //1.7 and 3.0?      //konstant of derivation
        double current = 0;        //value to be sent to shooter motors
        double integralActiveZone = 2.0;    // zone of error values in which the total error for the                    integral term accumulates
        double errorT = 0;                        // total error accumulated
        //double lastError = 0;                    // last error recorded by the controller
        double proportion;                        // the proportional term
        double integral;                            // the integral term
        double derivative;
        double error = potValue - desiredPosition;
        shovel_time = runtime.milliseconds();
// the derivative term
/*////////////////////////////////////////////////////////
  NOTE:
  Error is a float declared at global level, it represents the difference between target velocity and current velocity
  power is a float declared at global level, it represents target velocity
  velocity is a float declared at global level, it is the current measured velocity of the shooter wheels
  /////////////////////////////////////////////*/
//if (current >.2 && Math.abs(error-lastErrorGlyph) < .1 && runtime.milliseconds() > (shovel_time + 500)){
//    shovel_time = runtime.milliseconds();
//    boat.glyph_flipper.setPower(0);
//}
//else {
        if (boat.armPotentiometer.getVoltage() < .002 || boat.armPotentiometer.getVoltage() > 2.7){
            boat.glyph_flipper.setPower(0);
        }else {

                //  drive();
                if (error < integralActiveZone && error != 0) {// total error only accumulates where        /                                                                                            //there is error, and when the error is
                    //within the integral active zone
                    //DON'T
                    // Check for integral until you're within a certain limit
                    errorT += error;// adds error to the total each time through the loop
                } else {
                    errorT = 0;// if error = zero or error is not withing the active zone, total       /                                                    //error is set to zero
                }
                if (errorT > 50 / ki) { //caps total error at 50
                    errorT = 50 / ki;
                }
                if (error == 0) {
                    derivative = 0; // if error is zero derivative term is zero
                }
                proportion = error * kp; // sets proportion term
                integral = errorT * ki;// sets integral term
                derivative = (error - lastErrorGlyph) * kd;// sets derivative term
                lastErrorGlyph = error; // sets the last error to current error so we can use it in the next loop
                current = proportion + integral + derivative;// sets value current as total of all terms

                boat.glyph_flipper.setPower(current * coefficient);

                sleep(20);
                //glyph_flipper_last_time = runtime.milliseconds();// waits so we dont hog all our CPU power or cause loop instability
            }

    }

    public void Playglyph_flipper_runToPosition(double desiredPosition, double coefficient){ //Replace later with Josh's PID loop
        double desiredTime = 20;
        double potValue = boat.armPotentiometer.getVoltage();
        double kp = 1.0;   //.8              // proportional konstant
        double ki = 0.5;    //.4         //konstant of integration
        double kd = 0.1;     //0.0 //1.7 and 3.0?      //konstant of derivation
        double current = 0;        //value to be sent to shooter motors
        double integralActiveZone = 2.0;    // zone of error values in which the total error for the                    integral term accumulates
        double errorT = 0;                        // total error accumulated
        //double lastError = 0;                    // last error recorded by the controller
        double proportion;                        // the proportional term
        double integral;                            // the integral term
        double derivative;
        double error = potValue - desiredPosition;

// the derivative term
/*////////////////////////////////////////////////////////
  NOTE:
  Error is a float declared at global level, it represents the difference between target velocity and current velocity
  power is a float declared at global level, it represents target velocity
  velocity is a float declared at global level, it is the current measured velocity of the shooter wheels
  /////////////////////////////////////////////*/
        if (boat.armPotentiometer.getVoltage() < .002 || boat.armPotentiometer.getVoltage() > 2.7){
            boat.glyph_flipper.setPower(0);
        }else {



                //  drive();
                if (error < integralActiveZone && error != 0) {// total error only accumulates where        /                                                                                            //there is error, and when the error is
                    //within the integral active zone
                    //DON'T
                    // Check for integral until you're within a certain limit
                    errorT += error;// adds error to the total each time through the loop
                } else {
                    errorT = 0;// if error = zero or error is not withing the active zone, total       /                                                    //error is set to zero
                }
                if (errorT > 50 / ki) { //caps total error at 50
                    errorT = 50 / ki;
                }
                if (error == 0) {
                    derivative = 0; // if error is zero derivative term is zero
                }
                proportion = error * kp; // sets proportion term
                integral = errorT * ki;// sets integral term
                derivative = (error - lastErrorGlyph) * kd;// sets derivative term
                lastErrorGlyph = error; // sets the last error to current error so we can use it in the next loop
                current = proportion + integral + derivative;// sets value current as total of all terms

                boat.glyph_flipper.setPower(current * coefficient);

                sleep(20);
                //glyph_flipper_last_time = runtime.milliseconds();// waits so we dont hog all our CPU power or cause loop instability
            }

    }

}
