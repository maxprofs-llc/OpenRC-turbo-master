package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TestRelic", group="Pushbot")
public class TestReliv extends LinearOpMode {
    NathanPushboat boat = new NathanPushboat();
    private ElapsedTime runtime = new ElapsedTime();
    //////////////////////
    /* TOGGLE VARIABLES */
    //////////////////////

    Boolean gamepad2_y_toggle = false;
    Boolean gamepad2_a_toggle = false;
    Boolean gamepad2_b_toggle = false;
    Boolean gamepad2_x_toggle = false;
    Boolean gamepad2_right_bumper_toggle = false;

    /////////////////////////////
    /* DRIVE RELATED VARIABLES */
    /////////////////////////////

    float stick_x = 0;
    float stick_y = 0;
    double gyroAngle = 0;
    float rotate_angle = 0;
    double theta = 0;
    double Px = 0;
    double Py = 0;
    double threshold = 0.05;
    double Protate = 0;
    double maxRotate = 0.29;
    int moveAngle = 9001;
    double currentTimeForAngle = 0;
    double servo_angle;
    double servo_angle2;
    boolean applePie = true;
    double lastErrorGlyph = 0;
    double lastErrorIntake = 0;

    //////////////////////////////////
    /* GLYPH FLIP RELATED VARIABLES */
    //////////////////////////////////

    double glyph_flip_last_press_time = 0;
    double servo_adjust_last_time = 0;
    double glyph_flipper_last_time = 0;
    double glyph_flip_stomp_last_time = 0;
    Boolean glyph_grabber_openness = false;
    double glyph_grab_last_press_time = 0;
    double errorServo = .835; //could be .26
    double firstError;
    double shovelServo;
    @Override
    public void runOpMode() {
        boat.init(hardwareMap);
        servo_angle = boat.relic_noose.getPosition();
        servo_angle2 = boat.relic_flop.getPosition();
        //boat.glyph_aligner.setPosition(boat.glyph_aligner.getPosition());

        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //boat.intake_aligner.setPower(0);
        telemetry.addData("Say", "Nathan Boat 1.0 without crossbow is ready to be sailed!");
        telemetry.update();

        while(!opModeIsActive()){
            // intake_runToPosition(1.2,.5);
            // boat.glyph_grabber.setPosition(1);
            //boat.glyph_aligner.setPosition(.5);
            firstError = boat.armPotentiometer.getVoltage();


        }
        while(opModeIsActive()){
            // boat.jewel_arm.setPosition(56.0/180.0);
            servo_angle = servoAdjustAligner(servo_angle);
            //glyph_flipper_adjust();
            telemetry.addData(">", "Current Angle: " + boat.glyph_aligner.getPosition());
            //intake(); // this encompasses all intake functions necessary
            getPotValues();
            //Conveyor();
            //intakeSimple();
            //drive();
            // testMotor();
            //glyph_flip();
            //glyph_flip_low();
            //glyph_flip_high();

            //relicGrab();
            //drive();
            testMotor();
            // testMotor();
            //   glyph_flip();
            // glyph_flipper_adjust();

            //Conveyor();
//            telemetry.addData(">", "flop angle " + boat.relic_flop.getPosition());
//            telemetry.addData(">", "noose angle " + boat.relic_noose.getPosition());
            telemetry.update();//THIS GOES AT THE END
        }
    }

    /////////////////////////////
    /////////////////////////////
    /* DRIVE RELATED FUNCTIONS */
    /////////////////////////////
    /////////////////////////////
    public void drive() {
        stick_x = gamepad1.left_stick_x;
        stick_y = gamepad1.left_stick_y;
        stick_x = stick_x / 2;
        stick_y = stick_y / 2;

        if(gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5 && gamepad1.a) {
            setCurrentHeadingToZero();
        }

        gyroAngle = getHeading() * Math.PI / 180;
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;
        //MOVEMENT
        if (gamepad1.right_trigger < 0.5) {
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
        } else if (gamepad1.right_trigger > 0.5) {
            //Runs Px Py independent of gyro (old drive) -> DOUBLE SPEED GYRO
            stick_x = stick_x * 2;
            stick_y = stick_y * 2;
            if (gamepad1.left_bumper) {
                stick_x = stick_x / 3;
                stick_y = stick_y / 3;
            }
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));

            // theta = (Math.atan2(stick_y, stick_x)) - 0; //burt is an idiot
            // Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            // Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
        }
        //ROTATION
        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            if (Px == 0 && Py == 0) {
                Protate = gamepad1.right_stick_x * .5;
            } else {
                Protate = gamepad1.right_stick_x * 3 / 2 * maxRotate;
                Px = Px / 3 * 2;
                Py = Py / 3 * 2;
            }
        } else {
            Protate = 0;
        }
        //SPINNING LEFT IS NEGATIVE POWER

        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger < 0.5) {
            rotateToAngle();
        }
        if (gamepad1.left_trigger <= 0.5) { // cardinal directions
            if (gamepad1.dpad_down) {
                boat.back_left_motor.setPower(-.5);
                boat.front_right_motor.setPower(-.5);
                boat.front_left_motor.setPower(-.5);
                boat.back_right_motor.setPower(-.5);
            } else if (gamepad1.dpad_left) {
                boat.back_left_motor.setPower(.5);
                boat.front_right_motor.setPower(.5);
                boat.front_left_motor.setPower(-.5);
                boat.back_right_motor.setPower(-.5);
            } else if (gamepad1.dpad_right) {
                boat.back_left_motor.setPower(-.5);
                boat.front_right_motor.setPower(-.5);
                boat.front_left_motor.setPower(.5);
                boat.back_right_motor.setPower(.5);
            } else if (gamepad1.dpad_up) {
                boat.back_left_motor.setPower(.5);
                boat.front_right_motor.setPower(.5);
                boat.front_left_motor.setPower(.5);
                boat.back_right_motor.setPower(.5);
            } else {
                boat.back_left_motor.setPower(-(Px - Protate));
                boat.front_right_motor.setPower(-(Px + Protate));
                boat.front_left_motor.setPower(-(Py - Protate));
                boat.back_right_motor.setPower(-(Py + Protate));
            }
        } else if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger < 0.5) { //rotation around off center axis
            if (gamepad1.right_stick_x > 0) {
                boat.back_left_motor.setPower(.7);
                boat.front_right_motor.setPower(0);
                boat.front_left_motor.setPower(0);
                boat.back_right_motor.setPower(-.7);
            } else if (gamepad1.right_stick_x < 0) {
                boat.back_left_motor.setPower(-.7);
                boat.front_right_motor.setPower(0);
                boat.front_left_motor.setPower(0);
                boat.back_right_motor.setPower(.7);
            } else {
                boat.back_left_motor.setPower(-(Px - Protate));
                boat.front_right_motor.setPower(-(Px + Protate));
                boat.front_left_motor.setPower(-(Py - Protate));
                boat.back_right_motor.setPower(-(Py + Protate));
            }
        }

        //telemetry.addData(">", "Protate: " + Protate);
        //telemetry.addData(">", "theta: " + theta);
        //telemetry.addData(">", "Px: " + Px);
        //telemetry.addData(">", "Py: " + Py);
        //telemetry.addData(">", "gyroAngle" + gyroAngle);
        //telemetry.addData(">", "leftsticky" + gamepad1.left_stick_y);
    }
    public void rotateToAngle() { //updated move to angle
        double desiredRunTime = 5000;
        double incorrectHeading = getHeading();
        if (gamepad1.b) {
            moveAngle = 9001;
        } else if (gamepad1.dpad_down) {
            moveAngle = 180;
            currentTimeForAngle = runtime.milliseconds();
        } else if (gamepad1.dpad_left) {
            moveAngle = 90;
            currentTimeForAngle = runtime.milliseconds();
        } else if (gamepad1.dpad_right) {
            moveAngle = -90;
            currentTimeForAngle = runtime.milliseconds();
        } else if (gamepad1.dpad_up) {
            moveAngle = 0;
            currentTimeForAngle = runtime.milliseconds();
        }
        if (moveAngle != 9001 && runtime.milliseconds() > currentTimeForAngle && runtime.milliseconds() < currentTimeForAngle + desiredRunTime) {
            //dostuff
        }
    }
    public void moveToAngle() { //Relative to start of teleop (unless reset)
        double desiredRunTime = 5000;
        double power = 0;

        int notRealHeading = Math.round(getHeading()); //FIX ALL THESE BADDIES
        if (notRealHeading < 0) {
            notRealHeading = 360 + notRealHeading;
        }
        if (gamepad1.b) {
            moveAngle = 9001;
        } else if (gamepad1.dpad_down) {
            moveAngle = 180;
            currentTimeForAngle = runtime.milliseconds();
        } else if (gamepad1.dpad_left) {
            moveAngle = 90;
            currentTimeForAngle = runtime.milliseconds();
        } else if (gamepad1.dpad_right) {
            moveAngle = 270;
            currentTimeForAngle = runtime.milliseconds();
        } else if (gamepad1.dpad_up) {
            moveAngle = 0;
            currentTimeForAngle = runtime.milliseconds();
        }
        if (moveAngle != 9001 && runtime.milliseconds() > currentTimeForAngle && runtime.milliseconds() < currentTimeForAngle + desiredRunTime) {
            if (notRealHeading < moveAngle) {
                Protate = maxRotate * 3 / 2;
                Px = Px * 2 / 3;
                Py = Py * 2 / 3;
                if (notRealHeading < moveAngle && notRealHeading + 30 >= moveAngle) {
                    Protate = maxRotate / 3;
                }
            }
            if (notRealHeading > moveAngle) {
                Protate = -maxRotate * 3 / 2;
                Px = Px * 2 / 3;
                Py = Py * 2 / 3;
                if (notRealHeading > moveAngle && notRealHeading - 30 <= moveAngle) {
                    Protate = -maxRotate / 3;
                }
            }
            if (moveAngle == 0) {
                if (notRealHeading > 180) {
                    notRealHeading = notRealHeading - 360;
                }
                if (notRealHeading < moveAngle) {
                    Protate = maxRotate * 3 / 2;
                    Px = Px * 2 / 3;
                    Py = Py * 2 / 3;
                    if (notRealHeading < moveAngle && notRealHeading + 30 >= moveAngle) {
                        Protate = maxRotate / 3;
                    }
                }
                if (notRealHeading > moveAngle) {
                    Protate = -maxRotate * 3 / 2;
                    Px = Px * 2 / 3;
                    Py = Py * 2 / 3;
                    if (notRealHeading > moveAngle && notRealHeading - 30 <= moveAngle) {
                        Protate = -maxRotate / 3;
                    }
                }
            }
            if (notRealHeading == moveAngle) {
                Protate = 0;
                moveAngle = 9001;
            }
            telemetry.addData(">", "gyroInput: " + notRealHeading);
        }
    }
    public void setCurrentHeadingToZero(){
        if (applePie == true) {
            rotate_angle = rotate_angle + getHeading();
            applePie = false;
        }
    }
    public float getHeading(){
        Orientation angles = boat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;
        float fakeHeading = heading - rotate_angle;
        return fakeHeading;
    }

    //////////////////////////////////
    //////////////////////////////////
    /* GLYPH FLIP RELATED FUNCTIONS */
    //////////////////////////////////
    //////////////////////////////////
    public void glyph_flip(){ //Includes toggle and calling all the glyph_flip functions
        double desiredTime = 1000;
        if(gamepad2.y && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_high();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_y_toggle = toggle(gamepad2_y_toggle);
            gamepad2_a_toggle = false;
            gamepad2_b_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad2.a && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_low();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_a_toggle = toggle(gamepad2_a_toggle);
            gamepad2_y_toggle = false;
            gamepad2_b_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad2.b && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_hover();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_b_toggle = toggle(gamepad2_b_toggle);
            gamepad2_y_toggle = false;
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad2.x && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_stomp();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_x_toggle = toggle(gamepad2_x_toggle);
            gamepad2_y_toggle = false;
            gamepad2_a_toggle = false;
            gamepad2_b_toggle = false;
        }
        telemetry.addData(">", "Arm Potentiometer: " + boat.armPotentiometer.getVoltage());
        //glyph_flipper_threshold_check();
        //   glyph_flip_high();
        //   glyph_flip_low();
        shovelFlip();
//        glyph_flip_hover();
//        glyph_flip_stomp();
        //glyph_grab();

//        glyph_flick();
        // drive();
    }
// FLIPPER FIRST SERVO SECOND
//    glyph_flipper_runToPosition(desired_glyph_position);
//    if(Math.abs(potValue - desired_glyph_position) <= .01){
//        glyph_aligner_servo_runToPosition(desired_servo_position);
//    }


//SERVO FIRST FLIPPER SECOND
//    glyph_aligner_servo_runToPosition(desired_servo_position);
//    if(Math.abs(servoValue - desired_servo_position) <= .015){
//        glyph_flipper_runToPosition(desired_glyph_position);
//    }



    public void glyph_grab() { //works
        double desiredTime = 1000;
        if((gamepad2.right_stick_y > 0.5 || gamepad2.right_stick_y < -0.5) && runtime.milliseconds() > (glyph_grab_last_press_time + desiredTime)){
            if(glyph_grabber_openness == true){
                //boat.glyph_grabber.setPosition(0);
                glyph_grabber_openness = false;
            }
            else if(glyph_grabber_openness == false){
                //boat.glyph_grabber.setPosition(.75);
                glyph_grabber_openness = true;
            }
            glyph_grab_last_press_time = runtime.milliseconds();
        }
    }

//.371

    public void glyph_flick(){
        if (gamepad2.right_trigger > .5){
            double potValue = boat.armPotentiometer.getVoltage();
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
            gamepad2_y_toggle = false;
            gamepad2_b_toggle = false;

            //glyph_aligner_servo_runToPosition(desired_servo_position);
            boat.glyph_aligner.setPosition(120.0/180.0);






        }
    }

    public void glyph_flip_high(){ //Moves motor then servo
        if (gamepad2_y_toggle == true ) {
            double potValue = boat.armPotentiometer.getVoltage();
            double servoValue = boat.glyph_aligner.getPosition();
            double desired_glyph_position = .854;
            double desired_servo_position = 160.0 / 180.0; //used to be 155
            glyph_flipper_runToPosition(.313, 1.0);//old glyph flip high thing



            telemetry.addData(">", "Glyph Flip High");
        }
    }
    public void glyph_flip_low(){ //Moves motor then servo
        if (gamepad2_a_toggle == true) {
            double potValue = boat.armPotentiometer.getVoltage();
            double servoValue = boat.glyph_aligner.getPosition();
            double desired_glyph_position = 1.869; //2.069
            double desired_servo_position = 56.0/180.0;
            // glyph_flipper_runToPosition(1.2);
            glyph_flipper_runToPosition(2.3, 1.0);//old glyph flip high thing

            //glyph_flipper_runToPosition(desired_glyph_position, 0.6);
//            if (Math.abs(potValue - desired_glyph_position) <= .5) {
//                glyph_aligner_servo_runToPosition(desired_servo_position, .1);
//            }
//
            if (Math.abs(potValue - desired_glyph_position) <= .3) {
                boat.glyph_aligner.setPosition(desired_servo_position);
            }



            telemetry.addData(">", "Glyph Flip Low");
        }
    }
    public void glyph_flip_hover(){ //Moves servo then motor
        if(gamepad2_b_toggle == true){
            double servoValue = boat.glyph_aligner.getPosition();
            double potValue = boat.armPotentiometer.getVoltage();
            double desired_glyph_position = .3;
            double desired_servo_position = 1.0 / 180.0;
            double desiredTime = 1000;
            glyph_flipper_runToPosition(desired_glyph_position, 0.7);

            if (Math.abs(potValue - desired_glyph_position) <= .2) {
                boat.glyph_aligner.setPosition(desired_servo_position);
            }
            else if (Math.abs(potValue - desired_glyph_position) <= .5) {
                glyph_aligner_servo_runToPosition(desired_servo_position, .1);
            }

            telemetry.addData(">", "Glyph Flip Hover");
        }
    }
    public void glyph_flip_stomp() { //Moves motor then servo
        if (gamepad2_x_toggle == true) {
            double servoValue = boat.glyph_aligner.getPosition();
            double potValue = boat.armPotentiometer.getVoltage();
            double desired_glyph_position = .003;
            double desired_servo_position = 10.0 / 180.0;
            double desiredTime = 1000;

            boat.glyph_aligner.setPosition(desired_servo_position);
            if (Math.abs(servoValue - desired_servo_position) < .02) {
                glyph_flipper_runToPosition(desired_glyph_position, 1);
            }
//            if (Math.abs(potValue - desired_glyph_position) <= .5 && runtime.milliseconds() > (glyph_flip_stomp_last_time + desiredTime)) {
//                glyph_aligner_servo_runToPosition(desired_servo_position, .25);
//            }
//            else if (Math.abs(potValue - desired_glyph_position) <= .1) {
//                boat.glyph_aligner.setPosition(desired_servo_position);
//            }
//
//            else if(Math.abs(potValue - desired_glyph_position) > .5){
//                glyph_flip_stomp_last_time = runtime.milliseconds();
//            }
//
//
//            glyph_flipper_runToPosition(desired_glyph_position, 1);
//            if(Math.abs(potValue - desired_glyph_position) <= .1){
//                boat.glyph_aligner.setPosition(desired_servo_position);
//            }
//        }
            telemetry.addData(">", "Glyph Flip Stomp");
        }
    }

    //.854, 2.096., 0.096
    public void glyph_flipper_runToPosition(double desiredPosition, double coefficient){ //Replace later with Josh's PID loop
        double desiredTime = 20;
        double potValue = boat.armPotentiometer.getVoltage();
        double kp = 1.0;   //.5              // proportional konstant
        double ki = 0.345;    //.345         //konstant of integration
        double kd = 0.2;     //.2 //1.7 and 3.0?      //konstant of derivation
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
        if(Math.abs(desiredPosition - potValue) > .001 && opModeIsActive()){

            drive();
            glyph_grab();//change this error bound runtime.milliseconds() > (glyph_flipper_last_time + desiredTime)
            if(error < integralActiveZone && error != 0){// total error only accumulates where        /                                                                                            //there is error, and when the error is
                //within the integral active zone
                //DON'T Check for integral until you're within a certain limit
                errorT += error;// adds error to the total each time through the loop
            }
            else{
                errorT = 0;// if error = zero or error is not withing the active zone, total       /                                                    //error is set to zero
            }
            if(errorT > 50 / ki){ //caps total error at 50
                errorT = 50 / ki;
            }
            if(error == 0){
                derivative = 0; // if error is zero derivative term is zero
            }
            proportion = error * kp; // sets proportion term
            integral = errorT * ki;// sets integral term
            derivative = (error - lastErrorGlyph) * kd;// sets derivative term
            lastErrorGlyph = error; // sets the last error to current error so we can use it in the next loop
            current = proportion + integral + derivative;// sets value current as total of all terms

            boat.glyph_flipper.setPower(current*coefficient);

            sleep(20);
            //glyph_flipper_last_time = runtime.milliseconds();// waits so we dont hog all our CPU power or cause loop instability
        }
        else {
            telemetry.addData(">", "Glyph_flipper PID complete.");
        }
    }


    public void servoHorizontal(double startValue){ // this function is supposed to keep the plate
        // attached to the glyphs horizontal even as the glyph_flipper moves by
        //errorServo = startValue;// THIS IS FOR STARTING POSITION // used to say start at .26
        errorServo = errorServo + (boat.armPotentiometer.getVoltage()-firstError)/1.7; // need to convert values later

        if (Math.abs(boat.armPotentiometer.getVoltage()-firstError) > .01) {
            boat.glyph_aligner.setPosition(errorServo);
        }
        //increment the glyph aligner servo by the fraction of error that the glyph arm has moved
        firstError = boat.armPotentiometer.getVoltage();
        //sleep(10);
        if (errorServo < 0.001){
            boat.glyph_aligner.setPosition(.004);
        }
// store the last potentiometer value    //80.83/230                                    // changing the servo angle according to the armPotentiometer error bound
        // function will always be declared in a toggle such that whenever
    }


    public void shovelFlip() {

        if (gamepad2_a_toggle == true) {
            //toggle
            glyph_flipper_runToPosition(2.5, .5);//old glyph flip high thing
            if (boat.armPotentiometer.getVoltage() > .7) {
                boat.glyph_aligner.setPosition(.81);
            }
            errorServo = .835;
            firstError = boat.armPotentiometer.getVoltage();

        }  if (gamepad2_y_toggle == true) {



//            if (boat.armPotentiometer.getVoltage() < .5){
//                //boat.glyph_aligner.setPosition(.23); // need to change this value
//                //glyph_flipper_runToPosition(., 1.0);//old glyph flip high thing
//
//            }
//             if (boat.armPotentiometer.getVoltage() > 2.4){
//                boat.glyph_aligner.setPosition(.835); //8+
//                sleep(50);
//                firstError = boat.armPotentiometer.getVoltage();
//            }
//            else if (boat.armPotentiometer.getVoltage() > .39){
            //servoHorizontal(.26);
            if(boat.armPotentiometer.getVoltage() > 2.45) {
                boat.glyph_aligner.setPosition(.87);
            }
            else{
                servoSlow(.01);
            }
            //glyph_flipper_runToPosition(.38, 1.0);//old glyph flip high thing
//            }
//            else {
//                //boat.glyph_aligner.setPosition();
//            }

            glyph_flipper_runToPosition(.5, 1.0);//old glyph flip high thing

        }
        if (gamepad2.b == true){
            boat.glyph_aligner.setPosition(50.0/180.0);
        }

        if (gamepad2.x == true){
            boat.glyph_aligner.setPosition(.87);

        }

//        else{
//            errorServo = .835;
//             firstError = boat.armPotentiometer.getVoltage();

        //boat.glyph_flipper.setPower(0);

        //}


    }



    public double servoAdjustAligner(double servo_angle) {

        if (gamepad1.right_trigger>.3) { //CONFLICTING CONTROL KEYS
            servo_angle = boat.glyph_aligner.getPosition() - 1.0 / 180.0;
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.glyph_aligner.setPosition(servo_angle);
        }

        if (gamepad1.left_trigger>.3) {
            servo_angle = boat.glyph_aligner.getPosition() + 1.0 / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.glyph_aligner.setPosition(servo_angle);
        }

        telemetry.addData(">", "Current Angle: " + servo_angle*180);
        return servo_angle;
    }

    public double servoAdjustRelicArm(double servo_angle) {

        if (gamepad1.right_trigger>.3 && !gamepad1.right_bumper) { //CONFLICTING CONTROL KEYS
            servo_angle = boat.relic_flop.getPosition() - 3.0 / 180.0;
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.relic_flop.setPosition(servo_angle);
        }

        if (gamepad1.left_trigger>.3 && !gamepad1.right_bumper) {
            servo_angle = boat.relic_flop.getPosition() + 3.0 / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.relic_flop.setPosition(servo_angle);
        }

        telemetry.addData(">", "Current Angle: " + servo_angle);
        return servo_angle;
    }

    public double servoSlow(double shovelServo) {
        //double servo_angle = .84;
        if ((boat.glyph_aligner.getPosition() - shovelServo) < - 8.0 / 180.0) { //CONFLICTING CONTROL KEYS
            servo_angle = boat.glyph_aligner.getPosition() + 7.2 / 180.0;
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.glyph_aligner.setPosition(servo_angle);
        }

        else  if ((boat.glyph_aligner.getPosition() - shovelServo) >  8.0 / 180.0)  {
            servo_angle = boat.glyph_aligner.getPosition() - 7.2 / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.glyph_aligner.setPosition(servo_angle);
        }


        telemetry.addData(">", "Current Angle: " + servo_angle*180);
        return servo_angle;
    }
    public void servoSlowNoIF(double shovelServo) {
        double servo_angle;
        if ((boat.glyph_aligner.getPosition() - shovelServo) < - 8.0 / 180.0) { //CONFLICTING CONTROL KEYS
            servo_angle = boat.glyph_aligner.getPosition() + 7.2 / 180.0;
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.glyph_aligner.setPosition(servo_angle);
        }

        if ((boat.glyph_aligner.getPosition() - shovelServo) >  8.0 / 180.0)  {
            servo_angle = boat.glyph_aligner.getPosition() - 7.2 / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.glyph_aligner.setPosition(servo_angle);
        }


    }

    public void glyph_aligner_servo_runToPosition(double desiredPosition, double increment){
        double desiredTime = 1;
        double servoValue = boat.glyph_aligner.getPosition();
        if(Math.abs(servoValue - desiredPosition) > .05 ) {
            servoValue = boat.glyph_aligner.getPosition();
            //drive();
            //glyph_grab();
            // && runtime.milliseconds() > (servo_adjust_last_time + desiredTime) && runtime.milliseconds() > (servo_adjust_last_time + desiredTime)
            //servo_adjust_last_time = runtime.milliseconds();
            if (servoValue < desiredPosition) {
                boat.glyph_aligner.setPosition(servoValue + increment);
            } else if (servoValue > desiredPosition) {
                boat.glyph_aligner.setPosition(servoValue - increment);
            }
            sleep(5);
        }
        else{
            //  drive();
            //  glyph_grab();
        }
    }


    public void glyph_flipper_threshold_check(){ //Checks to make sure glyph_flipper is within bounds
        double potValue = boat.armPotentiometer.getVoltage();
        if (potValue < 0.001 || potValue > 3.3){
            boat.glyph_flipper.setPower(0);
            telemetry.addData(">", "*WARNING* GLYPH_FLIPPER MOTOR OUTSIDE OF BOUNDS. KILLING GLYPH_FLIPPER MOTOR. *WARNING*");
        }
    }
    public void glyph_flipper_adjust(){
        if(gamepad1.dpad_up){
            boat.glyph_flipper.setPower(0.3);
        }
        if(gamepad1.dpad_down){
            boat.glyph_flipper.setPower(-0.5);
        }
        else {
            boat.glyph_flipper.setPower(0);
        }
    }

    //////////////////////////
    //////////////////////////
    /* INTAKE RELATED STUFF */
    //////////////////////////
    //////////////////////////

    public void intake(){
        intakeSimple();
//       intake_middle();
//        intake_low();
//        intakeAuto();
        Conveyor();
        //outtake();

    }


    public void Conveyor (){ // works
        if (gamepad2.right_trigger>0){
            boat.conveyor.setPower(-1);
        }
        else if ( gamepad2.left_trigger>0){
            boat.conveyor.setPower(1);
        }
        else{
            boat.conveyor.setPower(0);
        }

    }

    public void intakeSimple(){
        if (gamepad2.dpad_down){ //intake
            boat.right_intake.setPower(-1);
            boat.left_intake.setPower(1);
            boat.conveyor.setPower(-1);
        }
        else if(gamepad2.dpad_up){
            boat.right_intake.setPower(1);
            boat.left_intake.setPower(-1);
        }
        else if (gamepad2.right_trigger>0){
            boat.conveyor.setPower(-1);
        }
        else if ( gamepad2.left_trigger>0){
            boat.conveyor.setPower(1);
        }
        else{
            boat.right_intake.setPower(0);
            boat.left_intake.setPower(0);
            boat.conveyor.setPower(0);

        }
    }



    /////////////////////////////
    /////////////////////////////
    /* MISCELLANEOUS FUNCTIONS */
    /////////////////////////////
    /////////////////////////////

    public void relicGrab(){

        if (gamepad1.right_trigger>.3 && gamepad1.right_bumper)  { //CONFLICTING CONTROL KEYS
            servo_angle2 = boat.relic_noose.getPosition() - 3.0 / 180.0;
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.relic_noose.setPosition(servo_angle2);
        }

        if (gamepad1.left_trigger>.3 && gamepad1.right_bumper) {
            servo_angle2 = boat.relic_noose.getPosition() + 3.0 / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.relic_noose.setPosition(servo_angle2);
        }
    }

    public void testMotor() {
        double potValue = boat.armPotentiometer.getVoltage();

        if (gamepad1.dpad_up) {
            boat.glyph_flipper.setPower(.4); //decreases pot //opposite for glyph flipper, same for intake aligner (positive power intake leads to decrease in position)
        } else if (gamepad1.dpad_down) {
            boat.glyph_flipper.setPower(-.4); //increases pot

        } else {
            boat.glyph_flipper.setPower(0);



//            if (gamepad1.right_bumper) {
//                boat.glyph_flipper.setPower(.3); //decreases pot //opposite for glyph flipper, same for intake aligner (positive power intake leads to decrease in position)
//            } else if (gamepad1.left_bumper) {
//                boat.glyph_flipper.setPower(-.3); //increases pot
//
//            } else {
//                boat.glyph_flipper.setPower(0);
//
//            }
        }
    }

    public void getPotValues(){
        double value = boat.armPotentiometer.getVoltage();
        //double intakevalue = boat.intakePotentiometer.getVoltage();// 0 - 3.34
        telemetry.addData("Arm Pot",  value);
        // telemetry.addData("intake Pot", intakevalue);
    }

    public boolean toggle(boolean variable){ //Takes a boolean variable then swaps its trueness
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }

}
