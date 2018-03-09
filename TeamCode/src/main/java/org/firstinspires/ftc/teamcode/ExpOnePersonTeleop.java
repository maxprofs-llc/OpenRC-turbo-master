package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="ExpPersonTeleop", group="Pushbot")
public class ExpOnePersonTeleop extends LinearOpMode {
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
    Boolean killUrself = false;
    Boolean doKillUrself = true;
//:)
    /////////////////////////////
   /* DRIVE RELATED VARIABLES */
    /////////////////////////////


    boolean switchServo = false;
    boolean pButtonSwitch = false;
    boolean cButtonSwitch = false;
    static final double SERVO_SWITCH_CLOSE = .35;//165.0/180.0;
    static final double SERVO_SWITCH_OPEN = 0.0/180.0;


    boolean switchServo2 = false;
    boolean pButtonSwitch2 = false;
    boolean cButtonSwitch2 = false;
    static final double SERVO_SWITCH_CLOSE2 = .65;
    static final double SERVO_SWITCH_OPEN2 = .39;
    double wait = 0;
    float stick_x = 0;
    float stick_y = 0;
    double gyroAngle = 0;
    float rotate_angle = 0;
    double theta = 0;
    double Px = 0;
    double Py = 0;
    double threshold = 0.05;
    double Protate = 0;
    double maxRotate = 1.25;
    int moveAngle = 9001;
    double currentTimeForAngle = 0;
    double servo_angle;
    boolean applePie = true;
    double lastErrorGlyph = 0;
    boolean Shovelness = false;
    double coefficient = 1.0;
    double ProtateMultiplier = 1.0;
    boolean lowhigh = false;

    //////////////////////////////////
   /* GLYPH FLIP RELATED VARIABLES */
    //////////////////////////////////

    double switchangle = .52;
    double switchangle2 = .39; //for grabber
    double glyph_flip_last_press_time = 0;
    double glyph_grab_last_press_time2 = 0;
    double servo_adjust_last_time = 0;
    double glyph_flipper_last_time = 0;
    double glyph_flip_stomp_last_time = 0;
    Boolean glyph_grabber_openness = false;
    double glyph_grab_last_press_time = 0;
    double errorServo = .835; //could be .26
    double firstError;
    double shovel_time = 0;

    /////////////////////////////////
   /* RELIC RELATED VARIABLES */
    //////////////////////////////////

    boolean relicNooseGrabbed = true;
    boolean relicFlopDown = true;

    @Override
    public void runOpMode() {
        boat.init(hardwareMap);
        //boat.glyph_aligner.setPosition(boat.glyph_aligner.getPosition());
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("NO", "Nathan Boat 1.0 without crossbow is ready to be sailed!");
        telemetry.update();

        while(!opModeIsActive()){
            //boat.jewel_arm.setPosition(56.0/180.0);
            boat.relic_flop.setPosition(.85);// keep it out of the way of the shovel
            boat.glyph_aligner.setPosition(.32);
            boat.jewel_arm.setPosition(0); //should be 0
            boat.glyph_grabber.setPosition(.65);

        }
        while(opModeIsActive()){
            //boat.jewel_arm.setPosition(0);
            if (killUrself == false) {
                boat.winch.setPower(-.005);
            } else {
                boat.winch.setPower(0);
            }

            //Relic();
            jewel_arm();
            telemetry.addData(">", "Current Angle: " + boat.glyph_aligner.getPosition());
            telemetry.addData("Distance (mm)",
                    String.format( "%.02f", boat.distance_sensor.getDistance(DistanceUnit.MM)));
            intake(); // this encompasses all intake functions necessary
            getPotValues();
            glyph_flip();
            drive();
            Relic();
            killUrself();
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

        if (gamepad1.start && gamepad1.right_trigger > .2) {
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
        if (gamepad1.right_trigger < 0.5 && gamepad1.left_trigger < .2) {
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
            coefficient = 1.0;
        } else if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger < .2) {
            //Runs Px Py independent of gyro (old drive) -> DOUBLE SPEED GYRO
            coefficient = 1.5;
            stick_x = stick_x * 2;
            stick_y = stick_y * 2;
//            if (gamepad1.left_bumper) {
//                stick_x = stick_x / 3;
//                stick_y = stick_y / 3;
//            }
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));

            // theta = (Math.atan2(stick_y, stick_x)) - 0; //burt is an idiot
            // Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            // Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
        }
        else if (gamepad1.left_trigger >.2&& boat.armPotentiometer.getVoltage()<1.6){
            stick_x = stick_x * 1/2;
            stick_y = stick_y * 1/2;
//            if (gamepad1.left_bumper) {
//                stick_x = stick_x / 3;
//                stick_y = stick_y / 3;
//            }
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
        }
        //ROTATION
        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            if (Px == 0 && Py == 0) {
                Protate = ProtateMultiplier*gamepad1.right_stick_x * .4;
            } else {
                Protate = ProtateMultiplier*gamepad1.right_stick_x * 3 / 2 * 0.35; //* 3 / 2 * 0.42
                Px = Px / 3 * 2;
                Py = Py / 3 * 2;
            }
        } else {
            Protate = 0;
        }
        //SPINNING LEFT IS NEGATIVE POWER

        rotateToAngle();

        // cardinal directions
        if (gamepad1.start) {
            if (gamepad1.dpad_left) {
                Protate = -0.15;
            } else if (gamepad1.dpad_right) {
                Protate = .15;
            }
        }
        if (gamepad1.left_trigger>.2&& boat.armPotentiometer.getVoltage()<1.6) {
            ProtateMultiplier = .3;
        } else {

            ProtateMultiplier = 1.0;
        }


        if (gamepad1.dpad_down && !gamepad1.right_bumper && !gamepad1.right_bumper) {
            boat.back_left_motor.setPower(coefficient*-.5 + Protate);
            boat.front_right_motor.setPower(coefficient*-.5 - Protate);
            boat.front_left_motor.setPower(coefficient*-.5 + Protate);
            boat.back_right_motor.setPower(coefficient*-.5 - Protate);
            boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.dpad_left && !gamepad1.right_bumper && !gamepad1.right_bumper) {
            boat.back_left_motor.setPower(coefficient*.3 + Protate);
            boat.front_right_motor.setPower(coefficient*.3 - Protate);
            boat.front_left_motor.setPower(coefficient*-.3 + Protate);
            boat.back_right_motor.setPower(coefficient*-.3 - Protate);
        } else if (gamepad1.dpad_right && !gamepad1.right_bumper && !gamepad1.right_bumper) {
            boat.back_left_motor.setPower(coefficient*-.3 + Protate);
            boat.front_right_motor.setPower(coefficient*-.3 - Protate);
            boat.front_left_motor.setPower(coefficient*.3 + Protate);
            boat.back_right_motor.setPower(coefficient*.3 - Protate);
        } else if (gamepad1.dpad_up && !gamepad1.right_bumper && !gamepad1.right_bumper) {
            boat.back_left_motor.setPower(coefficient*.6 + Protate);
            boat.front_right_motor.setPower(coefficient*.6 - Protate);
            boat.front_left_motor.setPower(coefficient*.6 + Protate);
            boat.back_right_motor.setPower(coefficient*.6 - Protate);
            boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            boat.back_left_motor.setPower(-(Px - Protate));
            boat.front_right_motor.setPower(-(Px + Protate));
            boat.front_left_motor.setPower(-(Py - Protate));
            boat.back_right_motor.setPower(-(Py + Protate));
            boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //telemetry.addData(">", "Protate: " + Protate);
        //telemetry.addData(">", "theta: " + theta);
        //telemetry.addData(">", "Px: " + Px);
        //telemetry.addData(">", "Py: " + Py);
        //telemetry.addData(">", "gyroAngle" + gyroAngle);
        //telemetry.addData(">", "leftsticky" + gamepad1.left_stick_y);
    }
    public void rotateToAngle() { //updated move to angle
        int threshold = 1;
        double desiredRunTime = 5000;
        double incorrectHeading = getHeading();
//        if (gamepad1.a) {
//            moveAngle = 180;
//            currentTimeForAngle = runtime.milliseconds();
//            // } else if (gamepad1.x) {
//            // moveAngle = 90;
//            //    currentTimeForAngle = runtime.milliseconds();
//            //  } else if (gamepad1.b) {
//            //    moveAngle = -90;
//            //  currentTimeForAngle = runtime.milliseconds();
//        }
        if (gamepad1.left_bumper && gamepad1.left_trigger<.2) {
            moveAngle = 0;
            currentTimeForAngle = runtime.milliseconds();
        }
        if (moveAngle != 9001 && runtime.milliseconds() > currentTimeForAngle && runtime.milliseconds() < currentTimeForAngle + desiredRunTime) {
            if (moveAngle == 0) {
                if (getHeading() <= 0 - threshold) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, getHeading());
                } else if (getHeading() > 0 + threshold) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, getHeading());
                } else {
                    moveAngle = 9001;
                }
            }
            if (moveAngle == -90) {
                if (-90 - threshold < getHeading() && getHeading() < -90 + threshold) {
                    moveAngle = 9001;
                } else if (Math.abs(getHeading()) >= 90) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, getHeading());
                } else if (Math.abs(getHeading()) < 90) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, getHeading());
                }
            }
            if (moveAngle == 90) {
                if (90 - threshold < getHeading() && getHeading() < 90 + threshold) {
                    moveAngle = 9001;
                } else if (Math.abs(getHeading()) <= 90) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, getHeading());
                } else if (Math.abs(getHeading()) > 90) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, getHeading());
                }
            }
            if (moveAngle == 180) {
                if (180 - threshold < getHeading() || getHeading() < -180 + threshold) {
                    moveAngle = 9001;
                } else if (getHeading() >= 0) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, getHeading());
                } else if (getHeading() < 0) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, getHeading());
                }
            }
        }
    }
    public double scaleProtate(int moveAngle, float fakeHeading) {
        double ProtatePower;
        int angleMove = moveAngle;
        float angleDist;
        float incorrectHeading = fakeHeading;
        angleDist =  Math.abs(angleMove - incorrectHeading);
        if (angleDist > 180) {
            angleDist = 360 - angleDist;
        }
        ProtatePower = (1.0*((maxRotate * (angleDist / 180)) + 0.05));
        return ProtatePower;
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
        if (fakeHeading < -180) {
            fakeHeading = fakeHeading + 360;
        } else if (fakeHeading > 180) {
            fakeHeading = fakeHeading - 360;
        }
        return fakeHeading;
    }

    //////////////////////////////////
    //////////////////////////////////
   /* GLYPH FLIP RELATED FUNCTIONS */
    //////////////////////////////////
    //////////////////////////////////
    public void glyph_flip(){ //Includes toggle and calling all the glyph_flip functions
        double desiredTime = 1000;
        if(gamepad1.right_bumper && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_high();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_y_toggle = toggle(gamepad2_y_toggle);
            gamepad2_a_toggle = false;
            gamepad2_b_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad1.a && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_low();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_a_toggle = toggle(gamepad2_a_toggle);
            gamepad2_y_toggle = false;
            gamepad2_b_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad1.b && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_hover();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_b_toggle = toggle(gamepad2_b_toggle);
            gamepad2_y_toggle = false;
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad1.x && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_stomp();
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




    //.854, 2.096., 0.096

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
            if (Math.abs(desiredPosition - potValue) > .001 && opModeIsActive()) {

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
            } else {
                telemetry.addData(">", "Glyph_flipper PID complete.");
            }
        }
    }
    //}


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

            if (Math.abs(desiredPosition - potValue) > .001 && opModeIsActive()) {

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
            } else {
                telemetry.addData(">", "Glyph_flipper PID complete.");
            }
        }
    }

    public void shovelFlip() {
        pButtonSwitch2 = cButtonSwitch2;
        cButtonSwitch2 = gamepad1.right_bumper;
//        if(gamepad1.right_bumper) {
//            gamepad2_a_toggle = false;
//            gamepad2_x_toggle = false;
//            gamepad2_y_toggle = false;
//
//            if (cButtonSwitch2 && !pButtonSwitch2) {
//                switchServo2 = switchServo2 ? false : true;
//            }
//
//            if (switchServo2) {
//                boat.glyph_grabber.setPosition(SERVO_SWITCH_OPEN2);
//            } else if (!switchServo2) {
//                boat.glyph_grabber.setPosition(SERVO_SWITCH_CLOSE2);
//
//            }
//        }
        double desiredTime = 300;

        if (gamepad1.y && runtime.milliseconds() > (glyph_grab_last_press_time2 + desiredTime)){ //glyph flicking
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
            gamepad2_y_toggle = false;

            if (switchangle2 == .39 && Shovelness== true){
                boat.glyph_grabber.setPosition(.65);
                switchangle2 = .65;
            }
            else if (switchangle2 == .39 && Shovelness == false){
                boat.glyph_grabber.setPosition(.12);
                switchangle2 = .65;
            }
            else if (switchangle2 == .65 && Shovelness == false){
                boat.glyph_grabber.setPosition(.36); //.28 3/6
                switchangle2 = .39;
            }
            else if (switchangle2 == .65 && Shovelness == true){
                boat.glyph_grabber.setPosition(.4);
                switchangle2 = .39;
            }
            glyph_grab_last_press_time2 = runtime.milliseconds();

            if (lowhigh == true) {
                if (boat.armPotentiometer.getVoltage() < .55) {
                    stallShovel(.38);
                }
            }
            else{
                if (boat.armPotentiometer.getVoltage() < .55) {
                    stallShovel(.23);
                }
            }
        }




        if (gamepad2_a_toggle == true) {
            //toggle
            glyph_flipper_runToPosition(2.40, .3);//2.29 3/6/18

            if (boat.armPotentiometer.getVoltage() > 2.0) {
                boat.glyph_grabber.setPosition(.36); //.28 3/6/18
                boat.glyph_aligner.setPosition(.32); //.38 3/6/18

            }
            else{
                boat.glyph_aligner.setPosition(.5); //.38 3/6/18

            }
            switchangle2 = .39;
            Shovelness = false;


        }

        if (gamepad2_y_toggle == true) {
//            boat.left_intake.setPower(-.3);
//            boat.right_intake.setPower(-.3);
            lowhigh = true;
            if (boat.armPotentiometer.getVoltage() <.55){
                stallShovel(.39);
            }
            else if (boat.glyph_aligner.getPosition() == .7 || boat.glyph_aligner.getPosition()== .17 || boat.glyph_aligner.getPosition()== .09 || boat.glyph_aligner.getPosition()== .2) { // CHANGE THE IF STATEMENT DIM WIT
                Playglyph_flipper_runToPosition(.4, .8); //.45 3/7/18 OLD ONE FROM FEB 15 THAT WORKS //.438 SHOULD BE RIGHT 3/5
                //Playglyph_flipper_runToPosition(.25, 1.0); // OLD ONE FROM FEB 15 THAT WORKS
            }


            boat.glyph_grabber.setPosition(.65);
            switchangle2 = .65;
            switchangle= .65;
            if (boat.armPotentiometer.getVoltage()<.50){
                boat.glyph_aligner.setPosition(.09);  //.11 3/8/18
            }
            else if (boat.armPotentiometer.getVoltage()<1.7){ //1.7 3/8/18
                boat.glyph_aligner.setPosition(.17);    // CHANGE THE IF STATEMENT DIM WIT             // .07 3/6 march
            }
            else if (boat.armPotentiometer.getVoltage()>1.8){
                boat.glyph_aligner.setPosition(.70);
            }
            Shovelness = true;
//            if (boat.armPotentiometer.getVoltage()< 1.90) {
//                boat.glyph_grabber.setPosition(.65);
//                switchangle2 = .65;
//            }
//            if (boat.armPotentiometer.getVoltage()<1.7){
//                boat.glyph_aligner.setPosition(.08);                // .15, 9.0 was for hover
//            }
//            else if (boat.armPotentiometer.getVoltage()>2.0){
//                boat.glyph_aligner.setPosition(.7);         //NEED TO FIND THIS       // .15, 9.0 was for hover
//
//            }


//            if (boat.armPotentiometer.getVoltage() < 1.8){
//                boat.glyph_aligner.setPosition(.20);
//
//            }
//////            else{
//            if (boat.armPotentiometer.getVoltage() < 1.0){
//                servoSlow(0.07, 8.0); //.18
//
//            }
//            else if (boat.armPotentiometer.getVoltage() < 2.3) {
//                servoSlow(.07, 14.0); //.18
//            }


        }

        //double desiredTime = 100;

        if (gamepad1.b == true && runtime.milliseconds() > (glyph_grab_last_press_time + desiredTime)){ //glyph flicking
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
            gamepad2_y_toggle = false;
            if (Shovelness == true){
                if (lowhigh == true){
                    if (switchangle == .52){
                        boat.glyph_aligner.setPosition(.05);
                        switchangle = .65;
                    }
                    else if (switchangle == .65){
                        boat.glyph_aligner.setPosition(.23);
                        switchangle = .52;
                    }
                }
                else{
                    if (switchangle == .52){
                        boat.glyph_aligner.setPosition(.01);
                        switchangle = .65;
                    }
                    else if (switchangle == .65){
                        boat.glyph_aligner.setPosition(.09);
                        switchangle = .52;
                    }
                }

            }
            else {
                if (switchangle == .52){
                    boat.glyph_aligner.setPosition(.32);
                    switchangle = .65;
                }
                else if (switchangle == .65){
                    boat.glyph_aligner.setPosition(.5);
                    switchangle = .52;
                }
            }

            glyph_grab_last_press_time = runtime.milliseconds();
            boat.glyph_flipper.setPower(0);

        }

        if (gamepad2_x_toggle == true){
            lowhigh = false;
            Shovelness = true;
            if (boat.armPotentiometer.getVoltage() <.50){
                stallShovel(.23);
            }
            else if (boat.glyph_aligner.getPosition() == .7 || boat.glyph_aligner.getPosition()== .01) { // CHANGE THE IF STATEMENT DIM WIT
                Playglyph_flipper_runToPosition(.27, .8); //.3 3/6/18 OLD ONE FROM FEB 15 THAT WORKS //.438 SHOULD BE RIGHT 3/5
                //Playglyph_flipper_runToPosition(.25, 1.0); // OLD ONE FROM FEB 15 THAT WORKS
            }

            boat.glyph_grabber.setPosition(.65);
            switchangle2 = .65;
            if (boat.armPotentiometer.getVoltage()<1.7){
                boat.glyph_aligner.setPosition(.01);    // CHANGE THE IF STATEMENT DIM WIT             // .01 3/6 march
            }
            else if (boat.armPotentiometer.getVoltage()>1.8){
                boat.glyph_aligner.setPosition(.70);
            }


            // .15, 9.0 was for hover


            //Playglyph_flipper_runToPosition(.20, .9); //.44


        }

//        else{
//            errorServo = .835;
//             firstError = boat.armPotentiometer.getVoltage();

        //boat.glyph_flipper.setPower(0);

        //}


    }

    public void stallShovel( double target){
        double currentPos = boat.armPotentiometer.getVoltage();

        boat.glyph_flipper.setPower(Math.signum(currentPos - target)*((-.25/Math.sqrt(1+2000*(currentPos-target)*(currentPos-target)))+.25));

    }

    public double servoSlow(double shovelServo, double increment) {
        //double servo_angle = .84;
        servo_angle = boat.glyph_aligner.getPosition();
        if (boat.glyph_aligner.getPosition() < .12){
            boat.glyph_aligner.setPosition(.1); //used to be .3
        }
        else if ((boat.glyph_aligner.getPosition() - shovelServo) < - (increment+1.0) / 180.0) { //CONFLICTING CONTROL KEYS
            servo_angle = boat.glyph_aligner.getPosition() + increment / 180.0; // used to be 10
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.glyph_aligner.setPosition(servo_angle);
        }

        else  if ((boat.glyph_aligner.getPosition() - shovelServo) >  (increment+1.0 )/ 180.0)  {
            servo_angle = boat.glyph_aligner.getPosition() - increment / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.glyph_aligner.setPosition(servo_angle);
        }


        telemetry.addData(">", "Current Angle: " + servo_angle*180);
        return servo_angle;
    }


    //////////////////////////
    //////////////////////////
   /* INTAKE RELATED STUFF */
    //////////////////////////
    //////////////////////////

    public void killUrself(){
        if (gamepad2.start && doKillUrself == true) {
            if (killUrself == true) {
                killUrself = false;
            } else {
                killUrself = true;
            }
            doKillUrself = false;
        } else if (!gamepad2.start) {
            doKillUrself = true;
        }
    }

    public void intake(){
        intakeSimple();
//       intake_middle();
//        intake_low();
//        intakeAuto();
        //Conveyor();
        //outtake();
        // winch();

    }

    public void Conveyor (){ // works
        if (gamepad2_y_toggle == false && killUrself == false) {
            if (gamepad2.left_trigger > 0.5) {
                boat.conveyor.setPower(0);
            } else if (gamepad2.left_bumper) {
                boat.conveyor.setPower(10);
            } else {
                boat.conveyor.setPower(-10);
            }
        } else if (killUrself == true) {
            boat.conveyor.setPower(0);
        }
    }

    public void intakeSimple() {
        if (gamepad1.left_trigger>.3 && gamepad1.left_bumper) { //intake
            boat.right_intake.setPower(1);
            boat.left_intake.setPower(1);
        }

        else if (gamepad1.left_trigger>.2 && boat.armPotentiometer.getVoltage() >.8 ) {
            boat.right_intake.setPower(-1.0);
            boat.left_intake.setPower(-1.0);
        } else {
            boat.right_intake.setPower(0);
            boat.left_intake.setPower(0);

        }

    }



    /////////////////////////////
    /////////////////////////////
   /* MISCELLANEOUS FUNCTIONS */
    /////////////////////////////
    /////////////////////////////

    public double servoAdjust(double servo_angle) {

        if (gamepad1.right_trigger>.3) { //CONFLICTING CONTROL KEYS
            servo_angle = boat.glyph_aligner.getPosition() - 3.0 / 180.0;
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.glyph_aligner.setPosition(servo_angle);
        }

        if (gamepad1.left_trigger>.3) {
            servo_angle = boat.glyph_aligner.getPosition() + 3.0 / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.glyph_aligner.setPosition(servo_angle);
        }

        telemetry.addData(">", "Current Angle: " + servo_angle*180);
        return servo_angle;
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


    public void Relic(){
        if (gamepad2.right_trigger>0.2){ //extend linear slide
            boat.relic_extender.setPower(-gamepad2.right_trigger);
        } else if (gamepad2.left_trigger>0.2){
            boat.relic_extender.setPower(gamepad2.left_trigger);
        } else if (gamepad1.right_stick_button) {
            boat.relic_extender.setPower(-.2);
        } else if (gamepad1.left_stick_button) {
            boat.relic_extender.setPower(.2);
        }  else{
            boat.relic_extender.setPower(0);
        }

//        if (gamepad1.b){ //toggle noose
//            if (relicNooseGrabbed) {
//                boat.relic_noose.setPosition(0); //grab
//                relicNooseGrabbed = false;
//            } else {
//                boat.relic_noose.setPosition(1); //let go
//                relicNooseGrabbed = true;
//            }
//        }
        if (gamepad1.x){ //toggle flop
            if (relicFlopDown) {
                boat.relic_flop.setPosition(.285); //up
                relicFlopDown = false;
            } else {
                boat.relic_flop.setPosition(0.19); //down
                relicFlopDown = true;
            }
        }

    }
    public void winch() {
        if(Math.abs(gamepad2.right_stick_y)>.2) {
            boat.winch.setPower(gamepad2.right_stick_y);
        }
    }

    public void jewel_arm() {

        //deciding the state if the switch servo is at pos 1 || 0
        pButtonSwitch = cButtonSwitch;
        cButtonSwitch = gamepad2.left_bumper;
        if(gamepad2.left_bumper && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            if (cButtonSwitch && !pButtonSwitch) {
                switchServo = switchServo ? false : true;
            }

            if (switchServo) {
                boat.jewel_arm.setPosition(SERVO_SWITCH_OPEN);
            } else if (!switchServo) {
                boat.jewel_arm.setPosition(SERVO_SWITCH_CLOSE);

            }
        }
        else if(gamepad2.right_bumper && gamepad2.dpad_up){
            boat.jewel_arm.setPosition(boat.jewel_arm.getPosition() - 3/180.0);

        }
        else if(gamepad2.right_bumper && gamepad2.dpad_down){
            boat.jewel_arm.setPosition(boat.jewel_arm.getPosition() + 3/180.0);
        }
        else{
            boat.jewel_arm.setPosition(boat.jewel_arm.getPosition());
        }
    }



    public void getPotValues(){
        double value = boat.armPotentiometer.getVoltage();
        //double intakevalue = boat.intakePotentiometer.getVoltage();// 0 - 3.34
        telemetry.addData("Arm Pot",  value);
        //telemetry.addData("intake Pot", intakevalue);
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


