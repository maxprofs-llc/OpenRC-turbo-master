package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance; Commented out doopadoo


//Most code taken from HardwarePushbot.java in the FTC samples

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class NathanPushboat
{
    //////////////////
    /* DECLARATIONS */
    //////////////////

    //(Expansion Hub Number, Number)
    //DRIVE//
    public DcMotor front_left_motor   = null; //(9, 0)
    public DcMotor front_right_motor  = null; //(6, 0)
    public DcMotor back_left_motor    = null; //(9, 1)
    public DcMotor back_right_motor   = null; //(6, 1)

    //INTAKE//
    public DcMotor right_intake = null; //(6, 2)
    public DcMotor left_intake = null; //(9, 2)
    //public DcMotor intake_aligner = null;
    // public AnalogInput intakePotentiometer = null;
    public CRServo winch = null;
    //GLYPH//
    public DcMotor glyph_flipper = null; //(9, 3)
    public Servo glyph_aligner = null; //(9, 0)
    public Servo glyph_grabber = null; //(9, 1)
    public AnalogInput armPotentiometer = null;

    //JEWEL//
    public Servo jewel_arm = null; //(6, 0)
    public DistanceSensor distance_sensor = null;
    public ColorSensor color_sensor = null;

    //RELIC//
    public DcMotor relic_extender = null;
    public Servo relic_flipper = null;
    public Servo relic_grabber = null;

    //IMU//
    BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public NathanPushboat(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////

        //DRIVE//
        front_left_motor   = hwMap.dcMotor.get("front_left_wheel");
        front_right_motor  = hwMap.dcMotor.get("front_right_wheel");
        back_left_motor    = hwMap.dcMotor.get("back_left_wheel");
        back_right_motor   = hwMap.dcMotor.get("back_right_wheel");

        //INTAKE//
        right_intake = hwMap.dcMotor.get("right_intake");
        left_intake = hwMap.dcMotor.get("left_intake");
        //intake_aligner = hwMap.dcMotor.get("intake_aligner");
        winch = hwMap.crservo.get("winch");
        //intakePotentiometer = hwMap.analogInput.get("intakePotentiometer");
        //GLYPH//
        glyph_flipper = hwMap.dcMotor.get("glyph_flipper"); //positive power increases pot position
        glyph_aligner = hwMap.servo.get("glyph_aligner");
        glyph_grabber = hwMap.servo.get("glyph_grabber");
        armPotentiometer = hwMap.analogInput.get("armPotentiometer");

        //JEWEL//
        jewel_arm = hwMap.servo.get("jewel_arm");//.2 for STRAIGHT UP, .76 JEWEL DOWN
        color_sensor = hwMap.get(ColorSensor.class, "color_sensor");
        distance_sensor = hwMap.get(DistanceSensor.class,"color_sensor");

//        //RELIC//
        relic_extender  = hwMap.dcMotor.get("relic_extender");
        relic_flipper   = hwMap.servo.get("relic_flipper");
        relic_grabber = hwMap.servo.get("relic_grabber");
        relic_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relic_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");

        //////////////////
        /* STUFFY STUFF */
        //////////////////

        /* DRIVING STUFF */
        front_left_motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);

        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /* GLYPH STUFF */
        glyph_flipper.setDirection(DcMotor.Direction.REVERSE);
        glyph_flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyph_flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* INTAKE STUFF */
        right_intake.setDirection(DcMotor.Direction.FORWARD);
        left_intake.setDirection(DcMotor.Direction.REVERSE);

        right_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        intake_aligner.setDirection(DcMotor.Direction.REVERSE);
//        intake_aligner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake_aligner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void imu(){
        /* IMU STUFF */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     */
    public void delay (double time){
        double desiredRunTime = time;
        double currentTime = runtime.milliseconds();
        while (runtime.milliseconds() > currentTime && runtime.milliseconds() < currentTime + desiredRunTime ) {

        }
    }
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
