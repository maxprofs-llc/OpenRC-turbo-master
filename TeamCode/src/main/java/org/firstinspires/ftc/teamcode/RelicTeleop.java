package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="RelicTeleop", group="Pushbot")
public class RelicTeleop extends LinearOpMode {
    NathanPushboat boat = new NathanPushboat();
    private ElapsedTime runtime = new ElapsedTime();
    double relic_grabberAngle = 0.2;
    double relic_flipperAngle = 0.5;
    double lastFlipTime = 0;
    //////////////////////
    /* TOGGLE VARIABLES */
    //////////////////////


    @Override
    public void runOpMode() {
        boat.init(hardwareMap);
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //boat.intake_aligner.setPower(0);
        telemetry.addData("Say", "Nathan Boat 1.0 without crossbow is ready to be sailed!");
        telemetry.update();

        while(!opModeIsActive()){

        }
        while(opModeIsActive()){
            relicSlide();
            relicGrabber(); 
            relicFlipper();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    public void relicSlide(){ //Sliiiiides
        if(gamepad2.left_stick_y > 0.1){
            boat.relic_extender.setPower(gamepad2.left_stick_y);
        }
        else if(gamepad2.left_stick_y < -0.1){
            boat.relic_extender.setPower(gamepad2.left_stick_y);
        }
        else{
            boat.relic_extender.setPower(0);
        }
    }
    public void relicGrabber(){ //Opens and closes the relic grabber
        if(gamepad2.dpad_up){
            boat.relic_grabber.setPosition(0.21);
        }
        else{
            boat.relic_grabber.setPosition(0.40);
        }
    }
    public void relicFlipper(){ //Has the ability to adjust and set to specific positions
        if(gamepad2.right_stick_y > 0.1 && runtime.milliseconds() - lastFlipTime > 25){
            lastFlipTime = runtime.milliseconds();
            boat.relic_flipper.setPosition(boat.relic_flipper.getPosition() + 0.02);
        }
        else if(gamepad2.right_stick_y < -0.1 && runtime.milliseconds() - lastFlipTime > 25){
            lastFlipTime = runtime.milliseconds();
            boat.relic_flipper.setPosition(boat.relic_flipper.getPosition() - 0.02);
        }
        if(gamepad2.right_stick_x > 0.8){
            boat.relic_flipper.setPosition(0.10);
        }
        if(gamepad2.right_stick_x < -0.8){
            boat.relic_flipper.setPosition(0.86);
        }
        telemetry.addData("uwu", boat.relic_flipper.getPosition());
    }
}
