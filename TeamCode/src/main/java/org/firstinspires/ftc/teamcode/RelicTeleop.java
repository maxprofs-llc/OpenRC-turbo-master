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
    //////////////////////
    /* TOGGLE VARIABLES */
    //////////////////////


    @Override
    public void runOpMode() {
        boat.init(hardwareMap);
        boat.relic_grabber.setPosition(0);
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
            testServo(); 
            telemetry.update();//THIS GOES AT THE END
        }
    }
    public void relicSlide(){
        if(gamepad2.left_stick_y > 0.1){
            boat.relic_extender.setPower(-gamepad2.left_stick_y);
        }
        else if(gamepad2.left_stick_y < -0.1){
            boat.relic_extender.setPower(-gamepad2.left_stick_y);
        }
        else{
            boat.relic_extender.setPower(0);
        }
    }
    public void testServo(){
        if(gamepad2.dpad_up){
            boat.relic_grabber.setPosition(boat.relic_grabber.getPosition() + 1.0);
        }
        if(gamepad2.dpad_down){
            boat.relic_grabber.setPosition(boat.relic_grabber.getPosition() - 1.0);
        }
        if(gamepad2.right_stick_y > 0.1){
            boat.relic_flipper.setPosition(boat.relic_flipper.getPosition() + 1.0);
        }
        else if(gamepad2.right_stick_y < -0.1){
            boat.relic_flipper.setPosition(boat.relic_flipper.getPosition() - 1.0);
        }
        telemetry.addData("uwu", boat.relic_flipper.getPosition());
        telemetry.addData("uwu", boat.relic_grabber.getPosition());
    }
}
