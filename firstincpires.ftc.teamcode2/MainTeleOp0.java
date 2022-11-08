package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class MainTeleOp0 extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private Servo cage_motor;
    private DcMotor duck_motor;
    private DcMotor extension_motor;
    private Gyroscope imu;
    private DcMotor intake_motor;
    private DcMotor left_bottom_motor;
    private DcMotor left_front_motor;
    private DcMotor right_bottom_motor;
    private DcMotor right_front_motor;


    @Override
    public void runOpMode() {
        double x1,y1;
        double fortyFiveInRads=-Math.PI/4;
        double cosine45=Math.cos(fortyFiveInRads);
        double sine45=Math.sin(fortyFiveInRads);
        double x2,y2;
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "camera");
        cage_motor = hardwareMap.get(Servo.class, "cage_motor");
        duck_motor = hardwareMap.get(DcMotor.class, "duck_motor");
        extension_motor = hardwareMap.get(DcMotor.class, "extension_motor");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        left_bottom_motor = hardwareMap.get(DcMotor.class, "left_bottom_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_bottom_motor = hardwareMap.get(DcMotor.class, "right_bottom_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        left_bottom_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        left_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        cage_motor.setDirection(Servo.Direction.REVERSE);
        

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.dpad_up) cage_motor.setPosition(0.7);
            if(gamepad1.dpad_down) cage_motor.setPosition(0.24);
            
            if(gamepad1.b){
                intake_motor.setPower(1);
                
            }
            else{
                intake_motor.setPower(0);
            }
    
            if(gamepad1.x) intake_motor.setPower(-1);
            else intake_motor.setPower(0);
             // Setting position to 1 does affect else statement
            // else if (gamepad1.dpad_down) cage_motor.setPosition(0);
            if(gamepad1.right_bumper) duck_motor.setPower(0.65);
            else duck_motor.setPower(0);
            if(gamepad1.left_bumper) duck_motor.setPower(-0.65);
            else duck_motor.setPower(0);
            
            double extensionleft=gamepad1.left_trigger;
            double extensionright=gamepad1.right_trigger;
            if(extensionleft>0) extension_motor.setPower(-1*extensionleft*1);
            else if(extensionright>0) extension_motor.setPower(extensionright*1);
            else extension_motor.setPower(0);
            
            
            
            
            double spin=gamepad1.right_stick_x;
            if (Math.abs(spin)>0.1){
                right_front_motor.setPower(-spin);
                right_bottom_motor.setPower(-spin);
                left_front_motor.setPower(spin);
                left_bottom_motor.setPower(spin);
                
            }
            else{
            y1 = -gamepad1.left_stick_y; // Remember, this is reversed!
            x1 = -gamepad1.left_stick_x; // Counteract imperfect strafing
            y2=cosine45*y1+x1*sine45;
            x2=cosine45*x1-y1*sine45;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double frontLeftPower = y2;
            double backLeftPower = x2;
            double frontRightPower = x2;
            double backRightPower = y2;

            left_front_motor.setPower(frontLeftPower);
            left_bottom_motor.setPower(backLeftPower);
            right_front_motor.setPower(frontRightPower);
            right_bottom_motor.setPower(backRightPower);
}
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}