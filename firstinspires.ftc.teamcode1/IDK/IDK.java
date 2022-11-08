package org.firstinspires.ftc.teamcode.IDK;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="IDK")
public class IDK extends LinearOpMode {
    DcMotor motorl;
    DcMotor motorr;
    DcMotor krutilka1;
    DcMotor krutilka2;

    @Override
    public void runOpMode() {
        motorl = hardwareMap.get(DcMotorEx.class, "lmotor");
        motorr = hardwareMap.get(DcMotorEx.class, "rmotor");
        motorr.setDirection(DcMotorSimple.Direction.REVERSE);
    
        
        krutilka1 = hardwareMap.dcMotor.get("hanging1");
        krutilka1.setDirection(DcMotorSimple.Direction.REVERSE);        
        krutilka2 = hardwareMap.dcMotor.get("hanging2");
        

        waitForStart();
        while (opModeIsActive()){
            motorl.setPower(-gamepad1.right_stick_y);
            motorr.setPower(-gamepad1.left_stick_y);
            
            krutilka1.setPower((gamepad1.left_bumper ? 1.0 : 0.0) - (gamepad1.left_trigger));
            krutilka2.setPower((gamepad1.right_bumper ? 1.0 : 0.0) - (gamepad1.right_trigger));
        }
    }