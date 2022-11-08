package org.firstinspires.ftc.teamcode.FGC;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Main TeleOp", group="Iterative TeleOp")
public class MainTeleOp extends OpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    
    private DcMotor intakeMotor = null;
    private DcMotor outtakeMotor = null;
    
    private DcMotor leftCascadeMotor = null;
    private DcMotor rightCascadeMotor = null;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    private double outtakePower = 0;
    private double outtakeTimer = 0;
    
    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        outtakeMotor = hardwareMap.dcMotor.get("outtakeMotor");
        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        leftCascadeMotor = hardwareMap.dcMotor.get("leftCascadeMotor");
        leftCascadeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftCascadeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftCascadeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rightCascadeMotor = hardwareMap.dcMotor.get("rightCascadeMotor");
        rightCascadeMotor.setDirection(DcMotor.Direction.FORWARD);
        rightCascadeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCascadeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
    }
    
    @Override
    public void start() {
        runtime.reset();
    }
    
    private double gamepadScaling(double x) {
        return x / 1.07 * (0.62 * Math.pow(x, 2) + 0.45);
    }
    
    @Override
    public void loop() {
        double leftPower = gamepadScaling(-gamepad1.left_stick_y);
        double rightPower = gamepadScaling(-gamepad1.right_stick_y);
        
        double intakePower = 0;
        if (gamepad1.right_trigger > 0)
            intakePower = gamepad1.right_trigger;
        else if (gamepad1.right_bumper)
            intakePower = -1;
        
        if (gamepad1.left_trigger > 0)
            outtakePower = gamepad1.left_trigger;
        else if (gamepad1.left_bumper)
            outtakePower = -1;
        else if (outtakePower != -0.3)
            outtakePower = 0;
            
        if (gamepad1.x && runtime.milliseconds() - outtakeTimer > 200) {
            outtakeTimer = runtime.milliseconds();
            if (outtakePower == -0.3)
                outtakePower = 0;
            else
                outtakePower = -0.3;
        }
        
        double cascadePower = 0;
        if (gamepad1.y)
            cascadePower = 1;
        else if (gamepad1.a)
            cascadePower = -1;
        
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        
        intakeMotor.setPower(intakePower);
        outtakeMotor.setPower(outtakePower);
        
        leftCascadeMotor.setPower(cascadePower);
        rightCascadeMotor.setPower(cascadePower);
        
        telemetry.addData("Status", "Runtime: " + runtime.seconds());
        telemetry.addData("Motors", "Left: (%.2f), Right: (%.2f)", leftPower, rightPower);
    }
    
    @Override
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        
        leftCascadeMotor.setPower(0);
        rightCascadeMotor.setPower(0);
    }