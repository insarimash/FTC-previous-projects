package org.firstinspires.ftc.teamcode;

    import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.hardware.rev.RevColorSensorV3;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    @TeleOp(name="BoltM3 manual control test")
    public class BoltM3_manual_control extends LinearOpMode {
        DcMotor motorFL, motorBL, motorFR, motorIntake, motorBR, motorLift, motorDuck;
        Servo s1;
        boolean up = false, down = false;

        /**
         * Just test code, we used the same source for mecanum wheels btw
         *
         **/

        /*Runnable runnableServo = new Runnable() {
            @Override
            public void run() {
                s1.setPosition(0);
                try {
                    Thread.sleep(800);
                } catch (Exception ex) {}
                s1.setPosition(0.965);
            }
        };
        Thread threadServo = new Thread(runnableServo);*/

        @Override
        public void runOpMode() throws InterruptedException {
            motorFL = hardwareMap.get(DcMotorEx.class, "left_front_motor");
            motorBL = hardwareMap.get(DcMotorEx.class, "left_bottom_motor");
            motorFR = hardwareMap.get(DcMotorEx.class, "right_front_motor");
            motorBR = hardwareMap.get(DcMotorEx.class, "right_bottom_motor");
            motorIntake = hardwareMap.get(DcMotorEx.class, "intake_motor");
            motorDuck = hardwareMap.dcMotor.get("duck_motor");
            motorLift = hardwareMap.get(DcMotorEx.class, "extension_motor");
            s1 = hardwareMap.get(Servo.class, "cage_motor");

            s1.setDirection(Servo.Direction.REVERSE);
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorBL.setDirection(DcMotor.Direction.REVERSE);

            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);*/
            //s1.scaleRange(0, 0.57);

            waitForStart();
            if(opModeIsActive()){
                while (opModeIsActive()){
                    double y = -gamepad1.left_stick_y;
                    double x = gamepad1.left_stick_x * 1.1;
                    double rx = gamepad1.right_stick_x;
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    /*motorFL.setPower(frontLeftPower);
                    motorBL.setPower(backLeftPower);
                    motorFR.setPower(frontRightPower);
                    motorBR.setPower(backRightPower);*/

                    telemetry.addLine("ewqew");
                    telemetry.addLine(String.valueOf(frontLeftPower));
                    telemetry.addLine(String.valueOf(frontRightPower));
                    telemetry.addLine(String.valueOf(backLeftPower));
                    telemetry.addLine(String.valueOf(backRightPower));

                    motorDuck.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

                    if(gamepad1.left_bumper){
                        motorIntake.setPower(-1);
                    } else if(gamepad1.right_bumper){
                        motorIntake.setPower(1);
                    } else{
                        motorIntake.setPower(0);
                    }

                    if (gamepad1.dpad_up) {
                        s1.setPosition(0.8);
                    } else if (gamepad1.dpad_down) {
                        s1.setPosition(0.33);
                    }


                    /*if(gamepad1.left_bumper){
                        motorIntake.setPower(-1);
                    } else if(gamepad1.right_bumper){
                        motorIntake.setPower(1);
                    } else{
                        motorIntake.setPower(0);
                    }

                    if(gamepad1.dpad_left){
                        s1.setPosition(s1.getPosition() - 0.005);
                    } else if(gamepad1.dpad_right){
                        s1.setPosition(s1.getPosition() + 0.005);
                    }
                    motorDuck.setPower(0.8 * (gamepad1.left_trigger - gamepad1.right_trigger));
                    if((gamepad1.dpad_up || up) && motorServo.getCurrentPosition() > 50){
                        motorServo.setPower(-0.8);
                        if(motorServo.getCurrentPosition() < 700){
                            s1.setPosition(0.8);
                        } else {
                            s1.setPosition(0.965);
                        }
                    } else if((gamepad1.dpad_down || down) && motorServo.getCurrentPosition() < 1080){
                        motorServo.setPower(0.8);
                        s1.setPosition(0.965);
                    } else if(motorServo.getCurrentPosition() < 50){
                        motorServo.setPower(0);
                        up = false;
                    } else if(motorServo.getCurrentPosition() > 1080){
                        s1.setPosition(1);
                        motorServo.setPower(0);
                        down = false;
                    } else {
                        motorServo.setPower(0);
                    }

                    if(gamepad1.y){
                        up = true;
                    } else if(gamepad1.a){
                        down = true;
                    }
                    if(gamepad1.x) {
                        threadServo.start();
                    }*/
                }
            }
        }
    }
