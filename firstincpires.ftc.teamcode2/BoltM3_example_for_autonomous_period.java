package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

@Autonomous(name = "BoltM3 example for autonomous period", group = "")
public class BoltM3 extends LinearOpMode {

    private DcMotorEx motorFL, motorBL, motorFR, motorBR, motorIntake, motorLift;
    private DcMotor motorDuck;
    private Servo s;
    //private RevColorSensorV3 c1;
    private static int tickForRevolution = 560;
    private static double wheelRadius = 0.045; //Metres
    private static double lengthOfWheel = 2 * Math.PI * wheelRadius;
    private static double gearRatio = 1;
    private static double widthOfRobot = 0.208; //Metres

    /**
     * fuck vuforia and tensor flow, we beat this shit
     */
    /*private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };
    private static final String VUFORIA_KEY =
            "input youe key";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    boolean left = false, right = false, center = false;*/

    @Override
    public void runOpMode() {
        /*motorFL = hardwareMap.dcMotor.get("leftF");
        motorBL = hardwareMap.dcMotor.get("leftR");
        motorFR = hardwareMap.dcMotor.get("rightF");
        motorBR = hardwareMap.dcMotor.get("rightR");*/
        motorFL = hardwareMap.get(DcMotorEx.class, "left_front_motor");
        motorBL = hardwareMap.get(DcMotorEx.class, "left_bottom_motor");
        motorFR = hardwareMap.get(DcMotorEx.class, "right_front_motor");
        motorBR = hardwareMap.get(DcMotorEx.class, "right_bottom_motor");
        motorIntake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motorDuck = hardwareMap.dcMotor.get("duck_motor");
        motorLift = hardwareMap.get(DcMotorEx.class, "extension_motor");

    /**
    * to reset the encoder counter of each motor
    */
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /**
         * Dear Gleb
         * this is just example
         * 0:13 27.02.2022 - these series of strings have not been tested yet
         * and not all motors are used
         **/
        //c1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        //c1.setGain(100f);
        /*motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);*/
        /*initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        Recognition mainRec = null;
        float maxConf = 0f;
        boolean check = false;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (true) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addLine("Object(" + i + ")" + recognition.getLabel() + " Conf:" + recognition.getConfidence()
                                + "\nLeft:" + recognition.getLeft() + " Right: " + recognition.getRight());
                        i++;
                        if(maxConf < recognition.getConfidence()){
                            maxConf = recognition.getConfidence();
                            mainRec = recognition;
                        }
                        check = true;
                    }
                    telemetry.update();
                    if(check || runtime.seconds() > 3){
                        break;
                    }
                }
            }
        }
        if(mainRec != null) {
            if (mainRec.getLeft() < 150) {
                telemetry.addLine("Duck on Left");
                telemetry.update();
                left = true;
            } else {
                telemetry.addLine("Duck on Center");
                telemetry.update();
                center = true;
            }
        } else {
            telemetry.addLine("Duck on Right");
            telemetry.update();
            right = true;
        }
        tfod.shutdown();*/
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.update();
                if (gamepad1.y) {
                    drive(0.7, 2.3, 2.3);
                    /*drive_side(0.7, 0.4, 0.4, "right");
                    drive_side(0.7, 0.4, 0.4, "left");
                    drive(0.7, -0.7, -0.7);
                    turn(0.7, 60, "left");
                    turn(0.7, 60, "right");*/
                    /*turn(1, 90, "left");
                    sleep(200);
                    drive_forward(0.7, -0.7, -0.7);
                    sleep(200);
                    duck(0.6, 3000);
                    drive_side_right(0.7, 0.67, 0.67);
                    drive_forward(0.7, 0.5, 0.5);
                    //place_item
                    drive_forward(0.7, -0.49, -0.49);
                    drive_side_left(0.7, 0.2, 0.2);*/
                    /*turn(0.5 , 90, "left");
                    sleep(200);
                    drive_forward(0.7, 0.7, 0.7);
                    sleep(200);
                    //drive_side_left(0.7, 0.67, 0.67);
                    turn(0.7, 90, "right");
                    sleep(200);
                    drive_forward(0.7, 0.7, 0.7);
                    sleep(200);*/
                    //place_item
                    //drive_forward(0.6, -0.52, -0.52);
                    //drive_side_right(0.7, 0.3, 0.3);
                    //drive_forward(0.7, 1, 1);
                    //sleep(2000);
                    //drive_forward(0.7, 0.4, 0.4);
                    //drive_side_left(0.7, 0.2, 0.2);
                    //drive_side_right(0.7, 0.3, 0.3);
                    /*sleep(200);
                    turn(0.5 , 90, "right");
                    sleep(200);
                    drive_forward(0.7, 0.52, 0.52);
                    sleep(200);
                    drive_side_left(0.7, 0.67, 0.67);
                    sleep(200);
                    drive_forward(0.6, -0.52, -0.52);
                    drive_side_right(0.7, 0.3, 0.3);
                    drive_forward(0.7, 1, 1);
                    sleep(2000);*/
                }
            }
        }
    }

    /** WARNING!!!
     * for correct using of functions below, each motor must have both-side plugger encoder
     * i dont give a fuck where is your team's 4th abnormally wide go-builda encoder
     * for right rear motor
     */

    private void drive(double power, double toLeft, double toRight) {
        int fR, bR, fL, bL;
        if(opModeIsActive()) {

            //(int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
            fR = motorFR.getCurrentPosition() +
                    (int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
            bR = motorBR.getCurrentPosition() +
                    (int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
            fL = motorFL.getCurrentPosition() +
                    (int)((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);
            bL = motorBL.getCurrentPosition() +
                    (int)((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);

            motorFR.setTargetPosition(fR);
            //motorBR.setTargetPosition(bR);
            motorFL.setTargetPosition(fL);
            motorBL.setTargetPosition(bL);

            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setPower(power);
            //motorBR.setPower(power);
            motorFL.setPower(power);
            motorBL.setPower(power);

            if (motorFR.isBusy()) {
                motorBR.setPower(power);
            }
            while(opModeIsActive() &&
                    (motorFR.isBusy() || motorBR.isBusy()
                            || motorBL.isBusy() || motorFL.isBusy())) {
                telemetry.addLine("FR: " + motorFR.getCurrentPosition() + " " + fR);
                telemetry.addLine("FL: " + motorFL.getCurrentPosition() + " " + fL);
                telemetry.addLine("BR: " + motorBR.getCurrentPosition() + " " + bR);
                telemetry.addLine("BL: " + motorBL.getCurrentPosition() + " " + bL);
                telemetry.update();
            }
            motorFR.setPower(0);
            motorBR.setPower(0);
            motorFL.setPower(0);
            motorBL.setPower(0);
        }
    }

    /**
     * @param power the power of motors
     * @param toLeft the range in meters for front left and rear left
     * @param toRight the range in meters for front right and rear right
     * @param side the side that you want to turn, either "left" or "right"
     */

    private void drive_side(double power, double toLeft, double toRight, String side) {
        int fR, bR, fL, bL;
        if(opModeIsActive()) {
            if (side.equals("right")) {
                //(int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
                fR = motorFR.getCurrentPosition() -
                        (int) ((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
                bR = motorBR.getCurrentPosition() +
                        (int) ((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
                fL = motorFL.getCurrentPosition() +
                        (int) ((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);
                bL = motorBL.getCurrentPosition() -
                        (int) ((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);

                motorFR.setTargetPosition(fR);
                //motorBR.setTargetPosition(bR);
                motorFL.setTargetPosition(fL);
                motorBL.setTargetPosition(bL);

                motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFR.setPower(power);
                //motorBR.setPower(power);
                motorFL.setPower(power);
                motorBL.setPower(power);

                if (motorFR.isBusy()) {
                    motorBR.setPower(power);
                }

                while (opModeIsActive() &&
                        (motorFR.isBusy() || motorBR.isBusy()
                                || motorFL.isBusy() || motorBL.isBusy())) {
                    telemetry.addLine("FR: " + motorFR.getCurrentPosition());
                    telemetry.addLine("FL: " + motorFL.getCurrentPosition());
                    telemetry.addLine("BR: " + motorBR.getCurrentPosition());
                    telemetry.addLine("BL: " + motorBL.getCurrentPosition());
                    telemetry.update();
                }
                motorFR.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorBL.setPower(0);
            } else if (side.equals("left")) {
                //(int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
                fR = motorFR.getCurrentPosition() +
                        (int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
                bR = motorBR.getCurrentPosition() -
                        (int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
                fL = motorFL.getCurrentPosition() -
                        (int)((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);
                bL = motorBL.getCurrentPosition() +
                        (int)((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);

                motorFR.setTargetPosition(fR);
                //motorBR.setTargetPosition(bR);
                motorFL.setTargetPosition(fL);
                motorBL.setTargetPosition(bL);

                motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFR.setPower(power);
                //motorBR.setPower(power);
                motorFL.setPower(power);
                motorBL.setPower(power);

                if (motorFR.isBusy()) {
                    motorBR.setPower(power);
                }

                while(opModeIsActive() &&
                        (motorFR.isBusy() || motorBR.isBusy()
                                || motorFL.isBusy() || motorBL.isBusy())) {
                    telemetry.addLine("FR: " + motorFR.getCurrentPosition() + " " + fR);
                    telemetry.addLine("FL: " + motorFL.getCurrentPosition() + " " + fL);
                    telemetry.addLine("BR: " + motorBR.getCurrentPosition() + " " + bR);
                    telemetry.addLine("BL: " + motorBL.getCurrentPosition() + " " + bL);
                    telemetry.update();
                }
                motorFR.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorBL.setPower(0);
            }
        }
    }

    /**
     * @param power the power of motors, [-1; 1]
     * @param degree the angle of rotation, any angle
     * @param side the side that you want to turn, either "left" or "right"
     */
    private void turn(double power, double degree, String side) {
        if(opModeIsActive()){
            if (side.equals("right")) {
                drive(power, (degree / 360) * Math.PI * 2 * 0.32, -1 *  (degree / 360) * Math.PI * 2 * 0.32);
            } else if (side.equals(side == "left")) {
                drive(power, -1 * (degree / 360) * Math.PI * 2 * 0.32, (degree / 360) * Math.PI * 2 * 0.32);
            }
        }
    }

    /*private void drive_side_left(double power, double toLeft, double toRight) {
        int fR, bR, fL, bL;
        if(opModeIsActive()){
            //(int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
            fR = motorFR.getCurrentPosition() +
                    (int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
            bR = motorBR.getCurrentPosition() -
                    (int)((toRight / lengthOfWheel) * tickForRevolution * gearRatio);
            fL = motorFL.getCurrentPosition() -
                    (int)((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);
            bL = motorBL.getCurrentPosition() +
                    (int)((toLeft / lengthOfWheel) * tickForRevolution * gearRatio);

            motorFR.setTargetPosition(fR);
            motorBR.setTargetPosition(bR);
            motorFL.setTargetPosition(fL);
            motorBL.setTargetPosition(bL);

            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setPower(power);
            motorBR.setPower(power);
            motorFL.setPower(power);
            motorBL.setPower(power);
            while(opModeIsActive() &&
                    (motorFR.isBusy() || motorBR.isBusy()
                            || motorFL.isBusy() || motorBL.isBusy())) {
                telemetry.addLine("FR: " + motorFR.getCurrentPosition() + " " + fR);
                telemetry.addLine("FL: " + motorFL.getCurrentPosition() + " " + fL);
                telemetry.addLine("BR: " + motorBR.getCurrentPosition() + " " + bR);
                telemetry.addLine("BL: " + motorBL.getCurrentPosition() + " " + bL);
                telemetry.update();
            }
            motorFR.setPower(0);
            motorBR.setPower(0);
            motorFL.setPower(0);
            motorBL.setPower(0);
        }
    }*/

    /*private void turn(double power, double degree, String side) {
        if(opModeIsActive()){
            double arc = (degree / 360) * Math.PI * 2 * 0.38;
            if (side.equals("right")) {
                drive_forward(power, arc,
                        -1 * arc);
            } else if (side.equals("left")) {
                drive_forward(power,  -1 * arc, arc);
            }
        }
    }*/

    /*private void duck(double power, long sl){
        motorDuck.setPower(power);
        sleep(sl);
    }*/

    /*private float driveStraightTillTargetObject(double power){
        float driven = 0, last = motorFL.getCurrentPosition();
        if(opModeIsActive()){
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);

            while(opModeIsActive()){
                telemetry.addLine("First, Position:" + motorFL.getCurrentPosition()
                        + " Power:" + motorFL.getPower());
                telemetry.addLine("Second, Position:" + motorFR.getCurrentPosition()
                        + " Power:" + motorFR.getPower());
                float red = c1.getNormalizedColors().red * 255;
                float green = c1.getNormalizedColors().green * 255;
                float blue = c1.getNormalizedColors().blue * 255;
                double distance = c1.getDistance(DistanceUnit.CM);
                telemetry.addLine(" R:" + red + "\n G:" + green + "\n B:" + blue);
                telemetry.addLine("Distance:" + distance + " cm");
                telemetry.update();
                if(distance < 15 && (2 * green - red - blue > 7.5 || red + green > blue * 3.5 )){
                    break;
                }
            }
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
        driven = motorFL.getCurrentPosition() - last;
        sleep(500);
        return driven;
    }*/

    /*private void intake(int time) {
        if(opModeIsActive()){
            motorIntake.setPower(1);
            sleep(time);
            motorIntake.setPower(0);
        }
    }*/


    /*private void throwItem(double power, double up){
        motorLift.setTargetPosition((int)(1100 - up * 1050));
        s.setPosition(0.965);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(power);
        while(opModeIsActive() && motorLift.isBusy()){
            if(motorLift.getCurrentPosition() < 800){
                s.setPosition(0.9);
            }
        }
        s.setPosition(0);
        sleep(500);
        s.setPosition(0.9);

    }*/


    /*private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }*/
}