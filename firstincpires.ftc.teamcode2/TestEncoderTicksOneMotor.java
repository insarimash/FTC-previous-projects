package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
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
@Autonomous

public class TestEncoderTicksOneMotor extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor encoder_motor;
    private Gyroscope imu;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        encoder_motor = hardwareMap.get(DcMotor.class, "encoder_motor");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        
        encoder_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("running at",Integer.toString(encoder_motor.getCurrentPosition()));
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            encoder_motor.setTargetPosition(1000);
            encoder_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            encoder_motor.setPower(1);
            while (opModeIsActive() &&
                   encoder_motor.isBusy()) {

                // Display it for the driver.
                telemetry.addData(Integer.toString(encoder_motor.getCurrentPosition()),"position current");
        
                telemetry.update();
            }
            encoder_motor.setPower(0);
            encoder_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
