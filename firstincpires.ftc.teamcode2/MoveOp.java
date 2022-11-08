package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@TeleOp

public class MoveOp extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor left_motor;
    private DcMotor right_motor;
    private Gyroscope imu;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        left_motor = hardwareMap.get(DcMotor.class, "Left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "Right_motor");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
double tgtPowerLeft=0; double tgtPowerRight=0;
        while (opModeIsActive()) {
            tgtPowerLeft=-this.gamepad1.left_stick_y;
            tgtPowerRight=this.gamepad1.right_stick_y;
            left_motor.setPower(tgtPowerLeft);
            right_motor.setPower(tgtPowerRight);
            telemetry.addData("Target power left", tgtPowerLeft);
            telemetry.addData("Target power right", tgtPowerRight);
            telemetry.addData("Motor power left", left_motor.getPower());
            telemetry.addData("Motor power right", right_motor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
        // run until the end of the match (driver presses STOP)
      
    }
}