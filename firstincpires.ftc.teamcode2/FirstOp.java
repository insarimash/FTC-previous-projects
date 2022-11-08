package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Starter (Blocks to Java)")
public class FirstOp extends LinearOpMode {

  private Servo DuckSpinner;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    DuckSpinner = hardwareMap.get(Servo.class, "DuckSpinner");

    waitForStart();
        try{
                Thread.sleep(2000);
            } catch(Exception e){
            }
        DuckSpinner.setPosition(0);
        try{
                Thread.sleep(2000);
            } catch(Exception e){
            }
        DuckSpinner.setPosition(-45);
        return;
  }
}