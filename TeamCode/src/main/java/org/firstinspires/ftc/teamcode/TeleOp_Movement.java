package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp_Movement")
public class TeleOp_Movement extends LinearOpMode {
  private DcMotor BR;
  private DcMotor BL;
  private DcMotor FR;
  private DcMotor FL;
  private DcMotor Intake;
  private DcMotorEx LM;
  private Servo rightServo;
  private Servo leftServo;

  double leftFrontPower;
  double leftBackPower;
  double rightFrontPower;
  double rightBackPower;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
   
  @Override
  public void runOpMode() {
    ElapsedTime runtime;
    float axial;
    float lateral;
    float yaw;
    double max;

    BR = hardwareMap.get(DcMotor.class, "BR");
    BL = hardwareMap.get(DcMotor.class, "BL");
    FR = hardwareMap.get(DcMotor.class, "FR");
    FL = hardwareMap.get(DcMotor.class, "FL");
    Intake = hardwareMap.get(DcMotor.class, "Intake");
    LM = hardwareMap.get(DcMotorEx.class, "LM");
    rightServo = hardwareMap.get(Servo.class, "rightServo");
    leftServo = hardwareMap.get(Servo.class, "leftServo");

    PIDFController pid = new PIDFController(testPID.kP, testPID.kI, testPID.kD, testPID.kF);
  
    // This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
    // This code will work with either a Mecanum-Drive or an X-Drive train.
    // Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
    // Note that a Mecanum drive must display an X roller-pattern when viewed from above.
    // Also note that it is critical to set the correct rotation direction for each motor.  See details below.
    // Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
    // Each motion axis is controlled by one Joystick axis.
    // 1) Axial:    Dariving forward and backward                Left-joystick Forward/Backward
    // 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
    // 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
    // This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
    // When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
    // the direction of all 4 motors (see code below).
    runtime = new ElapsedTime();
    // ########################################################################################
    // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
    // ########################################################################################
    // Most robots need the motors on one side to be reversed to drive forward.
    // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft).
    // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
    // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
    // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
    // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
    // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
    BR.setDirection(DcMotor.Direction.FORWARD);
    BL.setDirection(DcMotor.Direction.REVERSE);
    FR.setDirection(DcMotor.Direction.FORWARD);
    FL.setDirection(DcMotor.Direction.REVERSE);
    LM.setDirection(DcMotor.Direction.REVERSE);
    Intake.setDirection(DcMotor.Direction.REVERSE);
    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    runtime.reset();
    // Run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
      // Note: pushing stick forward gives negative value
      axial = -gamepad1.left_stick_y;
      lateral = gamepad1.left_stick_x;
      yaw = gamepad1.right_stick_x;
      /*
      //I don't know what this does!!!
      if (yaw > 0.8) {
        yaw = (float) (yaw - 0.3);
      }
      if (yaw < -0.8) {
        yaw = (float) (yaw + 0.3);
      }
      */
      // Combine the joystick requests for each axis-motion to determine each wheel's power.
      // Set up a variable for each drive wheel to save the power level for telemetry.
      leftFrontPower = axial + lateral + yaw;
      rightFrontPower = (axial - lateral) - yaw;
      leftBackPower = (axial - lateral) + yaw;
      rightBackPower = (axial + lateral) - yaw;
      
      // Normalize the values so no wheel power exceeds 100%
      // This ensures that the robot maintains the desired motion.
      max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
      if (max > 1) {
          leftFrontPower = leftFrontPower / max ;
          rightFrontPower = rightFrontPower / max ;
          leftBackPower = leftBackPower / max ;
          rightBackPower = rightBackPower / max ;
      }

      // Send calculated power to wheels.
      if (gamepad1.right_trigger > 0.4) {
          BR.setPower(rightBackPower * 0.35);
          FR.setPower(rightFrontPower * 0.35);
          BL.setPower(leftBackPower * 0.35);
          FL.setPower(leftFrontPower * 0.35);
      }
      else {
          BR.setPower(rightBackPower);
          FR.setPower(rightFrontPower);
          BL.setPower(leftBackPower);
          FL.setPower(leftFrontPower);
      }

      // Intake is activated when the left trigger is pressed enough
        if (gamepad2.left_trigger > 0.4) {
            Intake.setPower(1);
        }
        else if (gamepad2.left_bumper) {
            Intake.setPower(-1);
        }
        else {
            Intake.setPower(0);
        }
      
      /* Whenever button y is pressed the 
      servo moves the ball towards the launching 
      mechanism */
      if (gamepad2.y){
        rightServo.setPosition(0.9);
        leftServo.setPosition(0.1);
      }
      else if(gamepad2.x){
        rightServo.setPosition(0.2);
        leftServo.setPosition(0.8);
      }
      else {
        rightServo.setPosition(0.5);
        leftServo.setPosition(0.5);
      }

      // 
      if (gamepad2.a) {
          testPID.targetPos = 2000;
          testPID.updateLaunchMotor(LM, pid);
      }
      else if (gamepad2.b) {
          testPID.targetPos = -2000;
          testPID.updateLaunchMotor(LM, pid);
      }
      else {
          LM.setPower(0);
      }

      // Show the elapsed game time and wheel power.
      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
      telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2) + "\n");
      telemetry.update();
    }
  }
}
