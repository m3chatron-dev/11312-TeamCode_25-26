package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp
public class testPID extends OpMode {


    DcMotorEx launchMotor;

    PIDFController pid;

    public static double kP = 0.01, kI = 0.01, kD = 0.0, kF = 0.0;

    public static double targetPos = 0;

    static double currentPos = 0;
    static double power = 0.0;

    public static void updateLaunchMotor(DcMotorEx launchMotor, PIDFController pid) {
        currentPos = launchMotor.getVelocity();
        pid.setPIDF(kP, kI, kD, kF);
        power = pid.calculate(targetPos, currentPos);
        launchMotor.setPower(power);
    }

    @IgnoreConfigurable
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        launchMotor = hardwareMap.get(DcMotorEx.class, "LM");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDFController(kP, kI, kD, kF);
    }

    @Override
    public void loop() {
        updateLaunchMotor(launchMotor, pid);
        panelsTelemetry.addData("current pos", currentPos);
        panelsTelemetry.addData("target pos", targetPos);
        panelsTelemetry.update(telemetry);
    }
}
