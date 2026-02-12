package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Disabled
@TeleOp(name = "camarão")
public class pidfshu extends OpMode {

    public DcMotorEx VS5;

    public double HighVelocity = 1500;
    public double LowVelocity = 900;

    double curTargetVelocity = HighVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = { 10.0, 1.0, 0.1,0.001,0.0001};

    int stepindex = 1;
    @Override
    public void init() {

        VS5 = hardwareMap.get(DcMotorEx.class,"VS5");
        VS5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VS5.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        VS5.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init completo");

    }

    @Override
    public void loop() {

        if(gamepad1.yWasPressed()) {
            if (curTargetVelocity == HighVelocity) {
                curTargetVelocity = LowVelocity;
            } else{ curTargetVelocity = HighVelocity;}
        }

        if (gamepad1.bWasPressed()){
            stepindex = (stepindex + 1)% stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepindex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepindex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepindex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepindex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        VS5.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        VS5.setVelocity(curTargetVelocity);

        double curVelocity = VS5.getVelocity();
        double erro = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error","%.2f",erro);
        telemetry.addLine("-------------------------");
        telemetry.addData("Tuning P","%.4f(D-pad U/D),", P);
        telemetry.addData("Tuning F","%.4f(D-pad L/R),", F);
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes [stepindex]);
    }
}
