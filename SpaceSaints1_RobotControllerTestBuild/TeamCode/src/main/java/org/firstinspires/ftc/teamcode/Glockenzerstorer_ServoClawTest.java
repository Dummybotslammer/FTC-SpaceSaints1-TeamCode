package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Glockenzerstorer_ServoClawTest extends LinearOpMode {
    //--HARDWARE VARIABLE INIT--
    private DcMotorEx pivotMotor; //PORT 3
    private Servo controlServo; //PORT 0
    private Servo clawServo; //PORT 1
    private DcMotorEx liftMotor; //PORT 2

    //--CUSTOM FUNCTION DECLARATIONS--
    private double[] PID_Output(double error, double previousError, long deltaTime, double[] PID_values, double[] PID_coefficients) {
        //time is in nanoSeconds
        PID_values[0] = PID_coefficients[0]*error;
        PID_values[1] += PID_coefficients[1]*(deltaTime*error);
        PID_values[2] = PID_coefficients[2]*(error-previousError/deltaTime);
        return PID_values;
    }

    @Override
    public void runOpMode() {
        //--HARDWARE  MAPPINGS--
        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivotMotor");
        controlServo = hardwareMap.get(Servo.class, "controlServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        //--RUNTIME VARIABLES:--
        double maxPosition = 883.0;
        double scaledPivotRange = pivotMotor.getCurrentPosition()/maxPosition;

        //For Elapsed Time
        long startElapsed = System.nanoTime();
        long endElapsed = System.nanoTime();
        long elapsedTime = startElapsed - endElapsed;

        this.telemetry.addData("Status:", "Running");

        //--MOTOR ENCODER CONFIGURATIONS:--
        //Lift Motor
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Pivot Motor
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setTargetPosition(0);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pivotMotor.setVelocity(80);

        waitForStart();

        liftMotor.setPower(0);
        pivotMotor.setPower(0);
        clawServo.setPosition(0.5);

        //Runtime Loop:
        while (opModeIsActive()) {
            scaledPivotRange = pivotMotor.getCurrentPosition()/maxPosition;
            controlServo.setPosition(scaledPivotRange);

            //Claw Control Loop
            //A is open, B is close.
            if (gamepad1.a) {
                clawServo.setPosition(0.5);

            }

            else if (gamepad1.b) {
                clawServo.setPosition(0);
            }

            else {
                clawServo.setPosition(clawServo.getPosition());
            }

            telemetry.addData("Pivot Motor Position", pivotMotor.getCurrentPosition());
            telemetry.addData("Scaled Range of Pivot", scaledPivotRange);
            telemetry.addData("Servo Target Position", controlServo.getPosition());
            telemetry.addData("gamepad A", gamepad1.a);
            telemetry.addData("gamepad B", gamepad1.b);
            telemetry.addData("gamepad X", gamepad1.x);
            telemetry.addData("gamepad Y", gamepad1.y);

            telemetry.addData("Lift Motor Position", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
