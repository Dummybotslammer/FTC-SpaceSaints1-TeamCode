package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Glockenzerstorer_LiftActuationTest extends LinearOpMode {
    //--HARDWARE VARIABLE INIT--
    private DcMotor rightDrive; //PORT 0
    private DcMotor leftDrive; //PORT 1
    private DcMotorEx liftMotor; //PORT 2
    private DcMotorEx pivotMotor; //PORT 3
    private Servo controlServo; //PORT 0
    private Servo clawServo; //PORT 1

    //--CUSTOM FUNCTION DECLARATIONS--
    private int moveDynamic(double powerR, double powerL) {
        this.leftDrive.setPower(-1.0D * powerL);
        this.rightDrive.setPower(powerR);
        return 1;
    }

    private double[] PID_Output(double error, double previousError, long deltaTime, double[] PID_values, double[] PID_coefficients) {
        //time is in nanoSeconds
        //PID_values in order: Proportional Value, Cumulative Integral Value, Derivative Value
        //PID_coefficients in order: Proportional Gain, Integral Gain, Derivative Gain
        PID_values[0] = PID_coefficients[0] * error;
        PID_values[1] += PID_coefficients[1] * (deltaTime * error);
        PID_values[2] = PID_coefficients[2] * (error - previousError / deltaTime);
        return PID_values;
    }

    @Override
    public void runOpMode() {
        //--HARDWARE  MAPPINGS--
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivotMotor");
        controlServo = hardwareMap.get(Servo.class, "controlServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        //--RUNTIME VARIABLES:--
        //For Drive-Train
        final double FwdDriveSpeedMax = 0.70;
        final double TurnSpeedMax = 0.5;
        double RightDriveSpeed = 0.5;
        double LeftDriveSpeed = 0.5;
        double threshold = 0.5;
        //For Linear Lift
        int liftPosition = 0;
        int maxLiftPosition = 0;
        int minLiftPosition = 0;
        double liftPower = 1.0;
        double liftError = 0;
        double liftPreviousError = 0;
        double liftPID_values[] = new double[3]; //Proportional Value, Cumulative Integral Value, Derivative Value
        double liftPID_coefficients[] = {4, 0, 2}; //Proportional Gain, Integral Gain, Derivative Gain
        //For Lift Pivot
        int pivotPosition = 0;
        int maxPivotPosition = 883;
        int minPivotPosition = 0;
        int pivotPresetPositions[] = {maxPivotPosition, 497, minPivotPosition};
        boolean wasPivotPresetLastActivated = false;
        double pivotPower = 1.0;
        double pivotError = 0;
        double pivotPreviousError = 0;
        double pivotPID_values[] = new double[3];
        double pivotPID_coefficients[] = {6, 0, 4};
        double scaledPivotRange = pivotMotor.getCurrentPosition()/ (double) maxPivotPosition;

        //For Elapsed Time
        long startElapsed = System.nanoTime();
        long endElapsed = System.nanoTime();
        long elapsedTime = startElapsed - endElapsed;

        telemetry.addData("Status:", "Running");

        //--MOTOR ENCODER CONFIGURATIONS:--
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        liftMotor.setPower(liftPower);

        //Runtime Loop:
        while (opModeIsActive()) {
            scaledPivotRange = pivotMotor.getCurrentPosition()/ (double) maxPivotPosition;
            LeftDriveSpeed = 0;
            RightDriveSpeed = 0;
            double stick_x = this.gamepad1.right_stick_x;
            double stick_y = this.gamepad1.left_stick_y;

            //Lift Controls
            if (gamepad1.dpad_up) {
                liftPosition -= 2;
            }

            else if (gamepad1.dpad_down) {
                liftPosition += 2;
            }

            else {
                //pass?
            }


            liftError = liftPosition - liftMotor.getCurrentPosition();
            pivotError = pivotPosition - pivotMotor.getCurrentPosition();
            endElapsed = System.nanoTime();
            elapsedTime = endElapsed - startElapsed;

            liftPID_values = PID_Output(liftError, liftPreviousError, elapsedTime, liftPID_values, liftPID_coefficients);
            //liftMotor.setVelocity((int) (liftPID_values[0]+liftPID_values[1]+liftPID_values[2]));
            liftMotor.setVelocity(100);

            liftPreviousError = liftError;
            pivotPreviousError = pivotError;
            startElapsed = System.nanoTime();

            telemetry.addData("Elapsed Time (Nanoseconds)", elapsedTime);
            telemetry.addData("Elapsed Time (Miliseconds)", elapsedTime/1000000);

            telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            telemetry.addData("Lift Target Position", liftPosition);
            telemetry.addData("Lift Velocity", liftMotor.getVelocity());
            telemetry.addData("Lift Error", liftError);
            telemetry.addData("Lift Previous Error", liftPreviousError);
            telemetry.addData("Lift Power", liftMotor.getPower());
            telemetry.addData("LIFT P", liftPID_values[0]);
            telemetry.addData("LIFT I", liftPID_values[1]);
            telemetry.addData("LIFT D", liftPID_values[2]);
            telemetry.addData("LIFT Final PID Output", (int) (liftPID_values[0]+liftPID_values[1]+liftPID_values[2]));

            telemetry.update();
        }
    }
}
