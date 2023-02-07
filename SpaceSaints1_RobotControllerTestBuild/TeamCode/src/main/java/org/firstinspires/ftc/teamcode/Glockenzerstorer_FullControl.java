package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Glockenzerstorer_FullControl extends LinearOpMode {
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

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setTargetPosition(pivotPosition);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pivotMotor.setVelocity(80);

        waitForStart();

        liftMotor.setPower(liftPower);
        pivotMotor.setPower(pivotPower);
        clawServo.setPosition(0);

        //Runtime Loop:
        while (opModeIsActive()) {
            scaledPivotRange = pivotMotor.getCurrentPosition()/ (double) maxPivotPosition;
            LeftDriveSpeed = 0;
            RightDriveSpeed = 0;
            double stick_x = this.gamepad1.right_stick_x;
            double stick_y = this.gamepad1.left_stick_y;

            //Drive-Train Controls:
            if (stick_y > threshold) {
                LeftDriveSpeed += FwdDriveSpeedMax;
                RightDriveSpeed += FwdDriveSpeedMax;
            }

            else if (stick_y < -threshold) {
                LeftDriveSpeed += FwdDriveSpeedMax * -1;
                RightDriveSpeed += FwdDriveSpeedMax * -1;
            }

            if (stick_x < -threshold) {
                RightDriveSpeed += TurnSpeedMax * -1;
                LeftDriveSpeed += TurnSpeedMax;
            }

            else if (stick_x > threshold) {
                RightDriveSpeed += TurnSpeedMax;
                LeftDriveSpeed += TurnSpeedMax * -1;
            }

            //Lift Controls
            if (gamepad1.dpad_up) {
                //liftPosition += 5;
                liftMotor.setVelocity(-500);
            }

            else if (gamepad1.dpad_down) {
                //liftPosition -= 5;
                liftMotor.setVelocity(500);
            }

            else {
                //pass?
                liftMotor.setVelocity(0);
            }

            //Pivot Controls
            if (gamepad1.right_bumper) {
                pivotPosition -= 5;
            }

            else if (gamepad1.left_bumper) {
                pivotPosition += 5;
            }

            //Pivot Presets
            /*
            if (gamepad1.dpad_left) {

                if (!wasPivotPresetLastActivated) {
                    int index = 0;
                    int minPosition = pivotPresetPositions[index];

                    for (index = 1; index < pivotPresetPositions.length; index++){
                        if (pivotPosition - pivotPresetPositions[index] >= pivotPosition - minPosition && pivotPosition - pivotPresetPositions[index] < 0){
                            minPosition = pivotPresetPositions[index];
                        }
                    }
                    pivotPosition = minPosition;
                }

                wasPivotPresetLastActivated = true;
            }

            else if (gamepad1.dpad_right) {

                if (!wasPivotPresetLastActivated) {
                    int index = 0;
                    int minPosition = pivotPresetPositions[index];

                    for (index = 1; index < pivotPresetPositions.length; index++){
                        if (pivotPosition - pivotPresetPositions[index] <= pivotPosition - minPosition && pivotPosition - pivotPresetPositions[index] > 0){
                            minPosition = pivotPresetPositions[index];
                        }
                    }
                    pivotPosition = minPosition;
                }

                wasPivotPresetLastActivated = true;
            }

            else {
                wasPivotPresetLastActivated = false;
            }
            */

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

            //Soft Limits:
            //Lift
            if (liftMotor.getCurrentPosition() > maxLiftPosition || liftMotor.getCurrentPosition() < minLiftPosition) {
                //liftPosition = Math.min(liftPosition, maxLiftPosition);
                liftPosition = Math.max(liftPosition, minLiftPosition);
            }

            //Pivot
            if (pivotMotor.getCurrentPosition() > maxPivotPosition || pivotMotor.getCurrentPosition() < minPivotPosition) {
                pivotPosition = Math.min(pivotPosition, maxPivotPosition);
                pivotPosition = Math.max(pivotPosition, minPivotPosition);
            }

            //Actuator Updates
            controlServo.setPosition(scaledPivotRange-0.2);

            liftError = liftPosition - liftMotor.getCurrentPosition();
            pivotError = pivotPosition - pivotMotor.getCurrentPosition();
            endElapsed = System.nanoTime();
            elapsedTime = endElapsed - startElapsed;

            liftPID_values = PID_Output(liftError, liftPreviousError, elapsedTime, liftPID_values, liftPID_coefficients);
            //liftMotor.setVelocity((int) (liftPID_values[0]+liftPID_values[1]+liftPID_values[2]));

            pivotPID_values = PID_Output(pivotError, pivotPreviousError, elapsedTime, pivotPID_values, pivotPID_coefficients);
            pivotMotor.setVelocity((int) (pivotPID_values[0]+pivotPID_values[1]+pivotPID_values[2]));

            moveDynamic(RightDriveSpeed, LeftDriveSpeed);

            liftPreviousError = liftError;
            pivotPreviousError = pivotError;
            startElapsed = System.nanoTime();

            telemetry.addData("Lift Motor Position", liftMotor.getCurrentPosition());
            telemetry.addData("Lift Error", liftError);
            telemetry.addData("Elapsed Time (Nanoseconds)", elapsedTime);
            telemetry.addData("Elapsed Time (Miliseconds)", elapsedTime/1000000);

            telemetry.addData("wasPivotPresetLastActivated", wasPivotPresetLastActivated);
            telemetry.addData("Pivot Position", pivotMotor.getCurrentPosition());
            telemetry.addData("Pivot Target Position", pivotPosition);
            telemetry.addData("Scaled Pivot Range", scaledPivotRange);
            telemetry.addData("Pivot Velocity", pivotMotor.getVelocity());
            telemetry.addData("Pivot Error", pivotError);
            telemetry.addData("Pivot Power", pivotMotor.getPower());
            telemetry.addData("PIVOT P", pivotPID_values[0]);
            telemetry.addData("PIVOT I", pivotPID_values[1]);
            telemetry.addData("PIVOT D", pivotPID_values[2]);
            telemetry.addData("PIVOT Final PID Output", (int) (pivotPID_values[0]+pivotPID_values[1]+pivotPID_values[2]));

            telemetry.addData("Control Servo Target Position", controlServo.getPosition());
            telemetry.addData("Claw Servo Target Position", clawServo.getPosition());

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
