package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.FTC_PowerPlayControlSystems;

@Autonomous
public class Glockenzerstorer_DirectEncoderAutonomousTest extends LinearOpMode {
    //--CONSTANTS & HARDWARE VARIABLE INIT--
    DcMotorEx leftDrive;
    DcMotorEx rightDrive;
    DcMotorEx liftMotor;
    DcMotorEx pivotMotor;

    //DRIVETRAIN CONSTANTS:
    static final double TICKS_PER_HDMOTOR_REVOLUTION = 28;
    static final double TOTAL_GEAR_REDUCTION = (5.23/1) * (3.61/1);
    static final double WHEEL_DIAMETER = 90;
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    //To make one rotation of the wheel, the motor must make X rotations. X is based on the gear reduction.
    static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_HDMOTOR_REVOLUTION * TOTAL_GEAR_REDUCTION;
    static final double TICKS_PER_WHEEL_MM = TICKS_PER_WHEEL_REVOLUTION / WHEEL_CIRCUMFERENCE;

    static final double WHEEL_TO_CENTRE_DISTANCE = (281+25+25)/2;

    //RUNTIME VARIABLES:
    //Drivetrain
    double drivePower = 0.70;
    int leftPosition = 0;
    double leftError = 0;
    double leftPreviousError = 0;
    double leftPID_values[] = new double[3];
    double leftPID_coefficients[] = {6, 0, 4};

    int rightPosition = 0;
    double rightError = 0;
    double rightPreviousError = 0;
    double rightPID_values[] = new double[3];
    double rightPID_coefficients[] = {6, 0, 4};

    //Pivot
    double pivotPower = 1.0;
    int pivotPosition = 0;
    int maxPivotPosition = 883;
    int minPivotPosition = 0;
    double pivotError = 0;
    double pivotPreviousError = 0;
    double pivotPID_values[] = new double[3];
    double pivotPID_coefficients[] = {6, 0, 4};
    double scaledPivotRange = pivotMotor.getCurrentPosition()/ (double) maxPivotPosition;

    //Lift
    double liftPower = 1.0;
    int liftPosition = 0;
    int maxLiftPosition = 0;
    int minLiftPosition = 0;
    double liftError = 0;
    double liftPreviousError = 0;
    double liftPID_values[] = new double[3]; //Proportional Value, Cumulative Integral Value, Derivative Value
    double liftPID_coefficients[] = {4, 0, 2}; //Proportional Gain, Integral Gain, Derivative Gain

    //Autonomous Logic
    boolean hasStepFinished = false;
    int stepIndex = 0;
    int[][] autoSteps = {
            {(int)(500*TICKS_PER_WHEEL_MM), (int)(500*TICKS_PER_WHEEL_MM)},
    };

    //For Elapsed Time
    long startElapsed = System.nanoTime();
    long endElapsed = System.nanoTime();
    long elapsedTime = startElapsed - endElapsed;

    //--CUSTOM FUNCTION DECLARATIONS--
    public void UpdateDrivePID() {
        leftError = leftPosition - leftDrive.getCurrentPosition();
        rightError = rightPosition - rightDrive.getCurrentPosition();
        liftError = liftPosition - liftMotor.getCurrentPosition();
        pivotError = pivotPosition - pivotMotor.getCurrentPosition();

        double[][] PID_values = FTC_PowerPlayControlSystems.DifferentialDrivePID_Output(
                leftError,
                leftPreviousError,
                rightError,
                rightPreviousError,
                elapsedTime,
                leftPID_values,
                leftPID_coefficients,
                rightPID_values,
                rightPID_coefficients);

        leftPID_values = PID_values[0];
        rightPID_values = PID_values[1];

        leftDrive.setVelocity(leftPID_values[0] + leftPID_values[1] + leftPID_values[2]);
        rightDrive.setVelocity(rightPID_values[0] + rightPID_values[1] + rightPID_values[2]);

        leftPreviousError = leftError;
        rightPreviousError = rightError;
        liftPreviousError = liftError;
        pivotPreviousError = pivotError;
    }

    @Override
    public void runOpMode() {
        //--INITIALIZATION PHASE--
        //HARDWARE MAPPINGS
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivotMotor");

        //MOTOR ENCODER CONFIGURATIONS:
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //Motors are powered up.
        leftDrive.setPower(drivePower);
        rightDrive.setPower(drivePower);
        liftMotor.setPower(liftPower);
        pivotMotor.setPower(pivotPower);

        while (opModeIsActive()) {
            endElapsed = System.nanoTime();
            elapsedTime = endElapsed - startElapsed;

            //AUTONOMOUS SEQUENCE LOGIC
            if (hasStepFinished) {
                stepIndex++;
                if (stepIndex == autoSteps.length) {
                    stop();
                }
            }

            leftPosition = autoSteps[stepIndex][0];
            rightPosition = autoSteps[stepIndex][1];

            //Add logic to figure out when the step has finished.

            UpdateDrivePID();

            startElapsed = System.nanoTime();
        }

    }
}
