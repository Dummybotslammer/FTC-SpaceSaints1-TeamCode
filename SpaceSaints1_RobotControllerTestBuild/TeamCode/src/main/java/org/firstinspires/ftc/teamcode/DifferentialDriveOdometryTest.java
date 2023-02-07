package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
All measurements of distance and length are in MM (milimetres) unless specified otherwise.
All measurements of orientation/angles are in Radians unless specified otherwise.

In any array that is used to represent a 2D vector, the 0th index is right, and the 1st index is left. And if a 3rd index is present, it represents orientation.
 */

@Autonomous
public class DifferentialDriveOdometryTest extends LinearOpMode {
    private DcMotorEx rightDrive;
    private DcMotorEx leftDrive;

    private boolean stopRuntime = false;

    //DRIVETRAIN CONSTANTS:
    static final double TICKS_PER_HDMOTOR_REVOLUTION = 28;
    static final double TOTAL_GEAR_REDUCTION = (5.23/1) * (3.61/1);
    static final double WHEEL_DIAMETER = 90;
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    //To make one rotation of the wheel, the motor must make X rotations. X is based on the gear reduction.
    static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_HDMOTOR_REVOLUTION * TOTAL_GEAR_REDUCTION;
    static final double TICKS_PER_WHEEL_MM = TICKS_PER_WHEEL_REVOLUTION / WHEEL_CIRCUMFERENCE;

    static final double WHEEL_TO_CENTRE_DISTANCE = (281+25+25)/2;

    //KINEMATIC/MOVEMENT VARIABLES:
    private double leftDrivePower = 0.5;
    private double rightDrivePower = 0.5;

    //These velocities would be in RPS
    private int leftDriveVelocity = 175;
    private int rightDriveVelocity = 175;

    private int targetDeltaRightDrivePosition;
    private int targetDeltaLeftDrivePosition;

    private double targetDeltaOrientation; //In radians

    //TRACKER VARIABLES:
    private int previous_rightDrivePosition;
    private int previous_leftDrivePosition;
    private int current_rightDrivePosition;
    private int current_leftDrivePosition;

    private double delta_rightDriveDistance;
    private double delta_leftDriveDistance;

    private double totalDistanceTravelled;
    private double delta_distance;

    private double current_orientation;
    private double delta_orientation;

    private double current_xpos;
    private double current_ypos;

    private double delta_xpos;
    private double delta_ypos;

    //PATH WAYPOINTS:
    private double remainingDistanceToWaypoint;
    private int waypointIndex = -1;
    private double waypoints[][] =
            {
                    {0, 0, 0},
                    {500, 0, 0},
                    {0, 0, 0},
                    {0, -500, 0},
            };

    //METHODS:
    private void updateCurrentOdometry() {
        current_rightDrivePosition = rightDrive.getCurrentPosition();
        current_leftDrivePosition = leftDrive.getCurrentPosition();

        /*
        Issue with these calculations stems from the fact that the distance, xpos, and ypos (both calculated from distance), are all
        calculated solely based on the encoder ticks, so even if the robot rotates 180 degrees and moves
        in the opposite direction, the xpos and ypos will increase relative to the robot, not the global coordinate
        system. The important bit to understand is that although the robot is moving backwards, the backwards movement
        requires the robot to rotate backwards and move FORWARDS, which causes the change in coordinates to be positive.

        Solution: Account for rotation.
        (Use a linear transformation, and have a global and relative coordinate system)
        */
        delta_rightDriveDistance = (current_rightDrivePosition-previous_rightDrivePosition)/TICKS_PER_WHEEL_MM;
        delta_leftDriveDistance = (current_leftDrivePosition-previous_leftDrivePosition)/TICKS_PER_WHEEL_MM;

        delta_orientation = (delta_rightDriveDistance-delta_leftDriveDistance)/(2*WHEEL_TO_CENTRE_DISTANCE);
        delta_distance = (delta_leftDriveDistance+delta_rightDriveDistance)/2;

        delta_xpos = delta_distance * Math.cos(Math.toDegrees(delta_orientation));
        delta_ypos = delta_distance * Math.sin(Math.toDegrees(delta_orientation));

        current_xpos += delta_xpos;
        current_ypos += delta_ypos;
        current_orientation += delta_orientation;
        if (current_orientation > Math.PI) { current_orientation = -Math.PI + (current_orientation-Math.PI); }
        else if (current_orientation <= -Math.PI ) { current_orientation = Math.PI + (current_orientation+Math.PI); }
        totalDistanceTravelled += Math.abs(delta_distance);
    }

    private void updateDriveControls() {
        rightDrivePower = 0.5;
        leftDrivePower = -0.5;
        rightDrive.setPower(rightDrivePower);
        leftDrive.setPower(leftDrivePower);

        if (rightDrive.isBusy() || leftDrive.isBusy()) {
            remainingDistanceToWaypoint = Math.sqrt(Math.pow(waypoints[waypointIndex][0]-current_xpos, 2) + Math.pow(waypoints[waypointIndex][1]-current_ypos, 2));

            rightDriveVelocity = 500;
            leftDriveVelocity = 500;

            //rightDriveVelocity = (int) Math.round(remainingDistanceToWaypoint/WHEEL_CIRCUMFERENCE*TICKS_PER_WHEEL_REVOLUTION);
            //leftDriveVelocity = (int) Math.round(remainingDistanceToWaypoint/WHEEL_CIRCUMFERENCE*TICKS_PER_WHEEL_REVOLUTION);
        }

        else {
            waypointIndex += 1;
            if (waypointIndex > waypoints.length-1) {
                stopRuntime = true;
                return;
            }

            double next_xpos = waypoints[waypointIndex][0];
            double next_ypos = waypoints[waypointIndex][1];

            //Reorients the robot based on the new waypoint pose data:
            remainingDistanceToWaypoint = Math.sqrt(Math.pow(next_xpos-current_xpos, 2) + Math.pow(next_ypos-current_ypos, 2));

            rightDriveVelocity = 500;
            leftDriveVelocity = 500;

            //rightDriveVelocity = (int) Math.round(remainingDistanceToWaypoint/WHEEL_CIRCUMFERENCE*TICKS_PER_WHEEL_REVOLUTION);
            //leftDriveVelocity = (int) Math.round(remainingDistanceToWaypoint/WHEEL_CIRCUMFERENCE*TICKS_PER_WHEEL_REVOLUTION);

            //rightDrivePower = 0.5;
            //leftDrivePower = 0.5;
            //rightDrive.setPower(rightDrivePower);
            //rightDrive.setPower(rightDrivePower);

            //Find the angle required to direct the robot towards the waypoint.
            double radiansToWaypoint = 0;
            double theta = Math.atan((next_ypos-current_ypos)/(next_xpos-current_xpos)); //Also in radians
            if (next_ypos > current_ypos) {
                if (next_xpos > current_xpos) {
                    radiansToWaypoint = theta;
                }

                else if (next_xpos == current_xpos) {
                    radiansToWaypoint = Math.PI/2;
                }

                else {
                    radiansToWaypoint = Math.PI + theta;
                }
            }

            else if (next_ypos < current_ypos) {
                if (next_xpos > current_xpos) {
                    radiansToWaypoint = theta;
                }

                else if (next_xpos == current_xpos) {
                    radiansToWaypoint = -Math.PI/2;
                }

                else {
                    radiansToWaypoint = -Math.PI + theta;
                }
            }

            else if (next_xpos < current_xpos && next_ypos == current_ypos) {
                radiansToWaypoint = Math.PI;
            }

            else {
                //Also includes: if (next_xpos >= current_xpos && next_ypos == current_ypos)
                radiansToWaypoint = 0.0;
            }

            targetDeltaOrientation = radiansToWaypoint - current_orientation;
            targetDeltaRightDrivePosition = targetDeltaLeftDrivePosition = (int) Math.round(WHEEL_TO_CENTRE_DISTANCE*targetDeltaOrientation*TICKS_PER_WHEEL_MM);

            if (targetDeltaOrientation < 0) {
                targetDeltaRightDrivePosition *= -1;
            }

            else {
                targetDeltaLeftDrivePosition *= -1;
            }

            //Rotates the robot towards the next point.
            rightDrive.setTargetPosition((int) (Math.round(targetDeltaRightDrivePosition + current_rightDrivePosition)));
            leftDrive.setTargetPosition((int) (Math.round(targetDeltaLeftDrivePosition + current_leftDrivePosition)));

            while (rightDrive.isBusy() || leftDrive.isBusy()) {
                updateCurrentOdometry();
                updatePreviousOdometry();
            }

            rightDrive.setTargetPosition((int) Math.round(current_rightDrivePosition + (remainingDistanceToWaypoint*TICKS_PER_WHEEL_MM)));
            leftDrive.setTargetPosition((int) Math.round(current_leftDrivePosition + (remainingDistanceToWaypoint*TICKS_PER_WHEEL_MM)));

            updateCurrentOdometry();
            updatePreviousOdometry();
        }

        //rightDrive.setVelocity(rightDriveVelocity);
        //leftDrive.setVelocity(leftDriveVelocity);
    }

    private void updatePreviousOdometry() {
        previous_rightDrivePosition = current_rightDrivePosition;
        previous_leftDrivePosition = current_leftDrivePosition;
    }

    //Everything within runOpMode() is runtime code.
    @Override
    public void runOpMode() {
        //HARDWARE MAPPINGS:
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");

        //ENCODER INITIALIZATION:
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setTargetPosition(0);
        leftDrive.setTargetPosition(0);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Running");
        waitForStart();

        current_orientation = 0;
        current_xpos = 0;
        current_ypos = 0;
        totalDistanceTravelled = 0;

        current_rightDrivePosition = rightDrive.getCurrentPosition();
        current_leftDrivePosition = leftDrive.getCurrentPosition();
        previous_rightDrivePosition = rightDrive.getCurrentPosition();
        previous_leftDrivePosition = leftDrive.getCurrentPosition();

        //Main runtime loop:
        /*
        if (opModeIsActive()) {
            rightDrive.setVelocity(rightDriveVelocity);
            leftDrive.setVelocity(leftDriveVelocity);

            rightDrive.setPower(rightDrivePower);
            leftDrive.setPower(leftDrivePower);

            rightDrive.setTargetPosition((int) (500*TICKS_PER_WHEEL_MM));
            leftDrive.setTargetPosition((int) (500*TICKS_PER_WHEEL_MM));
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
         */

        while (opModeIsActive()) {

            updateCurrentOdometry();
            updateDriveControls();
            updatePreviousOdometry();

            if (stopRuntime) {
                stop();
            }

            telemetry.addData("X Position", current_xpos);
            telemetry.addData("Y Position", current_ypos);
            telemetry.addData("Orientation in Degrees", Math.toDegrees(current_orientation));
            telemetry.addData("Total Distance Travelled", totalDistanceTravelled);
            telemetry.addData("Target Delta Right Drive Position", targetDeltaRightDrivePosition);
            telemetry.addData("Target Delta Left Drive Position", targetDeltaLeftDrivePosition);
            telemetry.addData("Target Delta Orientation", targetDeltaOrientation);
            telemetry.update();
        }
    }
}
