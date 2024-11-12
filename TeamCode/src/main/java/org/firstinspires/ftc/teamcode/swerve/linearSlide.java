package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class linearSlide extends LinearOpMode {

    private DcMotor slideMotor;

    private static final double RobotBase_to_LS = 219.160;  // Distance from the front of the robot to the slide base
    private static final double Max_Extension = 42; // Max safe extension distance from slide base

    // PID constants
    private double kP = 0.01;
    private double kI = 0.0001;
    private double kD = 0.001;

    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private int targetPosition = 0;  // Starting at initial position (adjust)

    @Override
    public void runOpMode() {
        // Initialize motor
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            // Manual control for setting positions
            if (gamepad1.dpad_up) {
                targetPosition += 0;  // Increase position
            } else if (gamepad1.dpad_down) {
                targetPosition -= 0;  // Decrease position
            }

            // Clamp target position to a safe range (adjust limits as needed)
            targetPosition = Math.max(0, Math.min(targetPosition, 360));

            // Calculate current forward reach
            double currentExtension = slideMotor.getCurrentPosition() + RobotBase_to_LS;

            // Ensure the slide does not exceed max forward extension
            if ((targetPosition > slideMotor.getCurrentPosition() && currentExtension < Max_Extension) ||
                    (targetPosition < slideMotor.getCurrentPosition() && currentExtension > RobotBase_to_LS)) {

                double power = calculatePID(targetPosition, slideMotor.getCurrentPosition());
                slideMotor.setPower(power);
            } else {
                slideMotor.setPower(0);  // Stop motor if exceeding extension limits (failsafe)
            }

            // Update the slide motor power using PID
            double power = calculatePID(targetPosition, slideMotor.getCurrentPosition());
            slideMotor.setPower(power);

            // Telemetry for debugging
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", slideMotor.getCurrentPosition());
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }

    private double calculatePID(int target, int current) {
        // Calculate error
        double error = target - current;

        // Proportional term
        double pTerm = kP * error;

        // Integral term
        integral += error;
        double iTerm = kI * integral;

        // Derivative term
        double dTerm = kD * (error - lastError);
        lastError = error;

        // PID output
        double output = pTerm + iTerm + dTerm;

        // Limit the output to motor range [-1, 1]
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }
}
