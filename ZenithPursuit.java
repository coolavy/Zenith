package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ZenithPursuit extends Command {
    private final Drive drive;
    private final Vision vision;

    // PID Controllers
    // Tune these: Pulled from old 2025 code.
    private final PIDController forwardController = new PIDController(2.5, 0, 0);
    private final PIDController strafeController  = new PIDController(2.5, 0, 0);
    private final PIDController turnController    = new PIDController(3.5, 0, 0.1);

    public ZenithPursuit(Drive drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // We want the ball to be 0.4 meters in front of the robot center 
        // (Adjust this based on where your intake is or wherever you mount it)
        forwardController.setSetpoint(0.4); 
        strafeController.setSetpoint(0.0);
        turnController.setSetpoint(0.0);
        
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            // Spin at 1.5 radians per second until a ball enters the FOV
            drive.runVelocity(new ChassisSpeeds(0, 0, 1.5));
            return;
        }

        // Get Robot-Relative coordinates from the Orange Pi (Meters)
        double xMeters = vision.getTargetX(); // Distance Forward
        double yMeters = vision.getTargetY(); // Distance Lateral
        double yawRad  = Math.toRadians(vision.getTargetYaw());

        // Calculate Velocities: forwardVel uses X, strafeVel uses Y
        double vx = forwardController.calculate(xMeters);
        double vy = strafeController.calculate(yMeters);
        double omega = turnController.calculate(yawRad);

        // Limit maximum speeds for safety during testing
        // During production screw it.
        vx = Math.max(-1.5, Math.min(1.5, vx));
        vy = Math.max(-1.5, Math.min(1.5, vy));

        // DRIVE (Robot Relative)
        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we are close enough to the ball or the cluster center.
        return vision.hasTarget() && vision.getDistance() < 0.45;
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds()); // Stop the robot
    }
}
