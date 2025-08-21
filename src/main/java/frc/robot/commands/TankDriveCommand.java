package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveTankSubsystem;

public class TankDriveCommand extends Command {
    private final SwerveTankSubsystem drive;
    private final XboxController controller;

    public TankDriveCommand(SwerveTankSubsystem subsystem, XboxController controller) {
        this.drive = subsystem;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double left = -controller.getLeftY()*0.5;
        double right = -controller.getRightY()*0.5;

        // deadband
        if (Math.abs(left) < 0.1) left = 0;
        if (Math.abs(right) < 0.1) right = 0;

        drive.setLeft(left);
        drive.setRight(right);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
