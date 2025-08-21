package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveTankSubsystem;
import frc.robot.commands.TankDriveCommand;

public class RobotContainer {
    private final XboxController controller = new XboxController(0);
    private final SwerveTankSubsystem drive = new SwerveTankSubsystem();

    public RobotContainer() {
        drive.setDefaultCommand(new TankDriveCommand(drive, controller));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(controller, XboxController.Button.kA.value).onChange(new InstantCommand(() -> drive.stop()));
    }
    public Command getAutonomousCommand() {
        return null;
    }
}
