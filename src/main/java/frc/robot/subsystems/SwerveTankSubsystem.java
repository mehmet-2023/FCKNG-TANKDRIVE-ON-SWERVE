package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveTankSubsystem extends SubsystemBase {
    private final SparkMax flDrive = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax frDrive = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax blDrive = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax brDrive = new SparkMax(4, MotorType.kBrushless);

    public SwerveTankSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(false);

        flDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        frDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        blDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        brDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void setLeft(double power) {
        flDrive.set(power);
        blDrive.set(power);
    }

    public void setRight(double power) {
        frDrive.set(power);
        brDrive.set(power);
    }

    public void stop() {
        setLeft(0);
        setRight(0);
    }
}
