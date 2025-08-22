package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.enums.ControlType;
import com.revrobotics.spark.CANSparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveTankSubsystem extends SubsystemBase {
    private final SparkMax flDrive = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax frDrive = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax blDrive = new SparkMax(7, MotorType.kBrushless);
    private final SparkMax brDrive = new SparkMax(5, MotorType.kBrushless);

    private final SparkMax flSteer = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax frSteer = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax blSteer = new SparkMax(8, MotorType.kBrushless);
    private final SparkMax brSteer = new SparkMax(6, MotorType.kBrushless);

    private final CANSparkMaxPIDController flPID = flSteer.getPIDController();
    private final CANSparkMaxPIDController frPID = frSteer.getPIDController();
    private final CANSparkMaxPIDController blPID = blSteer.getPIDController();
    private final CANSparkMaxPIDController brPID = brSteer.getPIDController();

    private final double flHome, frHome, blHome, brHome;

    public SwerveTankSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(false);
        config.closedLoop.p(0.1).i(0).d(0).outputRange(-1, 1);

        flDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        frDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        blDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        brDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        flSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        frSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        blSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        brSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        flHome = flSteer.getEncoder().getPosition();
        frHome = frSteer.getEncoder().getPosition();
        blHome = blSteer.getEncoder().getPosition();
        brHome = brSteer.getEncoder().getPosition();

        flPID.setReference(flHome, ControlType.kPosition);
        frPID.setReference(frHome, ControlType.kPosition);
        blPID.setReference(blHome, ControlType.kPosition);
        brPID.setReference(brHome, ControlType.kPosition);
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
