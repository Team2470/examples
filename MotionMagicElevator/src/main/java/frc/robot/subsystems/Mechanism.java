package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Mechanism extends SubsystemBase {
    private final TalonFX m_motor;
    private final TalonFX m_motorFollower;

    public Mechanism() {
        //
        // Base Motor configuration: Brake Mode, Current limits, etc...
        //
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = 125;

        //
        // Setup Follower motor
        //
        m_motorFollower = new TalonFX(MechanismConstants.kMotorFollowerID);
        m_motorFollower.getConfigurator().apply(config);
        m_motorFollower.optimizeBusUtilization();
        m_motorFollower.setControl(new Follower(MechanismConstants.kMotorID, true));

        //
        // Apply add extra leader configuration on top of the base config
        // - Any control related settings for PID, Motion Magic, Remote Sensors, etc.. 
        // - Soft limits should only be applied to the leader motor, if they are applied to the follower
        //   it may stop moving if their built in encoders are not in sync.
        //
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 80;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5;

        m_motor = new TalonFX(MechanismConstants.kMotorID);
        m_motor.getConfigurator().apply(config);

        // We want to read position data from the leader motor
        m_motor.getPosition().setUpdateFrequency(50);

        // These 3 are needed for the follower motor to work
        m_motor.getDutyCycle().setUpdateFrequency(50);
        m_motor.getMotorVoltage().setUpdateFrequency(50);
        m_motor.getTorqueCurrent().setUpdateFrequency(50);
        m_motor.optimizeBusUtilization();    
    }

    @Override
    public void periodic() {
        // Just have the motors run when the robot is enabled!
        if (DriverStation.isEnabled()) {
            m_motor.setVoltage(1);
        }
    }
}
