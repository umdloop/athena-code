#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <string>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "talon_ros2_control/talon_control.h"

// ── Motor initialization ─────────────────────────────────────────────────────

void applyStatusFramePeriods(TalonSRX * motor, const MotorConfig & config)
{
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General,
    static_cast<uint8_t>(config.status_1_general_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
    static_cast<uint8_t>(config.status_2_feedback0_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature,
    static_cast<uint8_t>(config.status_3_quadrature_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_4_AinTempVbat,
    static_cast<uint8_t>(config.status_4_aintempvbat_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_6_Misc,
    static_cast<uint8_t>(config.status_6_misc_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_7_CommStatus,
    static_cast<uint8_t>(config.status_7_commstatus_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_8_PulseWidth,
    static_cast<uint8_t>(config.status_8_pulsewidth_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_9_MotProfBuffer,
    static_cast<uint8_t>(config.status_9_motprofbuffer_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_10_Targets,
    static_cast<uint8_t>(config.status_10_targets_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_12_Feedback1,
    static_cast<uint8_t>(config.status_12_feedback1_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_13_Base_PIDF0,
    static_cast<uint8_t>(config.status_13_base_pidf0_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_14_Turn_PIDF1,
    static_cast<uint8_t>(config.status_14_turn_pidf1_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_15_FirmwareApiStatus,
    static_cast<uint8_t>(config.status_15_firmwareapistatus_period_ms),
    config.status_frame_timeout_ms);
  motor->SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_17_Targets1,
    static_cast<uint8_t>(config.status_17_targets1_period_ms),
    config.status_frame_timeout_ms);
}

void applyControlFramePeriods(TalonSRX * motor, const MotorConfig & config)
{
  motor->SetControlFramePeriod(
    ctre::phoenix::motorcontrol::ControlFrame::Control_3_General,
    config.control_3_general_period_ms);
  motor->SetControlFramePeriod(
    ctre::phoenix::motorcontrol::ControlFrame::Control_4_Advanced,
    config.control_4_advanced_period_ms);
  motor->ChangeMotionControlFramePeriod(config.control_6_motprofaddtrajpoint_period_ms);
}

void initMotor(TalonSRX * motor, const MotorConfig & config, const std::string & can_interface) {
  // Wake up the CAN bus (send a dummy frame so the kernel driver is active)
  std::string cmd = "cansend " + can_interface + " 123#00000000";
  std::system(cmd.c_str());

  motor->ConfigFactoryDefault(config.config_timeout_ms);
  motor->SetInverted(config.inverted);

  motor->ConfigSelectedFeedbackSensor(
    ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder,
    0, config.config_timeout_ms);
  motor->SetSensorPhase(config.sensor_phase);

  motor->Config_kP(0, config.kP, config.config_timeout_ms);
  motor->Config_kI(0, config.kI, config.config_timeout_ms);
  motor->Config_kD(0, config.kD, config.config_timeout_ms);
  motor->Config_kF(0, config.kF, config.config_timeout_ms);
  applyStatusFramePeriods(motor, config);
  applyControlFramePeriods(motor, config);
}

// ── Conversion helpers ───────────────────────────────────────────────────────

double convertRevtoTalonUnits(double rev, const MotorConfig & config) {
  double counts_per_revolution = config.encoder_resolution * config.gear_ratio;
  return rev * counts_per_revolution;
}

double convertTalonUnitstoRev(double counts, const MotorConfig & config) {
  double counts_per_revolution = config.encoder_resolution * config.gear_ratio;
  return counts / counts_per_revolution;
}

double convertRevToDistance(double rev, const MotorConfig & config) {
  return rev * config.distance_per_rev;
}

double convertDistanceToRev(double distance, const MotorConfig & config) {
  return distance / config.distance_per_rev;
}

// ── Position getters ─────────────────────────────────────────────────────────

float getPositionTalonUnits(TalonSRX * motor) {
  return motor->GetSelectedSensorPosition(0);
}

float getPositionRevolutions(TalonSRX * motor, const MotorConfig & config) {
  return static_cast<float>(
    convertTalonUnitstoRev(motor->GetSelectedSensorPosition(0), config));
}

float getPositionDistance(TalonSRX * motor, const MotorConfig & config) {
  // Output revolutions -> meters (for prismatic joints)
  return getPositionRevolutions(motor, config) * static_cast<float>(config.distance_per_rev);
}

float getPositionRadians(TalonSRX * motor, const MotorConfig & config) {
  // Output revolutions -> radians (for revolute joints)
  return getPositionRevolutions(motor, config) * static_cast<float>(2.0 * M_PI);
}

// ── Velocity getters ─────────────────────────────────────────────────────────

float getVelocityRPM(TalonSRX * motor, const MotorConfig & config) {
  // TalonSRX reports velocity in encoder units per 100 ms
  float units_per_minute = motor->GetSelectedSensorVelocity(0) * 600.0f;
  return static_cast<float>(convertTalonUnitstoRev(units_per_minute, config));
}

float getLinearVelocity(TalonSRX * motor, const MotorConfig & config) {
  // Talon units/100ms -> units/s -> rev/s -> m/s
  float units_per_second = motor->GetSelectedSensorVelocity(0) * 10.0f;
  float rev_per_second = static_cast<float>(convertTalonUnitstoRev(units_per_second, config));
  return rev_per_second * static_cast<float>(config.distance_per_rev);
}

float getAngularVelocity(TalonSRX * motor, const MotorConfig & config) {
  // Talon units/100ms -> units/s -> rev/s -> rad/s
  float units_per_second = motor->GetSelectedSensorVelocity(0) * 10.0f;
  float rev_per_second = static_cast<float>(convertTalonUnitstoRev(units_per_second, config));
  return rev_per_second * static_cast<float>(2.0 * M_PI);
}

// ── Motor command setters ────────────────────────────────────────────────────

void setDutyCycle(TalonSRX * motor, double dutyCycle, int ms) {
  motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, dutyCycle);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);
}

float setVelocityFromLinearVelocity(TalonSRX * motor, double vel_m_per_s, int ms, const MotorConfig & config) {
  // m/s -> m/100ms -> rev/100ms -> talon units/100ms
  double vel_m_per_100ms = vel_m_per_s / 10.0;
  double rev_per_100ms = convertDistanceToRev(vel_m_per_100ms, config);
  double talon_units_per_100ms = convertRevtoTalonUnits(rev_per_100ms, config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, talon_units_per_100ms);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units_per_100ms);
}

float setVelocityFromAngularVelocity(TalonSRX * motor, double vel_rad_per_s, int ms, const MotorConfig & config) {
  // rad/s -> rev/s -> rev/100ms -> talon units/100ms
  double rev_per_100ms = vel_rad_per_s / (2.0 * M_PI * 10.0);
  double talon_units_per_100ms = convertRevtoTalonUnits(rev_per_100ms, config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, talon_units_per_100ms);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units_per_100ms);
}

float setPositionFromDisplacement(TalonSRX * motor, double displacement_m, int ms, const MotorConfig & config) {
  // m -> rev -> talon units
  double talon_units = convertRevtoTalonUnits(convertDistanceToRev(displacement_m, config), config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, talon_units);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units);
}

float setPositionFromJointCommand(TalonSRX * motor, double position_rad, int ms, const MotorConfig & config) {
  // rad -> rev -> talon units
  double talon_units = convertRevtoTalonUnits(position_rad / (2.0 * M_PI), config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, talon_units);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units);
}

void stopMotor(TalonSRX * motor, int ms) {
  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0.0);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);
}
