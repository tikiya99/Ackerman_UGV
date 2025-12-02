#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include "DrivingController.h"
#include "MotorDriver.h"
#include "SteeringController.h"
#include "pin_config.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ========================================
// Global Objects
// ========================================

// Motor drivers
MotorDriver steeringMotor(STEERING_LPWM_PIN, STEERING_RPWM_PIN,
                          STEERING_LPWM_CHANNEL, STEERING_RPWM_CHANNEL);
MotorDriver drivingMotor(DRIVING_LPWM_PIN, DRIVING_RPWM_PIN,
                         DRIVING_LPWM_CHANNEL, DRIVING_RPWM_CHANNEL);

// Controllers
SteeringController steeringController(steeringMotor);
DrivingController drivingController(drivingMotor);

// micro-ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Subscribers
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// Publishers
rcl_publisher_t status_publisher;
std_msgs__msg__String status_msg;

char status_buffer[256];

// OLED Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
bool oledFound = false;
bool gy87Found = false;

// Timing
unsigned long lastControlUpdate = 0;
unsigned long lastStatusPublish = 0;
const unsigned long CONTROL_PERIOD_MS = 10; // 100Hz control loop
const unsigned long STATUS_PERIOD_MS = 100; // 10Hz status publishing

// System state
bool systemInitialized = false;
bool emergencyStopActive = false;

// ========================================
// Function Declarations
// ========================================

void cmdVelCallback(const void *msgin);
void publishStatus();
void performCalibration();
void updateControllers();
void checkErrorStates();

bool setupMicroROS();
void updateDisplay();
void displayError(const char *error);

// ========================================
// micro-ROS Callback
// ========================================

/**
 * @brief Callback for cmd_vel topic
 * Converts Twist message to Ackerman steering control
 */
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  if (emergencyStopActive) {
    return; // Ignore commands during emergency stop
  }

  // Extract linear and angular velocities
  float linearVel = msg->linear.x;   // m/s
  float angularVel = msg->angular.z; // rad/s

  // Set driving velocity
  drivingController.setVelocity(linearVel);

  // Convert angular velocity to steering angle (simplified Ackerman)
  // For low-speed robots: steering_angle ≈ atan(angular_vel * wheelbase /
  // linear_vel) Using simplified mapping: small angular velocities map to
  // steering angles This assumes wheelbase and kinematic model will be tuned
  // during testing

  float steeringAngle = 0.0;
  if (abs(linearVel) > 0.01) {
    // Estimate steering angle from angular/linear velocity ratio
    // Adjust the scaling factor based on your UGV's geometry
    const float WHEELBASE = 0.4; // meters (adjust to your UGV)
    steeringAngle = atan(angularVel * WHEELBASE / linearVel) * (180.0 / PI);
  } else if (abs(angularVel) > 0.01) {
    // Pure rotation - use maximum steering
    steeringAngle =
        (angularVel > 0) ? MAX_STEERING_ANGLE_DEG : MIN_STEERING_ANGLE_DEG;
  }

  // Apply steering angle
  steeringController.setTargetAngle(steeringAngle);
}

// ========================================
// Setup Functions
// ========================================

/**
 * @brief Setup micro-ROS
 */
bool setupMicroROS() {
  // Setup serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  delay(2000); // Wait for connection

  allocator = rcl_get_default_allocator();

  // Create init options
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node, "ugv_esp32_node", "", &support) !=
      RCL_RET_OK) {
    return false;
  }

  // Create cmd_vel subscriber
  if (rclc_subscription_init_default(
          &cmd_vel_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel") != RCL_RET_OK) {
    return false;
  }

  // Create status publisher
  if (rclc_publisher_init_default(
          &status_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/ugv/status") != RCL_RET_OK) {
    return false;
  }

  // Create executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
      RCL_RET_OK) {
    return false;
  }

  // Add subscription to executor
  if (rclc_executor_add_subscription(&executor, &cmd_vel_subscriber,
                                     &cmd_vel_msg, &cmdVelCallback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  // Initialize status message
  status_msg.data.data = status_buffer;
  status_msg.data.capacity = sizeof(status_buffer);

  return true;
}

/**
 * @brief Perform steering calibration
 */
void performCalibration() {
  Serial.println("Starting steering calibration...");

  // Set instance pointer for ISR
  SteeringController::instance_ = &steeringController;

  if (steeringController.calibrate()) {
    Serial.println("Calibration successful!");
    systemInitialized = true;
  } else {
    Serial.println("Calibration failed!");
    systemInitialized = false;
  }
}

// ========================================
// Main Control Functions
// ========================================

/**
 * @brief Update all controllers
 */
void updateControllers() {
  steeringController.update();
  drivingController.update();
}

/**
 * @brief Check for error states
 */
void checkErrorStates() {
  if (steeringController.isEmergencyStop()) {
    if (!emergencyStopActive) {
      emergencyStopActive = true;
      drivingController.stop();
      Serial.println("EMERGENCY STOP: Limit switch activated!");
    }
  }
}

/**
 * @brief Publish status message
 */
void publishStatus() {
  if (!systemInitialized) {
    return;
  }

  // Format status string
  snprintf(status_buffer, sizeof(status_buffer),
           "Angle:%.2f Target:%.2f Vel:%.2f Enc:%ld E-Stop:%d",
           steeringController.getCurrentAngle(),
           steeringController.getTargetAngle(),
           drivingController.getCurrentVelocity(),
           steeringController.getEncoderCount(), emergencyStopActive ? 1 : 0);

  status_msg.data.size = strlen(status_buffer);

  rcl_ret_t ret = rcl_publish(&status_publisher, &status_msg, NULL);
  if (ret != RCL_RET_OK) {
    // Log error if needed (silent for now to avoid serial spam)
  }
}

// ========================================
// Display Functions
// ========================================

void displayError(const char *error) {
  if (!oledFound)
    return;

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ERROR!"));
  display.setTextSize(1);
  display.println(error);
  display.display();
}

void updateDisplay() {
  if (!oledFound)
    return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (emergencyStopActive) {
    displayError("EMERGENCY STOP");
    return;
  }

  display.println(F("UGV STATUS: READY"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 15);
  display.print(F("Ang: "));
  display.print(steeringController.getCurrentAngle(), 1);
  display.print(F(" / "));
  display.println(steeringController.getTargetAngle(), 1);

  display.print(F("Vel: "));
  display.println(drivingController.getCurrentVelocity(), 2);

  display.print(F("Enc: "));
  display.println(steeringController.getEncoderCount());

  display.print(F("GY87: "));
  display.println(gy87Found ? "OK" : "NC");

  display.display();
}

// ========================================
// Arduino Setup and Loop
// ========================================

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    oledFound = false;
  } else {
    Serial.println(F("OLED Found"));
    oledFound = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Booting UGV..."));
    display.display();
  }

  // Check for GY87 (MPU6050 at 0x68)
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    Serial.println(F("GY87 (MPU6050) Found"));
    gy87Found = true;
  } else {
    Serial.println(F("GY87 Not Found"));
    gy87Found = false;
  }

  delay(1000);

  Serial.println("UGV ESP32 Controller Starting...");

  // Initialize controllers
  steeringController.begin();
  drivingController.begin();

  // Check if already calibrated
  if (steeringController.getCalibrationState() !=
      SteeringController::CALIBRATED) {
    Serial.println("No calibration found. Starting calibration...");
    performCalibration();
  } else {
    Serial.println("Calibration loaded from memory.");
    SteeringController::instance_ = &steeringController;
    systemInitialized = true;
  }

  // Setup micro-ROS
  Serial.println("Initializing micro-ROS...");
  while (!setupMicroROS()) {
    Serial.println("Failed to initialize micro-ROS. Retrying...");
    delay(1000);
  }
  Serial.println("micro-ROS initialized successfully!");

  Serial.println("System ready!");
}

void loop() {
  // Spin executor to handle callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  // Control loop
  unsigned long currentTime = millis();

  if (currentTime - lastControlUpdate >= CONTROL_PERIOD_MS) {
    lastControlUpdate = currentTime;

    checkErrorStates();

    if (systemInitialized && !emergencyStopActive) {
      updateControllers();
    }
  }

  // Status publishing
  if (currentTime - lastStatusPublish >= STATUS_PERIOD_MS) {
    lastStatusPublish = currentTime;

    publishStatus();
    updateDisplay();
  }
}