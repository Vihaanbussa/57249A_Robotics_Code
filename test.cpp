#include "main.h"
#include "lemlib/api.hpp"         // For chassis functions, etc.
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/ext_adi.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include <cstdio>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>

//---------------------------------------------------------------------
// Global Variables and Helper Functions
//---------------------------------------------------------------------
int currstate = 0;
int states[4] = {0, 15, 25, 130};
void nextState() {
  currstate++;
  if(currstate >= 4) currstate = 0;
  pros::lcd::print(5, "State set to: %d", states[currstate]);
}

//---------------------------------------------------------------------
// Robot Configuration
//---------------------------------------------------------------------
pros::MotorGroup left_motor_group({-2, -3, -4});    // Left motors (ports 2,3,4)
pros::MotorGroup right_motor_group({16, 15, 14});     // Right motors (ports 16,15,14)
pros::Motor ladybrown(5);
pros::MotorGroup intake({-20, 10});
pros::Motor frontstage(10);
pros::Motor hooks(-20);
pros::adi::DigitalOut clamp('E');
pros::adi::DigitalOut doink('C');
pros::adi::DigitalOut pistonLift('B');
pros::Imu imu(17);
pros::Rotation horizontalEnc(13);
pros::Rotation verticalEnc(-18);
pros::Rotation wallrotational(19);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, 1.8);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEnc, lemlib::Omniwheel::NEW_275, -1.5);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical color(12);
pros::adi::AnalogIn potentiometer('A');
// Two distance sensors (return cm): front sensor on port 8, right sensor on port 9.
pros::Distance distanceSensorFront(8);
pros::Distance distanceSensorRight(9);

//---------------------------------------------------------------------
// Chassis Configuration (lemlib)
//---------------------------------------------------------------------
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 11.3125, 
                              lemlib::Omniwheel::NEW_325, 480, 2);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);
lemlib::ControllerSettings lateral_controller(5.55, 0.05, 20, 7, 1, 1400, 0, 700, 80);
lemlib::ControllerSettings angular_controller(3, 0, 20, 10, 1, 100, 5, 500, 80);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, 
                        sensors, &throttle_curve, &steer_curve);

//---------------------------------------------------------------------
// Autonomous Route Structures
//---------------------------------------------------------------------

// Pose structure: x, y in inches; theta in degrees.
struct PoseLocal {
    double x;
    double y;
    double theta;
};

// A route segment holds a target pose and an optional event function.
struct RouteSegment {
    PoseLocal target;
    std::function<void()> event; // Can be nullptr if no event.
};

//---------------------------------------------------------------------
// Monte Carlo Localization (MCL) Class
//---------------------------------------------------------------------
// This class holds a bunch of particles (each a guess of our pose) and updates them with noisy odometry.
// The average of the particles is our estimated pose. We can also reset (recenter) the particles.
class MonteCarloLocalizationLocal {
public:
    MonteCarloLocalizationLocal(int numParticles = 50) {
        for (int i = 0; i < numParticles; i++) {
            Particle p = { randomInRange(-50.0, 50.0),
                           randomInRange(-50.0, 50.0),
                           randomInRange(0, 360.0),
                           1.0 };
            particles.push_back(p);
        }
    }
    void update(double delta_x, double delta_y, double delta_theta) {
        for (auto &p : particles) {
            p.x += delta_x + randomNoise();
            p.y += delta_y + randomNoise();
            p.theta += delta_theta + randomNoise(); // in degrees
        }
    }
    PoseLocal getEstimatedPose() {
        double sum_x = 0, sum_y = 0, sum_theta = 0;
        for (auto &p : particles) {
            sum_x += p.x;
            sum_y += p.y;
            sum_theta += p.theta;
        }
        int count = particles.size();
        return { sum_x / count, sum_y / count, sum_theta / count };
    }
    void resetToPose(const PoseLocal &newPose) {
        particles.clear();
        for (int i = 0; i < 50; i++) {
            Particle p = { newPose.x + randomNoise() * 0.5,
                           newPose.y + randomNoise() * 0.5,
                           newPose.theta + randomNoise() * 0.5,
                           1.0 };
            particles.push_back(p);
        }
    }
private:
    struct Particle {
        double x;
        double y;
        double theta;
        double weight;
    };
    std::vector<Particle> particles;
    double randomNoise() {
        return ((std::rand() % 100) / 1000.0) - 0.05;
    }
    double randomInRange(double min, double max) {
        return min + (max - min) * ((double)std::rand() / RAND_MAX);
    }
};

//---------------------------------------------------------------------
// Ramsete Controller Class
//---------------------------------------------------------------------
// This calculates the linear (v) and angular (omega) velocities to drive from our current pose to a target pose.
// All angle math is done in radians so we convert our degrees accordingly.
class RamseteControllerLocal {
public:
    RamseteControllerLocal(double b, double zeta) : b(b), zeta(zeta) {}
    void compute(const PoseLocal &current, const PoseLocal &desired,
                 double v_d, double omega_d,
                 double &v, double &omega) {
        double r_current = current.theta * M_PI / 180.0;
        double r_desired = desired.theta * M_PI / 180.0;
        double dx = desired.x - current.x;
        double dy = desired.y - current.y;
        double error_x = cos(r_current) * dx + sin(r_current) * dy;
        double error_y = -sin(r_current) * dx + cos(r_current) * dy;
        double error_theta = r_desired - r_current;
        double k = 2 * zeta * std::sqrt(omega_d * omega_d + b * v_d * v_d);
        v = v_d * cos(error_theta) + k * error_x;
        omega = omega_d + k * error_theta + b * v_d * sinc(error_theta) * error_y;
    }
private:
    double b, zeta;
    double sinc(double x) { return (fabs(x) < 1e-6) ? 1.0 : sin(x) / x; }
};

//---------------------------------------------------------------------
// Autonomous Route: Red Negative Route using MCL + Ramsete + Dual Sensor Reset
//---------------------------------------------------------------------
// We break our route into 13 segments as described in our writeup.
void mclRamseteRouteCustom2() {
    // Set the starting pose to (0, 0, 320°).
    PoseLocal initialPose = { 0.0, 0.0, 320.0 };
    MonteCarloLocalizationLocal mcl(50);
    mcl.resetToPose(initialPose);
    
    // Define our route as a vector of segments.
    std::vector<RouteSegment> route = {
        // Segment 0: Move to (-8.5, 10, 320). Then trigger nextState() calls.
        { { -8.5, 10.0, 320.0 },
          [](){
              nextState();
              nextState();
              pros::delay(750);
              nextState();
          }
        },
        // Segment 1: Turn to 315 (stay at (-8.5, 10)).
        { { -8.5, 10.0, 315.0 }, nullptr },
        // Segment 2: Move to (7, -10, 315).
        { { 7.0, -10.0, 315.0 }, nullptr },
        // Segment 3: Move to (7, -25, 315) then delay 250ms and release clamp.
        { { 7.0, -25.0, 315.0 },
          [](){
              pros::delay(250);
              clamp.set_value(false);
          }
        },
        // Segment 4: Turn to 130 (position stays (7, -25)).
        { { 7.0, -25.0, 130.0 }, nullptr },
        // Segment 5: Move to (18, -32, 130) and set intakespin true.
        { { 18.0, -32.0, 130.0 },
          [](){
              intakespin = true;
          }
        },
        // Segment 6: Turn to 109 (at (18, -32)).
        { { 18.0, -32.0, 109.0 }, nullptr },
        // Segment 7: Move to (27, -35, 109).
        { { 27.0, -35.0, 109.0 }, nullptr },
        // Segment 8: Move to (7, -17, 109) in reverse.
        { { 7.0, -17.0, 109.0 }, nullptr },
        // Segment 9: Turn to 90 (at (7, -17)).
        { { 7.0, -17.0, 90.0 }, nullptr },
        // Segment 10: Move to (26, -19, 90).
        { { 26.0, -19.0, 90.0 }, nullptr },
        // Segment 11: Turn to 270 (at (26, -19)).
        { { 26.0, -19.0, 270.0 }, nullptr },
        // Segment 12: Move to (0, -19, 270) and trigger nextState() calls.
        { { 0.0, -19.0, 270.0 },
          [](){
              nextState();
              nextState();
          }
        }
    };
    
    // Set desired speeds.
    double desiredLinearVel = 8.0;    // inches/sec
    double desiredAngularVel = 0.0;   // rad/sec (feedforward)
    double trackWidth = 12.0;         // inches between wheels
    
    // Create a Ramsete controller.
    double controller_b = 2.0;
    double controller_zeta = 0.7;
    RamseteControllerLocal ramsete(controller_b, controller_zeta);
    
    // Simulated encoder values (in inches).
    double prevLeftEnc = 0.0, prevRightEnc = 0.0;
    
    // For each segment in our route...
    for (size_t segIndex = 0; segIndex < route.size(); segIndex++) {
        PoseLocal targetPose = route[segIndex].target;
        int iterations = 0;
        while (iterations < 500) {
            iterations++;
            // Simulate encoder increments.
            double currLeftEnc = prevLeftEnc + (std::rand() % 10) / 10.0;
            double currRightEnc = prevRightEnc + (std::rand() % 10) / 10.0;
            double delta_left = currLeftEnc - prevLeftEnc;
            double delta_right = currRightEnc - prevRightEnc;
            prevLeftEnc = currLeftEnc;
            prevRightEnc = currRightEnc;
            double delta_distance = (delta_left + delta_right) / 2.0;
            double delta_theta = (delta_right - delta_left) / trackWidth; // in degrees
            
            // Compute global displacement using our current reference pose.
            double r_initialTheta = initialPose.theta * M_PI / 180.0;
            double delta_x = delta_distance * cos(r_initialTheta);
            double delta_y = delta_distance * sin(r_initialTheta);
            
            // Update our MCL particles.
            mcl.update(delta_x, delta_y, delta_theta);
            PoseLocal currentPose = mcl.getEstimatedPose();
            
            // Check if we are within threshold of the target.
            double error_distance = sqrt(pow(targetPose.x - currentPose.x, 2) +
                                         pow(targetPose.y - currentPose.y, 2));
            double error_angle = fabs(targetPose.theta - currentPose.theta);
            if (error_distance < 2.0 && error_angle < 5.0) {
                pros::delay(100); // Wait for sensor stability.
                // --- Dual Distance Sensor Reset ---
                // Read sensors (cm to inches conversion: divide by 2.54).
                double measuredFront = distanceSensorFront.get() / 2.54;
                double measuredRight = distanceSensorRight.get() / 2.54;
                PoseLocal correctedPose = currentPose;
                correctedPose.x = measuredRight;  // Right sensor gives x
                correctedPose.y = measuredFront;   // Front sensor gives y
                mcl.resetToPose(correctedPose);
                initialPose = correctedPose; // Update our reference pose.
                // Run any event function for this segment.
                if (route[segIndex].event)
                    route[segIndex].event();
                break; // Move to the next segment.
            }
            
            // Compute drive commands using the Ramsete controller.
            double v_command, omega_command;
            ramsete.compute(currentPose, targetPose, desiredLinearVel, desiredAngularVel,
                            v_command, omega_command);
            
            double leftSpeed = v_command - (omega_command * trackWidth / 2.0);
            double rightSpeed = v_command + (omega_command * trackWidth / 2.0);
            double leftPercent = leftSpeed * 100;
            double rightPercent = rightSpeed * 100;
            
            left_motor_group.move(leftPercent);
            right_motor_group.move(rightPercent);
            pros::delay(20);
        }
    }
    left_motor_group.stop();
    right_motor_group.stop();
}

//---------------------------------------------------------------------
// Competition Functions
//---------------------------------------------------------------------
void autonomous() {
    // For our autonomous, we run our red negative route.
    mclRamseteRouteCustom2();
}
