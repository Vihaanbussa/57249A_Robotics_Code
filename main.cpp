#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include "robodash/api.h"
#include "robodash/views/console.hpp"

pros::MotorGroup left_motor_group({-1, 17, -16}); // left motors on ports 1 (reversed), 2 (forwards), and 3 (reversed)
pros::MotorGroup right_motor_group({3, -12, 14}); // right motors on ports 4 (forwards), 5 (reversed), and 6 (forwards)
pros::MotorGroup ladybrown({-20,10});
pros::Motor intake(21);
pros::adi::DigitalOut clamp('A');
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut pistonLift('B');
pros::Imu imu(11);
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversedx
pros::Rotation horizontalEnc(-7);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(19);
pros::Rotation wallrotational(5);
// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.5);
// vertical tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, 2);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

bool clampbool = LOW;
bool pistonliftbool = LOW;
bool doink = LOW;
bool R2_pressed = false;
bool B_pressed = false;
bool idk = false;


// drivetrain setting
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.3125, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              480, // drivetrain rpm is 360
                              0 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(6.9, // proportional gain (kP)
                                              0.02, // integral gain (kI)
                                              8, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              700, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              60 // maximum acceleration (slew
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.5, // proportional gain (kP) 2.5
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              70 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127 
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, //  lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);
bool arm_moving = false;
int setpos = wallrotational.get_position();
void arm_move_load(){
    arm_moving = true;
    while(arm_moving){
        ladybrown.move(30);
        while(wallrotational.get_position()<34000){
            ladybrown.move(30);
            pros::delay(5);
        }
        while(wallrotational.get_position()>36000){
            ladybrown.move(-30);
            pros::delay(5);
        }
        if(wallrotational.get_position()>34000 && wallrotational.get_position()<36000){
            arm_moving = false;
        }
    }
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
    ladybrown.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    ladybrown.move(0); 
}

void arm_move_set(){
    while(wallrotational.get_position()<84000){
        ladybrown.move(70);
        pros::delay(5);
    }
    ladybrown.move(0);
}

void arm_move_return(){
    while(wallrotational.get_position()>12000){
        ladybrown.move(-70);
        pros::delay(5);
    }
    ladybrown.move(0);
}



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

int current_auton_selection = 0;
void autonomous() { 
    switch(current_auton_selection){
        case 0: //redpos
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -28, 2000, {.forwards=false}); //p1
            chassis.turnToHeading(333, 300);
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -18, 2000, {.forwards=false}); //p1






            // chassis.setPose(0, 0, 0);
            // chassis.moveToPoint(0, -28, 2000, {.forwards=false}); //p1
            // chassis.turnToHeading(-333, 300);
            // chassis.moveToPoint(-7.8, -44, 1000, {.forwards=false}); //p2
            // chassis.waitUntilDone();
            // clamp.set_value(true);
            // pros::delay(100);
            // intake.move(-127);
            // pros::delay(200);
            // chassis.moveToPoint(-20, -27, 1000, {.forwards=true, .maxSpeed = 80}); //p3
            // chassis.waitUntilDone();
            // pros::delay(700);
            // clamp.set_value(false);
            // intake.move(127);
            // chassis.moveToPoint(-44.612, -35.767, 1500, {.forwards=false}); //p4
            // chassis.waitUntilDone();
            // clamp.set_value(true);
            // pros::delay(100);
            // pistonLift.set_value(true);
            // intake.move(-127);
            // chassis.moveToPoint(-75, -10, 1800, {.forwards=true, .maxSpeed=70});
            // chassis.waitUntilDone();
            // pistonLift.set_value(false);
            // intake.move(-127);
            // pros::delay(250);
            // chassis.turnToHeading(-30, 500);
            // chassis.moveToPoint(-60, -8.25, 1500, {.forwards=false, .maxSpeed = 60});
            // pros::delay(500);
            // intake.move(-127);
            // chassis.moveToPoint(-65.581, -27.499, 1500, {.forwards=true, .maxSpeed = 70});
            // intake.move(127);
            // pros::delay(75);
            // intake.move(-127);
            break;
        case 1:
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -28, 2000, {.forwards=false}); //p1
            chassis.turnToHeading(333, 300);
            chassis.moveToPoint(7.8, -44, 1000, {.forwards=false, .maxSpeed = 80}); //p2
            chassis.waitUntilDone();
            clamp.set_value(true);
            pros::delay(100);
            intake.move(-127);
            pros::delay(200);
            chassis.moveToPoint(20, -27, 1000, {.forwards=true, .maxSpeed = 80}); //p3
            chassis.waitUntilDone();
            pros::delay(700);
            clamp.set_value(false);
            intake.move(127);
            chassis.moveToPoint(44.612, -35.767, 1500, {.forwards=false}); //p4
            chassis.waitUntilDone();
            clamp.set_value(true);
            pros::delay(100);
            pistonLift.set_value(true);
            intake.move(-127);
            chassis.moveToPoint(75, -10, 1800, {.forwards=true, .maxSpeed=70});
            chassis.waitUntilDone();
            pistonLift.set_value(false);
            intake.move(-127);
            pros::delay(250);
            chassis.turnToHeading(30, 500);
            chassis.moveToPoint(60, -8.25, 1500, {.forwards=false, .maxSpeed = 60});
            pros::delay(500);
            intake.move(-127);
            chassis.moveToPoint(60.581, -27.499, 1500, {.forwards=true, .maxSpeed = 70});
            intake.move(127);
            pros::delay(75);
            intake.move(-127);
            break;
        case 2:
            chassis.moveToPoint(0, -1, 1500, {.maxSpeed = 10});
            intake.move(-127);
            pros::delay(1000);
            chassis.moveToPoint(0, 9, 1500, {.minSpeed = 10});
            chassis.turnToHeading(270, 750);
            chassis.moveToPoint(15, 7, 1500, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
            pros::delay(100);
            clamp.set_value(true);
            pros::delay(400);
            chassis.moveToPoint(20, 28, 1500, {.forwards = true, .minSpeed = 20});
            chassis.moveToPoint(30, 40, 1500, {.forwards = true, .minSpeed = 70});
            chassis.moveToPoint(40, 80, 2000, {.forwards = true});
            pros::delay(800);
            chassis.moveToPoint(48, 55, 1500, {.forwards = true});
            pros::Task task{[] {
                arm_move_load();
            }};
            chassis.moveToPoint(35, 62, 1500, {.forwards = false});
            break;
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::delay(20);
    while (true) {
         // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(7, "pos:%ld", wallrotational.get_position());
        
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        

        //move ladybrown mech
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			ladybrown.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			ladybrown.move(127);
        } else{
            ladybrown.move(0);
        }

        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
		// 	ladybrown.move(127);
        //     if (wallrotational.get_position()  > 211) {
        //         ladybrown.move(0);
        //     }
        // }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            pros::Task task{[] {
                arm_move_load();
            }};
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            if (wallrotational.get_position()<70000){
                pros::Task task{[] {
                    arm_move_set();
                }};
            }
            else {
                pros::Task task{[] {
                    arm_move_return();
                }};
            }
            
        }



        //move intake
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(127);
        } else{
            intake.move(0);
        }

        // mogo mech controls
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
		    if(R2_pressed){
                clamp.set_value(false);
                R2_pressed = false;
            } else{
                clamp.set_value(true);
                R2_pressed = true;
            }
		}

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		    if(B_pressed){
                pistonLift.set_value(false);
                B_pressed = false;
            } else{
                pistonLift.set_value(true);
                B_pressed = true;
            }
		}

        // BOT MOVEMENT!!!!!!!

        //chassis.arcade(leftY, rightX);
        double ctrX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 
        double ctrY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        double pwrF = ctrY;
        double pwrT = ctrX;

        if(1){
            double pwrF3 = pwrF*pwrF*pwrF;
            pwrF3 = pwrF3/10000;
        }

        if(1){
            double pwrT3 = pwrT*pwrT*pwrT;
            pwrT3 = pwrT3/10000;
            pwrT*=.925;
        }

        double pwrL = pwrF+pwrT;
        double pwrR = pwrF-pwrT;
        left_motor_group.move(pwrL);
        right_motor_group.move(pwrR);

        // delay to save resources
        pros::delay(25);
    }
}