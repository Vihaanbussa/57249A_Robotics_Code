#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
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
#include <cstdio>
#include <string>


pros::MotorGroup left_motor_group({-2, -3, -4}); // left motors on ports 1 (reversed), 2 (forwards), and 3 (reversed)
pros::MotorGroup right_motor_group({16, 15, 14}); // right motors on ports 4 (forwards), 5 (reversed), and 6 (forwards)
pros::Motor ladybrown(5);
pros::MotorGroup intake({-20, 10});
pros::Motor frontstage(10);
pros::Motor hooks(-20);
pros::adi::DigitalOut clamp('E');
pros::adi::DigitalOut doink('C');
pros::adi::DigitalOut pistonLift('B');
pros::Imu imu(17);
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversedx
pros::Rotation horizontalEnc(13);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-18);
pros::Rotation wallrotational(19);
// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.5);
// vertical tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, 1.8);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEnc, lemlib::Omniwheel::NEW_275, -1.5);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical color(12);
pros::adi::AnalogIn potentiometer ('A');



bool clampbool = LOW;
bool pistonliftbool = LOW;
//bool doink = LOW;
bool R2_pressed = false;
bool A_pressed = false;
bool alliancecolor = false; //true = red alliance





// drivetrain setting
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.3125, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              480, // drivetrain rpm is 360
                              2 // horizontal drift is 0. If we had traction wheels, it would have been 8
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(5.55, // proportional gain (kP) 6.9
                                              0.05, // integral gain (kI) 0.02
                                              20, // derivative gain (kD) 8
                                              7, // anti windup
                                              1, // small error range, in inches
                                              1400, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              80 // maximum acceleration (slew
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP) 2.5
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              10, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              80 // maximum acceleration (slew)
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

const int numstates = 4;
int states[numstates] = {0, 15, 25, 130};
int currstate = 0;
int target = 0;

void nextState(){ //macro to score
    currstate += 1;
    if(currstate == numstates){
        currstate = 0;
    }
    target = states[currstate];
}
bool bool360 = true;
void state360(){
    if(bool360){
        target = 230;
        bool360 = false;
    }
    else{
        currstate = 2;
        nextState();
        bool360 = true;
    }
}

void liftControl(){
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
    double kp = 1.5;
    double error = target - (wallrotational.get_position()/100.0);
    if ((wallrotational.get_position()/100.0) > 340) {
       error = (360 + target) - (wallrotational.get_position()/100.0); 
    }
    double velocity = kp*error;
    // if(abs(error) > 10){
    //     intake.move(velocity);
    // }
    ladybrown.move(velocity);
    if(abs(error) > 10){
    intake.move(velocity);
    }
}

void easyLoad(){
    nextState();
    pros::delay(700);
    nextState();
    nextState();
    intake.move(-127);
    pros::delay(1000);
    nextState();
    pros::delay(700);
    nextState();
}

bool intakespin = false;
bool runintakejam = true;
bool runfrontstage = false;
bool macropeck = false;
bool isextake = false;

void intakeunjam() {
    if (intakespin == true) {
        intake.move(-127);
        if (intake.get_actual_velocity() == 0) {
            intake.move(127);
            pros::delay(100);
            intake.move(-127);
        }
    }
    else if (intakespin == false) {
        intake.move(0);
    }
    else if (intakespin == true && runintakejam == false) {
        intake.move(-127);
    } 
    else if (intakespin == true && runfrontstage == true) {
        frontstage.move(-127);
    } 
    else if (intakespin == true && macropeck == true) {
        intake.move(-127);
        if (intake.get_actual_velocity() == 0) {
            intake.move(127);
            pros::delay(5);
            intake.move(-127);
        }
    } 
} //not used


void colorSortAuton(){
    if (pros::competition::is_autonomous()){
        intake.set_brake_mode(pros::MotorBrake::hold);
        //blue ring color sort
        if (intakespin == true && alliancecolor == true  && isextake == false && runfrontstage == false && color.get_hue() >= 180 && color.get_hue() <= 240) {
            pros::lcd::print(6, "blue ring detected");
            intake.move(0);
            pros::delay(200);
            intake.move(-127);
        }
        //red ring color sort
        else if (intakespin == true && alliancecolor == false  && isextake == false  && runfrontstage == false && (color.get_hue() <= 40 || color.get_hue() >= 350)) {
            pros::lcd::print(6, "red ring detected");
            intake.move(0);
            pros::delay(200);
            intake.move(-127);
        }
        //intake unjam
        else if (intakespin == true && runintakejam == true && runfrontstage == false && macropeck == false && isextake == false) {
            intake.move(-127);
            if (intake.get_actual_velocity() == 0) {
                intake.move(127);
                pros::delay(220);
                intake.move(-127);
            }
        }
        else if (intakespin == false  && isextake == false) {
            intake.move(0);
        }
        else if (intakespin == true && runintakejam == false  && isextake == false) {
            intake.move(-127);
        } 
        else if (intakespin == true && runfrontstage == true && isextake == false) {
            hooks.move(0);
            frontstage.move(-127);
        } 
        else if (intakespin == true && macropeck == true && isextake == false) {
            intake.move(-127);
            if (intake.get_actual_velocity() == 0) {
                intake.move(127);
                pros::delay(25);
                intake.move(-127);
            }
        }
        else if (intakespin == false && isextake == true) {
            intake.move(127);
        } 
    }
}


void colorSortforRobot(){
    intake.set_brake_mode(pros::MotorBrake::hold);
    //alliance color true = red and false = blue
    if (alliancecolor == true && color.get_hue() >= 180 && color.get_hue() <= 240) {
        pros::lcd::print(6, "blue ring detected");
        intake.move(0);
        pros::delay(200);
        intake.move(-127);
    }
    else if (alliancecolor == false && (color.get_hue() <= 40 || color.get_hue() >= 350)) {
        pros::lcd::print(6, "red ring detected");
        intake.move(0);
        pros::delay(200);
        intake.move(-127);
    }
    else if (1) {
        intake.move(-127);
    }

} 

//not used




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

int current_auton_selection = -1; // Initialize to an invalid value
const int numcases = 8;

const char* auton_names[numcases + 1] = {
    "redneg",
    "skills",
    "blueneg",
    "redpos",
    "bluepos",
    "SAWPred",
    "47skills",
    "SAWPblue",
    "Testing"
};

// Function to update the selected autonomous mode efficiently
void update_auton_selector() {
    int pot_value = potentiometer.get_value(); // Read potentiometer value (10-4095)

    // Ensure potentiometer value is within valid range
    if (pot_value < 10) pot_value = 10;
    if (pot_value > 4095) pot_value = 4095;

    // Correct section size calculation
    int section_size = (4095 - 10 + numcases) / (numcases + 1);

    // Determine which autonomous mode to select
    current_auton_selection = (pot_value - 10) / section_size;

    // Ensure value stays within valid range
    if (current_auton_selection > numcases) current_auton_selection = numcases;  // Fixing boundary case

    // Print selected autonomous mode name (clearing only necessary lines)
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "Auton: %s", auton_names[current_auton_selection]);

    // Overwrite previous text properly on the controller without flickering
    controller.print(0, 0, "Auton: %-10s", auton_names[current_auton_selection]); // Ensures old text is overwritten
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
    color.set_led_pwm(40);
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
    pros::Task liftControlTask([]{
        while(true) {
            // colorSort();
            liftControl();
            update_auton_selector();
            pros::delay(10);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    clamp.set_value(true);
}

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
 * If the robot is disabled or communications is los
 |t, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */ 

void autonomous() {
    clamp.set_value(true);
    pros::Task autotask([]{
        while(true) {
            colorSortAuton();
            // intakeunjam();
            pros::delay(5);
        }
    });
    switch(current_auton_selection){
        case 0: //red negative
            alliancecolor = true;
            chassis.setPose(0, 0, 320);
            chassis.moveToPoint(-8.5, 10, 2000,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            pros::delay(750);
            nextState();
            chassis.turnToHeading(315, 500);
            chassis.moveToPoint(7, -10, 750, {.forwards = false, .maxSpeed = 75});
            chassis.moveToPoint(7, -25, 500, {.forwards = false, .maxSpeed = 75});
            chassis.waitUntilDone();
            pros::delay(250);
            clamp.set_value(false);
            chassis.turnToHeading(130, 1000);
            chassis.waitUntilDone();
            intakespin = true;
            chassis.moveToPoint(18, -32, 1500);

            // chassis.moveToPoint(12, -22, 1000,{.forwards = false});
            chassis.turnToHeading(109, 750);
            chassis.moveToPoint(27, -35, 2000);

            chassis.moveToPoint(7, -17, 1250, {.forwards = false});
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(26, -19, 1500);
            chassis.turnToHeading(270, 750);
            chassis.moveToPoint(0, -19, 800);
            chassis.waitUntilDone();
            nextState();
            nextState();
            break;
        case 1: //Skills
            alliancecolor = true;
            chassis.setPose(0, 0, 0);
            clamp.set_value(true);
            runintakejam = false;
            intakespin = true;
            pros::delay(500);
            intakespin = false;
            chassis.moveToPoint(0, 13, 750);
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(-25, 16, 850, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            //clamp first mogo
            clamp.set_value(false);
            pros::delay(200);
            runintakejam = true;
            intakespin = true;
            chassis.turnToHeading(0, 500);  
            chassis.moveToPoint(-25, 42, 800); //ring 1
            chassis.turnToHeading(300, 500);
            chassis.moveToPoint(-38, 55, 800);
            chassis.turnToHeading(355, 500);
            chassis.moveToPoint(-50, 94, 1000); //ring 2
            chassis.waitUntilDone(); 
            pros::delay(75);
            chassis.moveToPoint(-40, 65, 1000, {.forwards = false}); //ring 3 68.5
            chassis.waitUntilDone();
            nextState();
            chassis.turnToHeading(271.5, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-59, 67.4, 1000, {.maxSpeed = 87});
            chassis.waitUntilDone();
            macropeck = true;
            pros::delay(700); //removable
            intakespin = false;
            isextake = true;
            pros::delay(50);
            isextake = false;
            intake.set_brake_mode(pros::MotorBrake::coast);
            chassis.turnToHeading(275, 500);
            chassis.waitUntilDone();
            nextState();
            pros::delay(950);
            chassis.turnToHeading(295, 300);
            chassis.turnToHeading(255, 300);
            chassis.turnToHeading(295, 300);
            chassis.turnToHeading(275, 300);
            chassis.moveToPoint(-64, 64, 600);
        

            chassis.waitUntilDone();


            macropeck = false;
            intake.set_brake_mode(pros::MotorBrake::hold);
            intakespin = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(-49, 66, 800, {.forwards = false});
            chassis.waitUntilDone();
            nextState();
            chassis.turnToHeading(181, 600);
            chassis.moveToPoint(-53, 12, 1500, {.maxSpeed = 70});
            chassis.moveToPoint(-51, 16, 600, {.forwards = false});
            chassis.turnToHeading(270, 800);
            chassis.moveToPoint(-61, 20, 1000, {.maxSpeed = 70});
            chassis.moveToPoint(-50, 20, 600, {.forwards = false});
            chassis.turnToHeading(315, 800);
            chassis.moveToPoint(-65, 5, 800, {.forwards = false});
            chassis.waitUntilDone();
            intakespin = false;
            clamp.set_value(true);
            //quadrant 1 end





            pros::delay(100);
            intakespin = true;
            chassis.moveToPoint(-50, 15, 900);
            chassis.turnToHeading(270,700);
            chassis.moveToPoint(0, 17, 1500,{.forwards = false, .minSpeed = 20});
            chassis.turnToHeading(270, 500);
            chassis.moveToPoint(23, 17, 1000,{.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(10);





            //quadrant 2 start
            clamp.set_value(false);
            pros::delay(200);
            intakespin = true;
            //time short start
            chassis.turnToHeading(359,500);
            chassis.moveToPoint(16,38,700);
            chassis.turnToHeading(70, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(26,49,700);
            chassis.turnToHeading(8, 600);
            chassis.moveToPoint(46, 96, 1000);
            chassis.moveToPoint(36, 64.5, 1100, {.forwards = false});
            chassis.waitUntilDone();
            nextState();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(52, 67, 800, {.maxSpeed = 87});
            chassis.waitUntilDone();
            macropeck = true;
            pros::delay(800);
            intakespin = false;
            isextake = true;
            pros::delay(40);
            isextake = false;
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            nextState();
            pros::delay(700);
            chassis.turnToHeading(111, 400);
            chassis.turnToHeading(71,300);
            chassis.turnToHeading(111, 300);
            chassis.turnToHeading(90, 300);
            chassis.moveToPoint(55, 68, 600);
            chassis.moveToPoint(40, 66.5, 700, {.forwards = false});
            chassis.waitUntilDone();
            nextState();
            intakespin = true;
            macropeck = false;
            chassis.turnToHeading(180, 500);
            chassis.moveToPoint(41, 4, 1500, {.maxSpeed = 70});
            chassis.moveToPoint(41, 19, 800, {.forwards = false});
            chassis.turnToHeading(90, 500);
            chassis.moveToPoint(56, 25, 800);
            chassis.moveToPoint(41, 21, 800, {.forwards = false});
            chassis.turnToHeading(340, 700);
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.moveToPoint(63, 3, 900, {.forwards = false});
            chassis.waitUntilDone();
            intakespin = false;
            chassis.turnToHeading(315, 500);
            chassis.waitUntilDone();

            //center field
            intakespin = true;
            runfrontstage = true;
            chassis.moveToPoint(0, 65, 1200, {.minSpeed = 40});
            chassis.waitUntilDone();
            runfrontstage = false;
            intakespin = false;
            chassis.moveToPoint(-20, 84, 900);
            chassis.turnToHeading(229, 500);
            intakespin = true;
            runfrontstage = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(3, 115, 1250,{.forwards = false, .maxSpeed = 65});
            chassis.waitUntilDone();
            //last mogo for scoring
            clamp.set_value(false);
            pros::delay(200);
            runfrontstage = false;
            pros::delay(50);
            intakespin = true;
            chassis.moveToPoint(-57, 96, 1500);
            chassis.turnToHeading(0,1000);
            chassis.moveToPoint(-63, 110, 1000);
            chassis.turnToHeading(60, 500);
            chassis.moveToPoint(-55, 120, 1000);
            chassis.turnToHeading(160, 750);
            chassis.moveToPoint(-50, 105, 1000);
            chassis.turnToHeading(160, 500);
            // chassis.turnToHeading(325, 700);
            // chassis.turnToHeading(175, 500);
            // chassis.moveToPoint(-55, 100, 1000);
            chassis.moveToPoint(-77, 142, 1100,{.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.moveToPoint(-40, 110, 1000);
            chassis.turnToHeading(90,500);
            chassis.moveToPoint(20, 137, 750,{.minSpeed = 90});
            chassis.moveToPoint(80, 135, 1000);
            //release last mogo

            break;
        case 2: // blue negative
            alliancecolor = false;
            chassis.setPose(-0, 0, -320);
            chassis.moveToPoint(8.5, 10, 2000,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            pros::delay(750);
            nextState();
            chassis.turnToHeading(-315, 500);
            chassis.moveToPoint(-7, -10, 750, {.forwards = false, .maxSpeed = 75});
            chassis.moveToPoint(-7, -25, 500, {.forwards = false, .maxSpeed = 75});
            chassis.waitUntilDone();
            pros::delay(250);
            clamp.set_value(false);
            chassis.turnToHeading(-130, 1000);
            chassis.waitUntilDone();
            intakespin = true;
            chassis.moveToPoint(-20.5, -32, 1500);

           
            chassis.turnToHeading(-111, 750);
            chassis.moveToPoint(-29, -36, 2000); //(coen delay)

            chassis.moveToPoint(-7, -17, 1250, {.forwards = false});
            chassis.turnToHeading(-90, 750);
            chassis.moveToPoint(-26, -19, 1500);
            chassis.turnToHeading(-270, 750);
            chassis.moveToPoint(-0, -19, 800);
            chassis.waitUntilDone();
            nextState();
            nextState();
            break;           
        case 3: //red positive
            alliancecolor = true;
            chassis.setPose(0, 0, 0);
            intakespin = true;
            runfrontstage = true;
            doink.set_value(true);
            chassis.moveToPoint(0, 34, 900, {.minSpeed = 50});
            chassis.turnToHeading(290, 700);
            chassis.turnToHeading(-300, 700);
            chassis.moveToPoint(-26, 26, 1500, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            clamp.set_value(false);
            runfrontstage = false;
            pros::delay(10);
            intakespin = true;
            doink.set_value(false);
            intakespin = true;
            pros::delay(100);
            intakespin = true;
            chassis.moveToPoint(0, -8, 1400, {.maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = true;
            chassis.turnToHeading(83, 750, {.maxSpeed = 80});
            chassis.waitUntilDone();
            doink.set_value(true);
            chassis.moveToPoint(24, -2, 1000, {.maxSpeed = 80});
            chassis.turnToHeading(350, 900, {.maxSpeed = 110});
            chassis.waitUntilDone();
            doink.set_value(false);
            chassis.turnToHeading(84, 750, {.maxSpeed = 80});
            chassis.moveToPoint(38, 4, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            pros::delay(1100);
            chassis.moveToPoint(0, 32, 1000, {.forwards = false});
            chassis.turnToHeading(0, 1000);
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.moveToPoint(-3, 38, 900);
            chassis.turnToHeading(160, 750);
            break;
        case 4: //blue positive 
            alliancecolor = false; 
            chassis.setPose(0, 0, 0);
            intakespin = true;
            runfrontstage = true;
            doink.set_value(true);
            chassis.moveToPoint(0, 28.5, 900, {.minSpeed = 80});
            chassis.turnToHeading(300, 700);
            chassis.moveToPoint(17, 26, 1200, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            doink.set_value(true);
            chassis.waitUntilDone();
            clamp.set_value(false);
            chassis.moveToPoint(-13, 30, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            runfrontstage = false;
            chassis.turnToHeading(190, 1200, {.maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = true;
            doink.set_value(true);
            chassis.moveToPoint(-30, 12, 2500, {.maxSpeed = 70});
            chassis.waitUntilDone();
            intakespin = false;
            chassis.waitUntilDone();
            chassis.turnToHeading(110, 800);
            chassis.waitUntilDone();
            doink.set_value(false);
            intakespin = true;
            chassis.moveToPoint(-30, 20, 1200, {.forwards = false}); //adjust
            chassis.turnToHeading(200, 800);
            chassis.moveToPoint(-31.5, 5, 1000, {.maxSpeed = 70});
            pros::delay(250);
            chassis.moveToPoint(-15, 15, 1000, {.maxSpeed = 70});
            pros::delay(250);
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 800);
            chassis.waitUntilDone();
            intakespin = false;
            clamp.set_value(true);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-13, 45, 2000, {.forwards = false, .maxSpeed = 80});
            break;
        case 5: // SAWP red
            alliancecolor = true;
            chassis.setPose(0, 0, 320);
            chassis.moveToPoint(-8.5, 10, 2000,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            pros::delay(750);
            nextState();
            chassis.turnToHeading(315, 500);
            chassis.moveToPoint(7, -10, 750, {.forwards = false, .maxSpeed = 75});
            chassis.moveToPoint(7, -25, 500, {.forwards = false, .maxSpeed = 75});
            chassis.waitUntilDone();
            pros::delay(250);
            clamp.set_value(false);
            // chassis.turnToHeading(130, 1000);
            // chassis.waitUntilDone();
            // intakespin = true;
            // chassis.moveToPoint(20, -33, 1500);
            // chassis.turnToHeading(150, 500);
            // chassis.moveToPoint(7, -17, 1250, {.forwards = false});
            // chassis.turnToHeading(90, 750);
            intakespin = true;
            chassis.turnToHeading(90, 500);
            chassis.moveToPoint(30, -19, 1500);
            chassis.turnToHeading(330, 750);
            chassis.moveToPoint(-7, 0, 800);
            chassis.turnToHeading(270, 600);
            chassis.waitUntilDone();
            clamp.set_value(true);
            intakespin = true;
            runfrontstage = false;
            chassis.moveToPoint(-45, 2, 1500, {.maxSpeed = 70});
            chassis.waitUntilDone();
            runfrontstage = true;
            chassis.turnToHeading(0, 750);
            runfrontstage = true;
            chassis.moveToPoint(-37, -30, 750,{.forwards = false});
            runfrontstage = true;
            chassis.waitUntilDone();
            runfrontstage = true;
            nextState();
            nextState();
            pros::delay(50);
            clamp.set_value(false);
            pros::delay(200);
            intakespin = true;
            runfrontstage = false;
        
    
            chassis.turnToHeading(270, 500);
            chassis.moveToPoint(-65, -15, 750); //-60
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(-30, -20, 1000);
            chassis.waitUntilDone();



            break; 
        case 6: //47skills
            alliancecolor = true;
            chassis.setPose(0, 0, 0);
            intakespin = true;
            pros::delay(750);
            intakespin = false;
            chassis.moveToPoint(0, 13, 750);
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(-23, 17, 1250, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            //clamp first mogo
            clamp.set_value(false);
            pros::delay(250);
            intakespin = true;
            chassis.moveToPoint(-20,17,750);
            chassis.turnToHeading(0 , 500);  
            chassis.moveToPoint(-20, 45, 500);
            chassis.turnToHeading(300, 1000);
            chassis.moveToPoint(-55, 65, 1000);
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-48, 65, 1000, {.forwards = false});
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-48, 12, 5000, {.maxSpeed = 80});
            chassis.moveToPoint(-48, 20, 1000, {.forwards = false});
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-59, 24, 1000);
            chassis.moveToPoint(-50, 20, 1000, {.forwards = false});
            chassis.turnToHeading(315, 500);
            chassis.moveToPoint(-65, 5, 1000, {.forwards = false});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(250);
            clamp.set_value(true);


            //quadrant 1 end
            pros::delay(250);
            intakespin = true;
            chassis.moveToPoint(-50, 15, 1000);
            chassis.turnToHeading(270,1000);
            chassis.moveToPoint(0, 18, 3000,{.forwards = false});
            chassis.moveToPoint(20, 18, 1500,{.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(250);
            //quadrant 2 start
            clamp.set_value(false);
            pros::delay(250);
            intakespin = true;
            //time short start
            chassis.turnToHeading(0,750);
            chassis.moveToPoint(20,40,1500);
            chassis.turnToHeading(60, 750);
            chassis.moveToPoint(45, 60, 1000);
            chassis.turnToHeading(90, 750);
            //time short
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(47, 10, 5000, {.maxSpeed = 80});
            chassis.moveToPoint(43, 20, 1000, {.forwards = false});
            chassis.turnToHeading(90,1000);
            chassis.moveToPoint(55, 25, 1000);
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(60, 5, 1000, {.forwards = false});
            chassis.turnToHeading(305, 750);
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(250);

            //quadrant 2 end
            clamp.set_value(true);
            intakespin = true;
            runfrontstage = true;
            pros::delay(250);
            //center field
            chassis.moveToPoint(0, 65, 2000);
            chassis.moveToPoint(-20, 80, 1500);
            chassis.waitUntilDone();
            runfrontstage = false;
            intakespin = false;
            chassis.moveToPoint(-25, 85, 1000);
            chassis.moveToPoint(-20, 80, 1500,{.forwards = false});
            chassis.turnToHeading(225, 1000);
            chassis.moveToPoint(0, 120, 2000, {.forwards = false, .maxSpeed = 65});
            chassis.waitUntilDone();
            //last mogo for scoring
            clamp.set_value(false);
            pros::delay(500);
            intakespin = true;
            chassis.moveToPoint(-60, 90, 1500);
            chassis.turnToHeading(0,1000);
            chassis.moveToPoint(-65, 110, 1000);
            chassis.turnToHeading(45, 500);
            chassis.moveToPoint(-55, 120, 1000);
            chassis.turnToHeading(190, 750);
            chassis.moveToPoint(-55,110, 1000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.moveToPoint(-75, 130, 1000,{.forwards = false});
            clamp.set_value(true);
            //release last mogo

            //Quadrant 4
            intakespin = false;
            chassis.turnToHeading(135, 750);
            chassis.moveToPoint(-40, 115, 1000, {.minSpeed = 70});
            chassis.moveToPoint(-20, 115, 800, {.minSpeed = 40});
            chassis.turnToHeading(60, 750, {.minSpeed = 40});
            chassis.moveToPoint(25, 134, 800, {.minSpeed = 40});
            chassis.moveToPoint(55, 134,800, {.minSpeed = 20});
            chassis.moveToPoint(-20,100, 800, {.forwards = false});
            break;
                    
        case 7: // SAWP blue
            alliancecolor = false;
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0,3.75,500, {.minSpeed = 100});
            chassis.turnToHeading(-35, 500);
            nextState();
            nextState();
            pros::delay(450);
            chassis.moveToPoint(37, -2, 1250, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            pros::delay(200);
            clamp.set_value(false);
            nextState();
            chassis.turnToHeading(-240, 600);
            intakespin = true;
            chassis.moveToPoint(47, -17, 900);
            chassis.waitUntilDone();            
            chassis.moveToPoint(50, -25, 500,{.maxSpeed = 80,.minSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(55, -27, 500,{.maxSpeed = 80,.minSpeed = 100});
            chassis.turnToHeading(-180, 300);
            chassis.moveToPoint(42, -7, 1000, { .forwards = false, .minSpeed = 100});
            chassis.moveToPoint(39, -26, 1000);
            chassis.turnToHeading(-30, 500);
            chassis.moveToPoint(10, -5, 750);
            chassis.turnToHeading(-0, 500);
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.moveToPoint(15, 40, 1350, {.maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = false;
            chassis.turnToHeading(-90, 500);
            chassis.moveToPoint(35, 40, 1000, {.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(false);
            chassis.turnToHeading(-0, 750);
            chassis.waitUntilDone();
            intakespin = true;
            chassis.moveToPoint(40, 65, 750,{.minSpeed = 110});
            chassis.turnToHeading(-180, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(45,40, 750);
            chassis.waitUntilDone();
            nextState();
            nextState();
            break;
        case 8: //testing
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(90, 1200);
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
    // state360();
    // clamp.set_value(true);

    while (true) {
        
        
         // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(7, "lbpos:%ld", wallrotational.get_position());
        
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        

        //move ladybrown mech
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            nextState();
        }
        /*if(color.get_hue()>200){
            pros::Task task{[]{
                color_sort();
            }}
        }*/
        



        //move intake (NOT TOGGLE | MUST HOLD)
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(127); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move(-127);
        } else{
            intake.move(0);
            pros::lcd::print(6, "");
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

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
		    chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 50, 1500);
		}

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            chassis.moveToPoint(0, 0, 1500, {.forwards = false});
		}

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
		    chassis.setPose(0, 0, 0);
            chassis.turnToHeading(90, 1500);
		}
        
        //piston lift controls
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
		    if(A_pressed){
                doink.set_value(false);
                A_pressed = false;
            } else{
                doink.set_value(true);
                A_pressed = true;
            }
		}

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            state360();
        }
        
         if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            pros::Task autoLoad{[] {
                easyLoad();
            }};
        }

        // BOT MOVEMENT!!!!!!!

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
            pwrT*=.925; //makes turning less sensitive
        }

        double pwrL = pwrF+pwrT;
        double pwrR = pwrF-pwrT;
        left_motor_group.move(pwrL);
        right_motor_group.move(pwrR);

        // delay to save resources
        pros::delay(25);
    }

    
            
            
}
