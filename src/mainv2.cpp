#include "main.h"
#define LEFT_FRONT_PORT 20
#define LEFT_REAR_PORT 10
#define RIGHT_FRONT_PORT 11
#define RIGHT_REAR_PORT 1
#define OPTICAL_PORT 19
#define OPTICAL_PORT_UPDATE 50
#define PI 3.14
#define wheel_diameter 4 //4"/11cm diameter wheels (omni)

int distance_per_degree = PI*(wheel_diameter/360);
bool tankdrive = false;

const float kP = 0.9;  // Proportional gain
const float kI = 0.01; // Integral gain
const float kD = 0.05; // Derivative gain

const int targetPosition = 1000;

pros::Optical optical_sensor(OPTICAL_PORT, OPTICAL_PORT_UPDATE);

void initializeMotors(pros::Motor& motor) {
	//pros::Controller master(pros::E_CONTROLLER_MASTER);
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // Set motor brake mode
    motor.tare_position(); // Reset motor encoder position
}

void moveBaseWithPID(pros::Motor& leftFront, pros::Motor& leftRear, pros::Motor& rightFront, pros::Motor& rightRear, int target) {
    initializeMotors(leftFront);
    initializeMotors(leftRear);
    initializeMotors(rightFront);
    initializeMotors(rightRear);

    // Variables for PID control
    double error;
    double integral = 0;
    double derivative;

    while (true) {
        // Calculate average error for all motors
        error = (target - leftFront.get_position() +
                 target - leftRear.get_position() +
                 target - rightFront.get_position() +
                 target - rightRear.get_position()) / 4.0;

        // Update integral and prevent integral windup
        integral += error;
        if (integral > 100) {
            integral = 100;
        } else if (integral < -100) {
            integral = -100;
        }

        // Calculate derivative
        derivative = error - (leftFront.get_position() +
                             leftRear.get_position() +
                             rightFront.get_position() +
                             rightRear.get_position()) / 4.0;

        // Calculate motor power using PID formula
        double power = kP * error + kI * integral + kD * derivative;

        // Set motor powers
        leftFront.move_velocity(power);
        leftRear.move_velocity(power);
        rightFront.move_velocity(power);
        rightRear.move_velocity(power);

        // Break the loop if the average error is small enough
        if (abs(error) < 10) {
            break;
        }

        pros::delay(20); // Add a delay to control the update rate of the PID loop
    }

    // Stop all motors when the target position is reached
    leftFront.move_velocity(0);
    leftRear.move_velocity(0);
    rightFront.move_velocity(0);
    rightRear.move_velocity(0);
}

void moveForward(double distance) {
	pros::Motor(LEFT_FRONT_PORT).move_absolute(distance, 100);
    pros::Motor(LEFT_REAR_PORT).move_absolute(distance, 100);
    pros::Motor(RIGHT_FRONT_PORT).move_absolute(distance, 100);
    pros::Motor(RIGHT_REAR_PORT).move_absolute(distance, 100);
}

// Function to turn the robot (clockwise for simplicity)
void turnRight(int angle) {
    pros::Motor(LEFT_FRONT_PORT).move_relative(angle, 100);  // Adjust speed as needed
    pros::Motor(LEFT_REAR_PORT).move_relative(angle, 100);
    pros::Motor(RIGHT_FRONT_PORT).move_relative(-angle, 100); // Negative angle for clockwise turn
    pros::Motor(RIGHT_REAR_PORT).move_relative(-angle, 100);
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	optical_sensor.disable_gesture();
	pros::lcd::initialize();
	//pros::Controller master(CONTROLLER_MASTER);
	pros::lcd::set_text(1, "Wassup");
	master.print(0, 0, "B to move 100 units");
	pros::delay(60);
	master.print(1, 0, "A to toggle drivemode");
	//pros::lcd::register_btn1_cb(on_center_button);
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

void autonomous() {
	/*pros::Motor leftFront(LEFT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
	pros::Motor leftRear(LEFT_REAR_PORT, pros::E_MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
	pros::Motor rightFront(RIGHT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
	pros::Motor rightRear(RIGHT_REAR_PORT, pros::E_MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
	initializeMotors(leftFront);
    initializeMotors(leftRear);
    initializeMotors(rightFront);
    initializeMotors(rightRear);
	moveForward(11);*/
	pros::Motor_Group motor_group ({20, 10, 11, 1});
	motor_group.move_absolute(100, 100); // Moves 100 units forward
	while (!((motor_group.get_positions()[0] < 105) && (motor_group.get_positions()[0] > 95))) {
		// Continue running this loop as long as the motor is not within +-5 units of its goal
		pros::delay(5);
	}
	motor_group.move_absolute(100, 100); // This does not cause a movement
	while (!((motor_group.get_positions()[0] < 105) && (motor_group.get_positions()[0] > 95))) {
		pros::delay(5);
	}
	motor_group.tare_position();
	motor_group.move_absolute(100, 100); // Moves 100 units forward
	while (!((motor_group.get_positions()[0] < 105) && (motor_group.get_positions()[0] > 95))) {
		pros::delay(5);
	}
    //moveBaseWithPID(leftFront, leftRear, rightFront, rightRear, targetPosition);
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor leftFront(LEFT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
	pros::Motor leftRear(LEFT_REAR_PORT, pros::E_MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
	pros::Motor rightFront(RIGHT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
	pros::Motor rightRear(RIGHT_REAR_PORT, pros::E_MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
	pros::c::optical_rgb_s_t rgb_value;
    initializeMotors(leftFront);
    initializeMotors(leftRear);
    initializeMotors(rightFront);
    initializeMotors(rightRear);
	master.clear();
	bool toggle_led = false;
	int ledPWM = 20;
	int controller_print_page = 0;

	while (true) {
		//optical_sensor.set_led_pwm(80);
		rgb_value = optical_sensor.get_rgb();
		/*printf("Hue value: %lf \n", optical_sensor.get_hue());
    	pros::delay(20);
		printf("Saturation value: %lf \n", optical_sensor.get_saturation());
    	pros::delay(20);*/
		if(master.get_digital_new_press(DIGITAL_R1)) toggle_led = !toggle_led;
		if(master.get_digital_new_press(DIGITAL_R2)) ledPWM += 20;
		if(ledPWM > 100){
			ledPWM = 20;
			master.clear();
		}
		if(toggle_led) optical_sensor.set_led_pwm(ledPWM);
		else optical_sensor.set_led_pwm(0);
		if(master.get_digital_new_press(DIGITAL_A)){
			tankdrive = !tankdrive;
			master.clear_line(2);
		}
		if(master.get_digital_new_press(DIGITAL_B)){
			leftFront.tare_position();
    		leftRear.tare_position();
			rightFront.tare_position();
			rightRear.tare_position();
			moveForward(400);
			while(!(leftFront.get_position() > 400) && (leftFront.get_position() < 395)){
				pros::delay(5);
			}
			leftFront.tare_position();
    		leftRear.tare_position();
			rightFront.tare_position();
			rightRear.tare_position();
		}
		if(!tankdrive){
			//master.print(2, 0,  "Mode: Normal drive  ");
			int power = master.get_analog(ANALOG_LEFT_Y);
			int turn = master.get_analog(ANALOG_RIGHT_X);
			double left = (power + turn) * kP;
			double right = (power - turn) * kP;

			leftFront.move(left);
			leftRear.move(left);
			rightFront.move(right);
			rightRear.move(right);
		}
		else{
			//master.print(2, 0,  "Mode: Tank drive  ");
			double left = master.get_analog(ANALOG_LEFT_Y);
			double right = master.get_analog(ANALOG_RIGHT_Y);

			leftFront.move(left);
			leftRear.move(left);
			rightFront.move(right);
			rightRear.move(right);
		}
		if(master.get_digital_new_press(DIGITAL_Y)){
			master.clear();
			controller_print_page += 1;
			if(controller_print_page >= 2) controller_print_page = 0;
			master.rumble(". - . -");
		}
		if(controller_print_page == 0){
			pros::delay(50);
			master.print(0, 0, "Red value: %lf", rgb_value.red);
			pros::delay(50);
			master.print(1, 0, "Green value: %lf", rgb_value.green);
			pros::delay(50);
			master.print(2, 0, "Blue value: %lf", rgb_value.blue);
			pros::delay(50);
		}
		else if(controller_print_page == 1){
			pros::delay(50);
			master.print(0, 0, "%s  ", tankdrive ? "Mode: Tank Drive":"Mode: Norm Drive");
			pros::delay(50);
			master.print(1, 0, "LED Bright: %d%%", ledPWM);
			pros::delay(50);
			master.print(2, 0, "LED Toggle: %s  ", toggle_led ? "ON" : "OFF");
			pros::delay(50);
		}
		pros::lcd::print(0, "Sensor LED Brightness: %d%%", ledPWM);
		pros::lcd::print(1, "LED Toggle: %s", toggle_led ? "ON" : "OFF");
		//pros::lcd::print(2, "%f", leftFront.get_position());
		pros::delay(30);
	}
}

/*
Driving forward
Pi x Wheel diameter / 360 = distance per degree

4" wheel example:
3.14 * 4 / 360 = .0349 inches per degree

To travel 24 inches:
24 / .0349 = 687.679 degrees of rotation.

~~ ~~

Turning
Looking from the top of your robot you need to identify Track Width which is the distance between the center of the left and right wheels.

If the Track Width is 16" then 16 * 3.14 is 50.24 inches. This is how far the robot would need to travel to spin 360 degrees. 50.24 / 360 degrees is 0.139" per degree of robot rotation.

You will also want to divide that calculation in half so that one set of wheels moves forward and the alternate side moves backwards. This will keep the robot rotating on center.

So a 90 degree turn to the left would be 90 * 0.139 = 12.56.
12.56 / 2 = 6.28.
The right motor(s) would move forward 6.28 inches and the left would spin backward 6.28 inches.*/