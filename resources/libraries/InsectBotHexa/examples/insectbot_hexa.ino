#include <Servo.h>

// #define DEBUG 1

#ifdef  DEBUG
#define DEBUG_PRINT(str)      Serial.print(str)
#define DEBUG_PRINTLN(str)    Serial.println(str)
#else
#define DEBUG_PRINT(str)
#define DEBUG_PRINTLN(str)
#endif

Servo f, m, r;                          /* Front, middle and rear servos */
int fa, ma, ra;                         /* Front, middle and rear servos current angle */

int s_delay = 5;                        /* Delay between s_step step in milliseconds */
int s_center = 90;                      /* Servos default angle */
int s_step = 1;                         /* Angle step when moving servos */

int fpin = 9, mpin = 11, rpin = 10;     /* Front, middle and rear servos digital pins */
int ledpin = 13;                        /* Blue LED on beetle board */

int moving_legs_min = s_center - 20;    /* Front and rear servos minimum angle: right leg front */
int moving_legs_max = s_center + 20;    /* Front and rear servos maximum angle: left leg front */
int bending_legs_min = s_center - 20;   /* Middle servo minimum angle: bend right */
int bending_legs_max = s_center + 20;   /* Middle servo maximum angle: bend left */

int lastDistanceFromObstacle;           /* Approximative distance in centimeters (5 - 80) */

int brightnessThreshold = 50;
int lastBrightnessOnLeft;               /* Raw value for left light sensor (0 - 1023) */
int lastBrightnessOnRight;              /* Raw value for right light sensor (0 - 1023) */

int distanceSensor = A1;
int lightSensorLeft = A2;
int lightSensorRight = A3;

#define COMPUTEBRIGHTNESS_LEFT 0x1
#define COMPUTEBRIGHTNESS_RIGHT 0x2

static void computeBrightness(int side);
static void computeDistanceFromObstacle(void);

static void lazySetup(void)
{
    static bool setup_done = false;
    if (setup_done) return;

    setup_done = true;
#ifdef  DEBUG
    Serial.begin(115200);
#endif
    DEBUG_PRINTLN("setup");

    pinMode(ledpin, OUTPUT);

    fa = ma = ra = s_center;
    f.attach(fpin);
    f.write(fa);
    m.attach(mpin);
    m.write(ma);
    r.attach(rpin);
    r.write(ra);

    computeBrightness(COMPUTEBRIGHTNESS_LEFT || COMPUTEBRIGHTNESS_RIGHT);
    computeDistanceFromObstacle();
}

/*
 * servo_move()
 *
 * Modifies the angle from <current angle> to <target angle> by <s_step> degree
 */
static void servo_move(Servo *s, int *current_angle, int target_angle)
{
    int step;
    if (*current_angle == target_angle) return;
    step = *current_angle > target_angle ? -s_step : s_step;

    do {
        *current_angle += step;
        s->write(*current_angle);
        delay(s_delay);
    } while (abs(*current_angle - target_angle) > s_step);
}

/*
 * cost_to_expected_position()
 *
 * Computes the cost of the planned position from current position
 */
static int cost_to_planned_position(int expected_fa, int expected_ma, int expected_ra)
{
    if ((expected_fa != fa) || (expected_ra != ra)) {
        /* 4 operations to perform:
         * - move the bending leg to middle
         * - move the front leg to expected position
         * - move the rear leg to expected position
         * - move the bending leg to expected position
         */
        return abs(s_center - ma) + abs(expected_fa - fa) + abs(expected_ra - ra) +
          abs(expected_ma - s_center);
    } else {
        return abs(expected_ma - ma);
    }
}

#define f_move(target_angle)  servo_move(&f, &fa, target_angle)
#define m_move(target_angle)  servo_move(&m, &ma, target_angle)
#define r_move(target_angle)  servo_move(&r, &ra, target_angle)

#define bend_left_angle   bending_legs_max
#define bend_right_angle  bending_legs_min

#define bend_left()    m_move(bend_left_angle)
#define bend_right()   m_move(bend_right_angle)
#define bend_middle()  m_move(s_center)

#define moving_left_leg_tofront_angle   moving_legs_max
#define moving_right_leg_tofront_angle  moving_legs_min
/* Aliases to above for clarity */
#define moving_left_leg_torear_angle    moving_right_leg_tofront_angle
#define moving_right_leg_torear_angle   moving_left_leg_tofront_angle

#define move_left_front_leg_tofront()  f_move(moving_left_leg_tofront_angle)
#define move_left_front_leg_torear()   f_move(moving_left_leg_torear_angle)
#define move_right_front_leg_tofront() move_left_front_leg_torear()
#define move_right_front_leg_torear()  move_left_front_leg_tofront()

#define move_left_rear_leg_tofront()   r_move(moving_left_leg_tofront_angle)
#define move_left_rear_leg_torear()    r_move(moving_left_leg_torear_angle)
#define move_right_rear_leg_tofront()  move_left_rear_leg_torear()
#define move_right_rear_leg_torear()   move_left_rear_leg_tofront()

static void computeDistanceFromObstacle(void)
{
    int i, count = 5, value = 0;
    int vals[] = { 600, 450, 300, 250, 170, 130, 100 }; /* Value read from sensor */
    int cms[] = { 5, 10, 15, 20, 30, 40, 50 };          /* Map to centimeters */

    lazySetup();

    DEBUG_PRINT("computeDistanceFromObstacle: ");

    for (i = 0; i < count; i++) {
        value += analogRead(distanceSensor);
    }
    value /= count;
    DEBUG_PRINT(value);
    DEBUG_PRINT(" ~> ");

    lastDistanceFromObstacle = 80;

    for (i = 0; i < sizeof (vals) / sizeof (*vals); i++) {
        if (value >= vals[i]) {
            lastDistanceFromObstacle = cms[i];
            break;
        }
    }

    DEBUG_PRINT(lastDistanceFromObstacle);
    DEBUG_PRINTLN(" cm");
}

int getDistanceFromObstacle(void)
{
    lazySetup();

    DEBUG_PRINT("getDistanceFromObstacle: ");

    computeDistanceFromObstacle();

    DEBUG_PRINT(lastDistanceFromObstacle);
    DEBUG_PRINTLN(" cm");
    return lastDistanceFromObstacle;
}

bool isInDanger(void)
{
    lazySetup();

    if (getDistanceFromObstacle() < 30) {   // centimeters
        DEBUG_PRINTLN("isInDanger: DANGER");
        return true;
    } else {
        DEBUG_PRINTLN("isInDanger: OK");
        return false;
    }
}

static void computeBrightness(int side)
{
    int i, count = 10, valueL = 0, valueR = 0;

    lazySetup();

    DEBUG_PRINT("computeBrightness: ");

    for (i = 0; i < count; i++) {
        if (side & COMPUTEBRIGHTNESS_LEFT) valueL += analogRead(lightSensorLeft);
        if (side & COMPUTEBRIGHTNESS_RIGHT) valueR += analogRead(lightSensorRight);
    }
    if (side & COMPUTEBRIGHTNESS_LEFT) {
        valueL /= count;
        DEBUG_PRINT("left: "); DEBUG_PRINT(valueL);
        lastBrightnessOnLeft = valueL;
    }
    if (side & COMPUTEBRIGHTNESS_RIGHT) {
        valueR /= count;
        DEBUG_PRINT("right: "); DEBUG_PRINTLN(valueR);
        lastBrightnessOnRight = valueR;
    }
}

bool isBrighterOnLeft(void)
{
    lazySetup();

    if (abs(lastBrightnessOnRight - lastBrightnessOnLeft) < brightnessThreshold) {
        return false;
    } else if (lastBrightnessOnLeft > lastBrightnessOnRight) {
        return true;
    } else {
        return false;
    }
}

bool isBrighterOnRight(void)
{
    lazySetup();

    if (abs(lastBrightnessOnRight - lastBrightnessOnLeft) < brightnessThreshold) {
        return false;
    } else if (lastBrightnessOnRight > lastBrightnessOnLeft) {
        return true;
    } else {
        return false;
    }
}

bool isBrightnessEqual(void)
{
    lazySetup();

    if (abs(lastBrightnessOnRight - lastBrightnessOnLeft) < brightnessThreshold) {
        return true;
    } else {
        return false;
    }
}

int getBrightnessOnLeft(void)
{
    lazySetup();

    computeBrightness(COMPUTEBRIGHTNESS_LEFT);

    return lastBrightnessOnLeft;
}

int getBrightnessOnRight(void)
{
    lazySetup();

    computeBrightness(COMPUTEBRIGHTNESS_RIGHT);

    return lastBrightnessOnRight;
}

void walkMode(void)
{
    s_delay = 5;
    s_step = 1;
}

void runMode(void)
{
    s_delay = 2;
    s_step = 1;
}

void goForward(void)
{
    lazySetup();

    DEBUG_PRINTLN("goForward: ");

    /*
     * To go forward:
     * - move the left front and rear legs from front to rear (move #A)
     * - move the right front and rear legs from front to rear (move #B)
     *
     * Order of the operations is chosen based on current position.
     *
     * Legend:
     * O  - foot touching ground
     * *  - foot not touching ground
     * XX - robot core
     *
     * Move #A:         *
     *  O            O-/
     *   \-*          XX
     *   XX     to    /-*
     *  O-\          O
     *     *
     *
     * Move #B:      *
     *     O          \-O
     *  *-/           XX
     *   XX     to   *-\
     *   /-O            O
     *  *
     */
    int moveA_dist = cost_to_planned_position(
        moving_left_leg_tofront_angle, bend_left_angle,
        moving_left_leg_tofront_angle);
    int moveB_dist = cost_to_planned_position(
        moving_right_leg_tofront_angle, bend_right_angle,
        moving_right_leg_tofront_angle);

    if (moveA_dist <= moveB_dist) {

        DEBUG_PRINT("moveA; ");

        if (moveA_dist != 0) {
            /* Prepare for move #A */
            DEBUG_PRINT("preparing legs; ");
            bend_middle();
            move_left_front_leg_tofront();
            move_left_rear_leg_tofront();
            bend_left();
        }

        DEBUG_PRINT("moving legs; ");

        /* Move #A */
        move_left_front_leg_torear();
        move_left_rear_leg_torear();

        DEBUG_PRINT("moveB; ");

        /* Right front and rear legs are already at front, prepare for move #B */

        DEBUG_PRINT("preparing legs; ");

        bend_right();

        DEBUG_PRINT("moving legs; ");

        /* Move #B */
        move_right_front_leg_torear();
        move_right_rear_leg_torear();

    } else {

        if (moveB_dist != 0) {
            /* Prepare for move #B */
            bend_middle();
            move_right_front_leg_tofront();
            move_right_rear_leg_tofront();
            bend_right();
        }

        /* Move #B */
        move_right_front_leg_torear();
        move_right_rear_leg_torear();

        /* Left front and rear legs are already at front, prepare for move #B */
        bend_left();

        /* Move #A */
        move_left_front_leg_torear();
        move_left_rear_leg_torear();

    }
}

void goBackward()
{
    lazySetup();

    DEBUG_PRINT("goBackward: ");

    /*
     * To go backward:
     * - move the left front and rear legs from rear to front (move #A)
     * - move the right front and rear legs from rear to front (move #B)
     *
     * Order of the operations is chosen based on current position.
     *
     * Legend:
     * O  - foot touching ground
     * *  - foot not touching ground
     * XX - robot core
     *
     * Move #A:
     *     *
     *  O-/           O
     *   XX     to     \-*
     *   /-*           XX
     *  O             O-\
     *                   *
     * Move #B:
     *  *
     *   \-O             O
     *   XX     to    *-/
     *  *-\            XX
     *     O           /-O
     *                *
     */
    int moveA_dist = cost_to_planned_position(
        moving_left_leg_torear_angle, bend_left_angle,
        moving_left_leg_torear_angle);
    int moveB_dist = cost_to_planned_position(
        moving_right_leg_torear_angle, bend_right_angle,
        moving_right_leg_torear_angle);

    if (moveA_dist <= moveB_dist) {

        if (moveA_dist != 0) {
            /* Prepare for move #A */
            bend_middle();
            move_left_front_leg_torear();
            move_left_rear_leg_torear();
            bend_left();
        }

        /* Move #A */
        move_left_front_leg_tofront();
        move_left_rear_leg_tofront();

        /* Right front and rear legs are already at rear, prepare for move #B */
        bend_right();

        /* Move #B */
        move_right_front_leg_tofront();
        move_right_rear_leg_tofront();

    } else {

        if (moveB_dist != 0) {
            /* Prepare for move #B */
            bend_middle();
            move_right_front_leg_torear();
            move_right_rear_leg_torear();
            bend_right();
        }

        /* Move #B */
        move_right_front_leg_tofront();
        move_right_rear_leg_tofront();

        /* Left front and rear legs are already at rear, prepare for move #B */
        bend_left();

        /* Move #A */
        move_left_front_leg_tofront();
        move_left_rear_leg_tofront();

    }
}

void turnLeft()
{
    lazySetup();

    /*
     * To turn left on itself:
     * - move right front and rear legs from front to rear (move #A)
     * - move left front and rear legs from rear to front (move #B)
     *
     * Order of the operations is chosen based on current position.
     *
     * Legend:
     * O  - foot touching ground
     * *  - foot not touching ground
     * XX - robot core
     *
     * Move #A:      *
     *     O          \-O
     *  *-/           XX
     *   XX     to   *-\    (moves forwards a bit)
     *   /-O            O
     *  *
     *
     * Move #B:
     *     *
     *  O-/          O
     *   XX     to    \-*   (moves backwards a bit)
     *   /-*          XX
     *  O            O-\
     *                  *
     */

    int moveA_dist = cost_to_planned_position(
        moving_right_leg_tofront_angle, bend_right_angle,
        moving_right_leg_tofront_angle);
    int moveB_dist = cost_to_planned_position(
        moving_left_leg_torear_angle, bend_left_angle,
        moving_left_leg_torear_angle);

    if (moveA_dist <= moveB_dist) {

        if (moveA_dist != 0) {
            /* Prepare for move #A */
            bend_middle();
            move_right_front_leg_tofront();
            move_right_rear_leg_tofront();
            bend_right();
        }

        /* Move #A */
        move_right_front_leg_torear();
        move_right_rear_leg_torear();

        /* Left front and rear legs are now at front, prepare for move #B */
        bend_middle();
        move_left_front_leg_torear();
        move_left_rear_leg_torear();
        bend_left();

        /* Move #B */
        move_left_front_leg_tofront();
        move_left_rear_leg_tofront();

    } else {

        if (moveB_dist != 0) {
            /* Prepare for move #B */
            bend_middle();
            move_left_front_leg_torear();
            move_left_rear_leg_torear();
            bend_left();
        }

        /* Move #B */
        move_left_front_leg_tofront();
        move_left_rear_leg_tofront();

        /* Right front and rear legs are now at rear, prepare for move #A */
        bend_middle();
        move_right_front_leg_tofront();
        move_right_rear_leg_tofront();
        bend_right();

        /* Move #A */
        move_right_front_leg_torear();
        move_right_rear_leg_torear();

    }
}

void turnRight()
{
    lazySetup();

    /*
     * To turn right on itself:
     * - move left front and rear legs from front to rear (move #A)
     * - move right front and rear legs from rear to front (move #B)
     *
     * Order of the operations is chosen based on current position.
     *
     * Legend:
     * O  - foot touching ground
     * *  - foot not touching ground
     * XX - robot core
     *
     * Move #A:         *
     *  O            O-/
     *   \-*          XX
     *   XX     to    /-*   (moves forwards a bit)
     *  O-\          O
     *     *
     *
     * Move #B:
     *  *
     *   \-O            O
     *   XX     to   *-/    (moves backwards a bit)
     *  *-\           XX
     *     O          /-O
     *               *
     */

    int moveA_dist = cost_to_planned_position(
        moving_left_leg_tofront_angle, bend_left_angle,
        moving_left_leg_tofront_angle);
    int moveB_dist = cost_to_planned_position(
        moving_right_leg_torear_angle, bend_right_angle,
        moving_right_leg_torear_angle);

    if (moveA_dist <= moveB_dist) {

        if (moveA_dist != 0) {
            /* Prepare for move #A */
            bend_middle();
            move_left_front_leg_tofront();
            move_left_rear_leg_tofront();
            bend_left();
        }

        /* Move #A */
        move_left_front_leg_torear();
        move_left_rear_leg_torear();

        /* Right front and rear legs are now at front, prepare for move #B */
        bend_middle();
        move_right_front_leg_torear();
        move_right_rear_leg_torear();
        bend_right();

        /* Move #B */
        move_right_front_leg_tofront();
        move_right_rear_leg_tofront();

    } else {

        if (moveB_dist != 0) {
            /* Prepare for move #B */
            bend_middle();
            move_right_front_leg_torear();
            move_right_rear_leg_torear();
            bend_right();
        }

        /* Move #B */
        move_right_front_leg_tofront();
        move_right_rear_leg_tofront();

        /* Left front and rear legs are now at rear, prepare for move #A */
        bend_middle();
        move_left_front_leg_tofront();
        move_left_rear_leg_tofront();
        bend_left();

        /* Move #A */
        move_left_front_leg_torear();
        move_left_rear_leg_torear();

    }
}

void setup(void)
{
    bool ledon = false;
    int i;

    lazySetup();

    while (getDistanceFromObstacle() >= 10) {
        digitalWrite(ledpin, ledon ? LOW : HIGH);
        ledon = !ledon;
        delay(1000);
    }

    for (i = 0; i < 20; i++) {
        digitalWrite(ledpin, ledon ? LOW : HIGH);
        ledon = !ledon;
        delay(100);
    }
    digitalWrite(ledpin, LOW);
}

void loop(void)
{
    static int dist;
    int i;
    static bool running = false;
    static bool turningleft = (random(0, 2) == 0 ? true : false);

    lazySetup();

    dist = getDistanceFromObstacle();

    if (dist <= 20) {
        if (running) {
            walkMode();
            running = false;
        }
        for (i = 0; i < random(3, 6); i++) {
            goBackward();
        }

        if (random(0, 10) < 3) {
            turningleft = !turningleft;
        }
        for (i = 0; i < random(3, 10); i++) {
            if (turningleft) {
                turnLeft();
            } else {
                turnRight();
            }
        }
    } else {
        if (dist >= 50 && !running) {
            runMode();
            running = true;
        } else if (dist < 40 && running) {
            walkMode();
            running = false;
        }
        goForward();
    }
}

/* vi: set filetype=c autoindent expandtab softtabstop=4 shiftwidth=4 : */
