#include <Bluepad32.h>

typedef struct ctrl_state {
    // the controller failsafes - we need to be both connected and alive for the robot to do anything other than blink LEDs at us
    bool connected;
    bool alive;

    // the master power switch - if this is false, we are not permitted to spin
    bool spin_requested;

    // translation commands and spin direction
    int translate_forback; // -512 - 512
    int translate_lr; // reserved for future implementation
    int turn_lr; // -512 - 512
    bool reverse_spin; // toggle, stays true between presses of the command button
    int target_rpm; // this may stay high even if spin_requested is false!

    float translate_trim;

    // and parameter tweaking
    // all of these are edge detectors, they'll go true once when the button is pressed and then drop back to false
    bool trim_left;
    bool trim_right;
};

typedef struct prev_state {
    // for keeping track of what's changed since last update
    bool reverse_spin_pressed;
    bool increase_translate_pressed;
    bool decrease_translate_pressed;
    bool trim_left_pressed;
    bool trim_right_pressed;
    bool spin_target_rpm_changed;
    long last_trim_at;
};

void ctrl_init();

// connect and disconnect callbacks for bpad32
void on_connected_controller(ControllerPtr ctr);
void on_disconnected_controller(ControllerPtr ctr);

// and what we all came here for, the controller interface functions
// update() should be called regardless of if there's been a controller update or not
bool is_connected();
ctrl_state* ctrl_update(bool upd8);
ctrl_state* get_state(ControllerPtr ctl);
// todo - vibrate the controller on big hits? That'd be cute.
