#include <Bluepad32.h>
#include <Preferences.h>

#include "controller.h"
#include "melty_config.h"
#include "subsystems/storage.h"

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// todo - save the trims & spin speed
// todo - work out how to get these into melty_config.h properly
int spin_target_rpms[] =  {600, 800, 1000, 1200, 1500, 1800, 2100, 2500, 3000};
float translation_trims[] = {1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2, 10.0};
#define NUM_TARGET_RPMS 9
int target_rpm_index = 3;

#define NUM_TRANS_TRIMS 13;
int target_trans_trim = 4;

bool reverse_spin = false;
bool connected = false;

long last_updated_millis;
prev_state previous_state;

// Rather than instantiate a new control state every time something updates, we just have two states
// and swap pointers back and forth
ctrl_state state_green;
ctrl_state state_blue;

ctrl_state* previous_ctrls = &state_green;
ctrl_state* new_ctrls = &state_blue;
bool prev_ctrls_are_green = true;

ctrl_state* ctrl_update(bool upd8) {
    long now = millis();

    if (upd8) {
        for (auto myController : myControllers) {
            if (myController && myController->isConnected() && myController->hasData()) {
                // create a new control state
                last_updated_millis = now;
                return get_state(myController);
            }
        }
    }

    // otherwise, recycle the existing control state
    previous_ctrls->connected = connected;

    // reset all the config buttons
    previous_ctrls->trim_left = false;
    previous_ctrls->trim_right = false;

    // and check for control timeout
    if (now - last_updated_millis > CONTROL_UPDATE_TIMEOUT_MS) {
        previous_ctrls->alive = false;
    }

    return previous_ctrls;
}

// right now, this is only written and tested with an xbox bluetooth controller
ctrl_state* get_state(ControllerPtr ctl) {
    // todo - validate controller type
    // todo - expand controller type support?

    // because we're processing a new update, we know the controller is alive
    new_ctrls->alive = true;

    // gotta hold down the throttle to spin
    // this both gives us a dead-girl switch and a constantly-changing input to keep the packets flowing
    new_ctrls->spin_requested = ctl->throttle() > CONTROL_THROTTLE_MINIMUM;

    new_ctrls->translate_forback = ctl->axisRY();
    new_ctrls->translate_lr = ctl->axisX(); // plumbing it through even though it currently isn't used
    new_ctrls->turn_lr = ctl->axisRX();

    // spin direction
    if ((ctl->buttons() & XBOX_BUTTON_X) && !previous_state.reverse_spin_pressed) {
        previous_state.reverse_spin_pressed = true;
        reverse_spin = !reverse_spin;
    } else if (!(ctl->buttons() & XBOX_BUTTON_X) && previous_state.reverse_spin_pressed) {
        previous_state.reverse_spin_pressed = false;
    }

    new_ctrls->reverse_spin = reverse_spin;

    // target RPM adjustment
    // forward on the xbox controller gives negative values, for some reason
    // todo - make this only adjust while spinning?
    int lstick = ctl->axisY();

    if ((abs(lstick) > CONTROL_SPIN_SPEED_DEADZONE) && !previous_state.spin_target_rpm_changed) {
        // we've just gone past the threshold to change the target spin speed
        previous_state.spin_target_rpm_changed = true;
        if (lstick < 0 && target_rpm_index < NUM_TARGET_RPMS-1) {
            target_rpm_index++;
        } else if (lstick > 0 && target_rpm_index > 0) {
            target_rpm_index--;
        }

        get_active_store()->set_target_rpm(target_rpm_index);

    } else if ((abs(lstick) < CONTROL_SPIN_SPEED_DEADZONE) && previous_state.spin_target_rpm_changed) {
        // we've just dropped back beneath the threshold, reset the ability to change speed
        previous_state.spin_target_rpm_changed = false;
    }

    new_ctrls->target_rpm = spin_target_rpms[target_rpm_index];

    // And the trim adjustments
    // todo - save trim into config in appropriate places (trim- IMU. translate - ???)
    int dpad = ctl->dpad();

    new_ctrls->trim_left = false;
    new_ctrls->trim_right = false;

    // trim config, from the dpad
    if ((dpad & XBOX_DPAD_UP) != previous_state.increase_translate_pressed) {
        previous_state.increase_translate_pressed = !previous_state.increase_translate_pressed;
        if (previous_state.increase_translate_pressed && target_trans_trim < NUM_TRANS_TRIMS-1) {
            target_trans_trim++;
            get_active_store()->set_trans_trim(target_trans_trim);
        }
    }

    if ((dpad & XBOX_DPAD_DOWN) != previous_state.decrease_translate_pressed) {
        previous_state.decrease_translate_pressed = !previous_state.decrease_translate_pressed;
        if (previous_state.decrease_translate_pressed && target_trans_trim > 0) {
            target_trans_trim--;
            get_active_store()->set_trans_trim(target_trans_trim);
        }
    }

    new_ctrls->translate_trim = translation_trims[target_trans_trim];

    if ((dpad & XBOX_DPAD_LEFT) != previous_state.trim_left_pressed) {
        previous_state.trim_left_pressed = !previous_state.trim_left_pressed;
        if (previous_state.trim_left_pressed) {
            new_ctrls->trim_left = true;
        }
    }

    if ((dpad & XBOX_DPAD_RIGHT) != previous_state.trim_right_pressed) {
        previous_state.trim_right_pressed = !previous_state.trim_right_pressed;
        if (previous_state.trim_right_pressed) {
            new_ctrls->trim_right = true;
        }
    }

    // and swap which control set is active
    if (prev_ctrls_are_green) {
        previous_ctrls = &state_blue;
        new_ctrls = &state_green;
        prev_ctrls_are_green = false;
    } else {
        previous_ctrls = &state_green;
        new_ctrls = &state_blue;
        prev_ctrls_are_green = true;
    }
    
    return previous_ctrls;
}

void ctrl_init() {
    target_rpm_index = get_active_store()->get_target_rpm();
    target_trans_trim = get_active_store()->get_trans_trim();
}

void on_connected_controller(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }

    connected = true;

    // Anguirel says: There is an issue where scan_evt will timeout eventually causing something to print "FEX x y",
    // (where x and y are various numbers) to the console and then eventually crash. Possible occurence of
    // https://github.com/ricardoquesada/bluepad32/issues/43.  The reported workaround is to disable scanning
    // for new controllers once a controller has connected.
    BP32.enableNewBluetoothConnections(false);
}

void on_disconnected_controller(ControllerPtr ctl) {
    connected = false;

    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }

    BP32.enableNewBluetoothConnections(true);
}