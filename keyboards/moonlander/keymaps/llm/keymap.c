#include QMK_KEYBOARD_H
#include "version.h"
#include "raw_hid.h"
#include "print.h"
#include <math.h>

#define KEY_COUNT (MATRIX_ROWS * MATRIX_COLS)

enum color_modes { COLOR_MODE_DEFAULT = 0, COLOR_MODE_FADEOUT = 1, COLOR_MODE_ROLLING = 2 };

#define MOON_LED_LEVEL LED_LEVEL

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  HSV_0_255_255,
  HSV_86_255_128,
  HSV_172_255_255,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
};

// /* key matrix size */
// #define MATRIX_ROWS 12
// #define MATRIX_COLS 7
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_moonlander(
    KC_EQUAL,       KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           KC_LEFT,                                        KC_PSCR,        KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,       
    KC_DELETE,      KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,           TG(1),                                          TG(1),          KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,        
    KC_BSPC,        KC_A,           KC_S,           KC_D,           KC_F,           KC_G,           TG(3),                                                                          MO(3),          KC_H,           KC_J,           KC_K,           KC_L,           LT(2,KC_SCLN),  MT(MOD_LGUI, KC_QUOTE),
    KC_LEFT_SHIFT,  MT(MOD_LCTL, KC_Z),KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         MT(MOD_RCTL, KC_SLASH),KC_RIGHT_SHIFT, 
    LT(1,KC_ESCAPE),KC_TRANSPARENT, KC_LEFT_GUI,    TD(DANCE_0),    TD(DANCE_1),    MT(MOD_LALT, KC_ESCAPE),                                                                                                MT(MOD_LCTL, KC_ESCAPE),KC_UP,          KC_DOWN,        KC_LEFT,        KC_RIGHT,       MO(1),          
    KC_SPACE,       KC_BSPC,        KC_LEFT_ALT,                    TD(DANCE_2),    KC_TAB,         KC_ENTER
  ),
  [1] = LAYOUT_moonlander(
    KC_ESCAPE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_TRANSPARENT, KC_EXLM,        KC_AT,          KC_LCBR,        KC_RCBR,        KC_PIPE,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_UP,          KC_7,           KC_8,           KC_9,           KC_ASTR,        KC_F12,         
    KC_TRANSPARENT, KC_HASH,        KC_DLR,         KC_LPRN,        KC_RPRN,        KC_GRAVE,       KC_TRANSPARENT,                                                                 KC_TRANSPARENT, KC_DOWN,        KC_4,           KC_5,           KC_6,           KC_KP_PLUS,     KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_PERC,        KC_CIRC,        KC_LBRC,        KC_RBRC,        KC_TILD,                                        KC_AMPR,        KC_1,           KC_2,           KC_3,           KC_BSLS,        KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_COMMA,       HSV_0_255_255,  HSV_86_255_128, HSV_172_255_255,RGB_MODE_FORWARD,                                                                                                RGB_TOG,        KC_TRANSPARENT, KC_DOT,         KC_0,           KC_EQUAL,       KC_TRANSPARENT, 
    RGB_VAD,        RGB_VAI,        TOGGLE_LAYER_COLOR,                RGB_SLD,        RGB_HUD,        RGB_HUI
  ),
  [2] = LAYOUT_moonlander(
    AU_TOGG,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_BOOT,        
    MU_TOGG,        KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_UP,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    MU_NEXT,        KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT, KC_TRANSPARENT,                                                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MEDIA_PLAY_PAUSE,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_BTN1,     KC_MS_BTN2,     LCTL(KC_PAGE_UP),                                                                                                LCTL(KC_PGDN),  KC_AUDIO_VOL_UP,KC_AUDIO_VOL_DOWN,KC_AUDIO_MUTE,  KC_TRANSPARENT, KC_TRANSPARENT, 
    LALT(LCTL(KC_MINUS)),LCTL(KC_Z),     KC_TRANSPARENT,                 KC_TRANSPARENT, LCTL(KC_Y),     LCTL(LSFT(KC_MINUS))
  ),
  [3] = LAYOUT_moonlander(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_UP,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT, KC_TRANSPARENT,                                                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_UP,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                                                                                 KC_TRANSPARENT, KC_LEFT,        KC_TRANSPARENT, KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_MS_BTN1,     KC_MS_BTN2,     KC_TRANSPARENT,                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

extern rgb_config_t rgb_matrix_config;

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {41,255,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {74,255,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {74,255,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {0,245,245}, {139,45,255}, {139,45,255}, {41,255,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {74,255,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {139,45,255}, {252,255,232}, {139,45,255}, {139,45,255}, {74,255,255}, {139,45,255} },

    [2] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {74,255,255}, {74,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {74,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {74,255,255}, {74,255,255}, {0,0,0}, {0,0,0} },

    [3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );   
    }
  }
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[3];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_LBRC);
        tap_code16(KC_LBRC);
        tap_code16(KC_LBRC);
    }
    if(state->count > 3) {
        tap_code16(KC_LBRC);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_LBRC); break;
        case DOUBLE_TAP: register_code16(KC_LBRC); register_code16(KC_LBRC); break;
        case DOUBLE_HOLD: register_code16(KC_LCBR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LBRC); register_code16(KC_LBRC);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_LBRC); break;
        case DOUBLE_TAP: unregister_code16(KC_LBRC); break;
        case DOUBLE_HOLD: unregister_code16(KC_LCBR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LBRC); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_RBRC);
        tap_code16(KC_RBRC);
        tap_code16(KC_RBRC);
    }
    if(state->count > 3) {
        tap_code16(KC_RBRC);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_RBRC); break;
        case DOUBLE_TAP: register_code16(KC_RBRC); register_code16(KC_RBRC); break;
        case DOUBLE_HOLD: register_code16(KC_RCBR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_RBRC); register_code16(KC_RBRC);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_RBRC); break;
        case DOUBLE_TAP: unregister_code16(KC_RBRC); break;
        case DOUBLE_HOLD: unregister_code16(KC_RCBR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_RBRC); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MINUS);
        tap_code16(KC_MINUS);
        tap_code16(KC_MINUS);
    }
    if(state->count > 3) {
        tap_code16(KC_MINUS);
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(KC_MINUS); break;
        case SINGLE_HOLD: register_code16(KC_RIGHT_GUI); break;
        case DOUBLE_TAP: register_code16(KC_MINUS); register_code16(KC_MINUS); break;
        case DOUBLE_HOLD: register_code16(KC_UNDS); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MINUS); register_code16(KC_MINUS);
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(KC_MINUS); break;
        case SINGLE_HOLD: unregister_code16(KC_RIGHT_GUI); break;
        case DOUBLE_TAP: unregister_code16(KC_MINUS); break;
        case DOUBLE_HOLD: unregister_code16(KC_UNDS); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MINUS); break;
    }
    dance_state[2].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
};


// Function to get a keycode from a letter, number, or common special character
uint16_t get_keycode_from_char(char character) {
    switch (character) {
        // Letters
        case 'a': case 'A': return KC_A;
        case 'b': case 'B': return KC_B;
        case 'c': case 'C': return KC_C;
        case 'd': case 'D': return KC_D;
        case 'e': case 'E': return KC_E;
        case 'f': case 'F': return KC_F;
        case 'g': case 'G': return KC_G;
        case 'h': case 'H': return KC_H;
        case 'i': case 'I': return KC_I;
        case 'j': case 'J': return KC_J;
        case 'k': case 'K': return KC_K;
        case 'l': case 'L': return KC_L;
        case 'm': case 'M': return KC_M;
        case 'n': case 'N': return KC_N;
        case 'o': case 'O': return KC_O;
        case 'p': case 'P': return KC_P;
        case 'q': case 'Q': return KC_Q;
        case 'r': case 'R': return KC_R;
        case 's': case 'S': return KC_S;
        case 't': case 'T': return KC_T;
        case 'u': case 'U': return KC_U;
        case 'v': case 'V': return KC_V;
        case 'w': case 'W': return KC_W;
        case 'x': case 'X': return KC_X;
        case 'y': case 'Y': return KC_Y;
        case 'z': case 'Z': return KC_Z;
        // Numbers
        case '0': case ')': return KC_0;
        case '1': case '!': return KC_1;
        case '2': case '@': return KC_2;
        case '3': case '#': return KC_3;
        case '4': case '$': return KC_4;
        case '5': case '%': return KC_5;
        case '6': case '^': return KC_6;
        case '7': case '&': return KC_7;
        case '8': case '*': return KC_8;
        case '9': case '(': return KC_9;
        // Common special characters
        case ' ': return KC_SPACE;
        case '-': case '_': return KC_MINUS;
        case '=': case '+': return KC_EQUAL;
        case '[': return KC_LBRC;
        case ']': return KC_RBRC;
        case '\\': return KC_BACKSLASH;
        case ';': return KC_COLON;
        case '\'': return KC_QUOTE;
        case '`': return KC_GRAVE;
        case ',': return KC_COMMA;
        case '.': return KC_DOT;
        case '/': case '?': return KC_SLASH;
        case '\n': return KC_ENTER;
        case '\t': return KC_TAB;
        // Modifier keys can also be included here if needed, for example:
        // case '<your-modifier-character>': return KC_LCTRL; // Left Control
        default: return KC_NO; // Return KC_NO for any other character
    }
}


// Function to find a keycode in the keymap
uint8_t keycode_to_matrix_index(uint16_t keycode) {
    for (int row = 0; row < MATRIX_ROWS; row++) {
        for (int col = 0; col < MATRIX_COLS; col++) {
            if (keymaps[0][row][col] == keycode) {
                uint8_t matrix_index = g_led_config.matrix_co[row][col];
                return matrix_index;
            }
        }
    }
    return 1;
}

static uint8_t color_mode = 0;
static uint32_t last_keypress_time[KEY_COUNT];
static uint16_t elapsed_time_since_last_press[KEY_COUNT];
static uint8_t key_color[KEY_COUNT][3];

// buffer for storing the last n keypresses
#define N_KEYPRESSES_STORED 64
#define ROLLING_MODE_MS_PER_KEY 100
static uint16_t last_n_keycode_indices[N_KEYPRESSES_STORED];
static int n_keypresses_since_reset = 0;
static uint32_t rolling_mode_start_time = 0;


void keyboard_post_init_user(void) {
    rgb_matrix_enable();
    debug_enable=true;
    //debug_matrix=true;
    //debug_keyboard=true;
    //debug_mouse=true;

    // Initialize the last_keypress_time array
    for (int i = 0; i < KEY_COUNT; i++) {
        last_keypress_time[i] = 0;
        elapsed_time_since_last_press[i] = UINT16_MAX;
        key_color[i][0] = 0;
        key_color[i][1] = 0;
        key_color[i][2] = 0;
    }
}

float age_dependent_brightness(float age, float age_full_fade_out){
    // Ensure the brightness factor scales down to 0 correctly over time
    float brightness_factor = exp(-2.0 * (age / age_full_fade_out));
    if (brightness_factor < 0) {
        brightness_factor = 0; // Ensure brightness factor does not go below 0
    }
    return brightness_factor;
}

void matrix_scan_user(void) {
    // update all timers
    for (int i = 0; i < KEY_COUNT; i++) {
        if (last_keypress_time[i] != 0) { // Check if the key has ever been pressed
            elapsed_time_since_last_press[i] = timer_elapsed(last_keypress_time[i]);
        } else {
            elapsed_time_since_last_press[i] = UINT16_MAX;
        }

        // disable to prevent integer overflow
        if (elapsed_time_since_last_press[i] > 3000){
            last_keypress_time[i] = 0;
        }
    }

    // Color according to time since keypress.
    for (int keycode = 0; keycode < KEY_COUNT; keycode++) {
        int r = key_color[keycode][0];
        int g = key_color[keycode][1];
        int b = key_color[keycode][2];

        switch(color_mode){
            case COLOR_MODE_DEFAULT: {
                rgb_matrix_set_color(keycode, r, g, b);
                break;
            }
            case COLOR_MODE_FADEOUT: {
                float age = (float)elapsed_time_since_last_press[keycode] / 1000;
                float age_full_fade_out = 1.0; // Time in ms after which the key is fully faded out

                // Ensure the brightness factor scales down to 0 correctly over time
                float brightness_factor = age_dependent_brightness(age, age_full_fade_out);

                // Apply the brightness factor to each color component
                // Use nonlinear scaling to make keys "pop" more
                r *= brightness_factor;
                g *= brightness_factor;
                b *= brightness_factor;

                rgb_matrix_set_color(keycode, r, g, b);
                break;
            }

            case COLOR_MODE_ROLLING: {
                if(rolling_mode_start_time == 0){
                    rolling_mode_start_time = timer_read32();
                }

                // check if keycode is in last_n_keycode_indices
                int n_keypresses = fmin(N_KEYPRESSES_STORED, n_keypresses_since_reset);
                int nth_key_in_sequence = -1;

                for(int i = 0; i < n_keypresses; i++){
                    if(last_n_keycode_indices[i] == keycode){
                        nth_key_in_sequence = i;
                        break;
                    }
                }

                if(nth_key_in_sequence < 0){
                    // turn off light for keys not part of the sequence
                    rgb_matrix_set_color(keycode, 0, 0, 0);
                }else{
                    // determine brightness by "state within the cycle"
                    // microseconds
                    uint32_t ms_since_start_of_sequence = timer_read32() - rolling_mode_start_time;
                    float nth_key = (float)ms_since_start_of_sequence / ROLLING_MODE_MS_PER_KEY;
                    
                    // reset the cycle
                    if(nth_key >= n_keypresses){
                        rolling_mode_start_time = timer_read32();
                        nth_key = 0;
                        ms_since_start_of_sequence = 0;
                    }

                    float key_age = nth_key - nth_key_in_sequence;
                    if(key_age < 0)
                    {
                        key_age = n_keypresses + key_age;
                    }

                    float brightness_factor = age_dependent_brightness(key_age, (float)n_keypresses);
                    r = 255.f * brightness_factor;
                    g = 255.f * brightness_factor;
                    b = 255.f * brightness_factor;

                    rgb_matrix_set_color(keycode, r, g, b);
                }

                break;
            }
        }   
    }
}

void print_bytestream(uint8_t *data, uint8_t length) {
    dprint("raw_hid_receive: [");
    for(uint8_t i = 0; i < length; i++) {
        dprintf("%02X", data[i]);
        if (i < length - 1) {
            dprint(", ");
        }
    }
    dprintln("]");
}

#define RAW_ENABLE 1
#ifdef RAW_ENABLE
void raw_hid_receive(uint8_t *data, uint8_t length) {
    print_bytestream(data, length);

    switch (data[0]) {
        // clear all leds
        case 1: {
            switch (data[1]) {
                // Clear all
                case 0: {
                    dprint("Clearing rgb matrix\n");
                    n_keypresses_since_reset = 0;
                    rgb_matrix_set_flags(LED_FLAG_NONE);
                    rgb_matrix_set_color_all(0, 0, 0);
                    keyboard_post_init_user();
                    break;
                }
            }
            break;
        }

        // Control individual leds
        case 2: {
            switch (data[1]) {
                case 0: {
                    uint8_t index = data[2];
                    uint8_t r     = data[3];
                    uint8_t g     = data[4];
                    uint8_t b     = data[5];
                    char c = index;
                    uint16_t kc = get_keycode_from_char(c);
                    const uint8_t keycode_index = keycode_to_matrix_index(kc);
                    last_n_keycode_indices[n_keypresses_since_reset % N_KEYPRESSES_STORED] = keycode_index;
                    n_keypresses_since_reset += 1;

                    dprintf("char: %c, keycode: %d\n", c, kc);
                    dprintf("rgb_matrix_set_color: %d: %d/%d/%d\n", keycode_index, r, g, b);
                    // rgb_matrix_set_color(keycode_index, r, g, b);
                    key_color[keycode_index][0] = r;
                    key_color[keycode_index][1] = g;
                    key_color[keycode_index][2] = b;
                    last_keypress_time[keycode_index] = timer_read32();
                    break;
                }
            }
            break;
        }

        // control mode
        case 3: {
            color_mode = data[1];
            break;
        }
    }
}
#endif
