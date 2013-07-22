#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

struct gpio_keys_button {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int type;		/* input event type (EV_KEY, EV_SW) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	int masking_gpio; /* Added for ASP Rev B phones which generate volume-up on cradle insertion */
	int mask_high; /* 1 if masking_gpio is high when this key should be ignored */
	void (*callback)(int code, int state, void *);
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int rep:1;		/* enable input subsystem auto repeat */
};

#endif
