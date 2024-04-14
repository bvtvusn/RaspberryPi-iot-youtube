import pigpio

#LED
GPIO_led=12
pi = pigpio.pi() # Connect to local Pi.

#BUTTON
GPIO_btn = 5
pi.set_mode(GPIO_btn, pigpio.INPUT)
pi.set_pull_up_down(GPIO_btn, pigpio.PUD_UP)


def setBrightness(_gpio, percentage):
    FREQUENCY=500
    f = pi.set_PWM_frequency(_gpio, FREQUENCY)
    norm = percentage / 100.0
    r = pi.get_PWM_range(_gpio)
    pi.set_PWM_dutycycle(_gpio, r * norm)
    

setBrightness(GPIO_led,20)
button_state = pi.read(GPIO_btn)
print(button_state)