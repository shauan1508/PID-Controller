#include "dlab_def.h"
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAXS 10000

// Global variables
pthread_t ControlThread;
sem_t data_avail;

float theta[MAXS];  // Motor position
float ref[MAXS];    // Reference signal

// PID Parameters
float Kp = 65;
float Ti = 0.025;
float Td = 0.00625;
float N = 20;

// Sampling parameters
float Fs = 200.0;
float run_time = 10.0;
int motor_number = 9;

// Control loop
void *ControlLoop(void *arg) {
    int k = 0;
    int no_of_samples = (int)(run_time * Fs);
    float motor_position, error, control_signal;
    float prev_error = 0;
    float integral = 0;
    float derivative = 0;
    float Ts = 1.0 / Fs;

    while (k < no_of_samples) {
        sem_wait(&data_avail);

        motor_position = EtoR(ReadEncoder());
        error = ref[k] - motor_position;

        integral += error * Ts;
        derivative = (error - prev_error) / Ts;

        control_signal = Kp * error + (Kp / Ti) * integral + Kp * Td * derivative;

        DtoA(VtoD(control_signal));
        theta[k] = motor_position;
        prev_error = error;
        k++;
    }

    pthread_exit(NULL);
}

int main() {
    char selection;
    int input_type;
    float magnitude, frequency, duty_cycle;
    int i;

    // Default reference = step input of 50 degrees
    for (i = 0; i < MAXS; i++) {
        ref[i] = 50.0 * M_PI / 180.0;
    }

    while (1) {
        // Menu
        printf("\nMenu:\n");
        printf("r: Run control\n");
        printf("p: Change Kp\n");
        printf("i: Change Ti\n");
        printf("d: Change Td\n");
        printf("n: Change N\n");
        printf("f: Change sampling frequency\n");
        printf("t: Change run time\n");
        printf("u: Select input type (Step or Square Wave)\n");
        printf("g: Plot response\n");
        printf("h: Save response\n");
        printf("q: Quit\n");
        printf("Enter selection: ");

        selection = getchar();
        getchar(); // Consume newline

        switch (selection) {
            case 'r':
                sem_init(&data_avail, 0, 0);
                Initialize(Fs, motor_number);
                pthread_create(&ControlThread, NULL, ControlLoop, NULL);
                pthread_join(ControlThread, NULL);
                Terminate();
                sem_destroy(&data_avail);
                break;

            case 'p':
                printf("Enter new Kp: ");
                scanf("%f", &Kp);
                getchar();
                break;

            case 'i':
                printf("Enter new Ti: ");
                scanf("%f", &Ti);
                getchar();
                break;

            case 'd':
                printf("Enter new Td: ");
                scanf("%f", &Td);
                getchar();
                break;

            case 'n':
                printf("Enter new N value: ");
                scanf("%f", &N);
                getchar();
                break;

            case 'f':
                printf("Enter new sampling frequency (Hz): ");
                scanf("%f", &Fs);
                getchar();
                break;

            case 't':
                printf("Enter new run time (s): ");
                scanf("%f", &run_time);
                getchar();
                break;

            case 'u':
                printf("Select input type:\n");
                printf("1: Step Input\n");
                printf("2: Square Wave Input\n");
                printf("Enter choice (1 or 2): ");
                scanf("%d", &input_type);
                getchar();

                if (input_type == 1) {
                    printf("Enter step magnitude (degrees): ");
                    scanf("%f", &magnitude);
                    getchar();

                    for (i = 0; i < MAXS; i++) {
                        ref[i] = magnitude * M_PI / 180.0;
                    }

                    printf("Step input set to %.2f degrees.\n", magnitude);
                }
                else if (input_type == 2) {
                    printf("Enter magnitude (deg), frequency (Hz), duty cycle (%%): ");
                    scanf("%f %f %f", &magnitude, &frequency, &duty_cycle);
                    getchar();

                    magnitude = magnitude * M_PI / 180.0;
                    Square(ref, MAXS, Fs, magnitude, frequency, duty_cycle);

                    printf("Square wave input generated.\n");
                }
                else {
                    printf("Invalid selection.\n");
                }
                break;

            case 'g':
                plot(ref, theta, Fs, (int)(run_time * Fs), SCREEN, "Motor Position", "Time", "Angle (rad)");
                break;

            case 'h':
                plot(ref, theta, Fs, (int)(run_time * Fs), PS, "Motor Position", "Time", "Angle (rad)");
                break;

            case 'q':
                printf("Exiting program.\n");
                return 0;

            default:
                printf("Invalid selection.\n");
        }
    }
}
