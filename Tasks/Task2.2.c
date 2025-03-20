#include "dlab_def.h"
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAXS 10000

// Declare global variables
pthread_t Control;  // Control thread
sem_t data_avail;   // Semaphore

float theta[MAXS];  // Array to store motor positions
float ref[MAXS];    // Array for reference input

float Kp = 1.0;      // Proportional gain
float Fs = 200.0;    // Sampling frequency
float run_time = 30; // RUNTIME (set to at least 30 sec for sustained oscillations)
int motor_number = 7; // Motor number (CHANGE IF NEEDED)

// Variables for Ultimate Sensitivity Method
float Ku = 0.0;  // Ultimate Gain
float Pu = 0.0;  // Period of Oscillation
float Ti = 0.0;  // Integral Time
float Td = 0.0;  // Derivative Time

// Function prototype
void *ControlLoop(void *arg);

int main() {
    char selection;

    while (1) {
        // Display menu
        printf("\nMenu:\n");
        printf("r: Run control\n");
        printf("p: Change Kp\n");
        printf("k: Increase Kp\n");
        printf("f: Change sampling frequency\n");
        printf("t: Change run time\n");
        printf("u: Select input type Step\n");
        printf("g: Plot response\n");
        printf("h: Save response\n");
        printf("i: Change Ti\n");
        printf("d: Change Td\n");
        printf("q: Quit\n");
        printf("Enter selection: ");
       
        selection = getchar();
        getchar();  // Consume newline

        switch (selection) {
            case 'r': // Run control
                sem_init(&data_avail, 0, 0);
                Initialize(Fs, motor_number);
                pthread_create(&Control, NULL, ControlLoop, NULL);
                pthread_join(Control, NULL);
                Terminate();
                sem_destroy(&data_avail);
                break;

            case 'p': // Change Kp
                printf("Enter new Kp: ");
                scanf("%f", &Kp);
                getchar();
                break;

            case 'k': // Increase Kp for Ku testing
                Kp += 0.1;
                printf("Increased Kp to: %f\n", Kp);
                break;

            case 'f': // Change sampling frequency
                printf("Enter new Fs: ");
                scanf("%f", &Fs);
                getchar();
                break;

            case 't': // Change run time
                printf("Enter new run time (s): ");
                scanf("%f", &run_time);
                getchar();
                break;

            case 'u': // Choose input type (Step)
                printf("Step input selected. Enter magnitude: ");
                float magnitude;
                scanf("%f", &magnitude);
                getchar();

                for (int i = 0; i < MAXS; i++) {
                    ref[i] = magnitude * M_PI / 180;  // Convert degrees to radians
                }
                break;

            case 'g': // Plot response
                plot(ref, theta, Fs, run_time * Fs, SCREEN, "Motor Position", "Time", "Angle (radians)");
                break;

            case 'h': // Save plot
                plot(ref, theta, Fs, run_time * Fs, PS, "Motor Position", "Time", "Angle (radians)");
                break;

            case 'i': // Change Ti
                printf("Enter new Ti: ");
                scanf("%f", &Ti);
                getchar();
                break;

            case 'd': // Change Td
                printf("Enter new Td: ");
                scanf("%f", &Td);
                getchar();
                break;

            case 'q': // Quit program
                return 0;

            default:
                printf("Invalid selection\n");
        }
    }
}

// Control thread implementation
void *ControlLoop(void *arg) {
    int k = 0;
    int no_of_samples = (int)(run_time * Fs);
    float motor_position, error, control_signal;
    float prev_motor_position = 0;
    float prev_peak_time = 0;
    float current_time = 0;
    int oscillation_count = 0;

    while (k < no_of_samples) {
        sem_wait(&data_avail);  // Wait for encoder data

        motor_position = EtoR(ReadEncoder());  // Get motor position
        error = ref[k] - motor_position;  // Compute error
        control_signal = Kp * error;  // Compute control signal

        DtoA(VtoD(control_signal)); // Send to D/A converter

        theta[k] = motor_position;  // Store position data
        prev_motor_position = motor_position;

        k++;
    }

    pthread_exit(NULL);
}
