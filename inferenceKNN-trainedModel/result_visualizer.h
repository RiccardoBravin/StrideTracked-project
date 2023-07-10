

#ifndef RICCARDO_RESULT_VISUALIZER_H_
#define RICCARDO_RESULT_VISUALIZER_H_



// Called by the main loop to produce some output based on the x and y values
void class_to_led(int class_label, bool log_silence = false);

static int protothreadBlinkLED(struct pt *pt, bool &training);

#endif  // RICCARDO_RESULT_VISUALIZER_H_
