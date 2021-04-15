#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

uint16_t line_detection(uint8_t *buffer);
float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
float get_find_me(void);

#endif /* PROCESS_IMAGE_H */