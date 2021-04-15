#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static float find_me = 0;
static float distance_cm = 0;

static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

uint16_t line_detection(uint8_t *buffer)
{ 
	uint16_t i = 0, mean = 0, begin = 0, end = 0, width = 0;  
	uint8_t wrong_line = 0, stop = 0, line_not_found = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	for (uint32_t i = 0; i < IMAGE_BUFFER_SIZE; ++i)
	{
		mean += buffer[i];
	}

	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0; 
		while (stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
			if (buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean ) //on detecte la ligne de début
			{
				stop = 1;
				begin = i;
			}
			i++;
		}
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin) {
			stop = 0;
			while (stop == 0 && i < IMAGE_BUFFER_SIZE) {
				if (buffer [i] > mean && buffer[i-WIDTH_SLOPE] < mean) {
					end = i;
					stop = 1;
				}
				i++;
			}
			if (i > IMAGE_BUFFER_SIZE || !end) {
				line_not_found = 1; // pas de end 
			}
		} else {
			line_not_found = 1; // pas de begin 
		}
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH) {
			wrong_line = 1;
			stop = 0;
			begin = 0;
			i = end;
			end = 0;
		}
	} while (wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
		
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }

    //chThdSleepMilliseconds(12);
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr; //pixels stored in this pointer that acts like an array
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint8_t pixel_data = 0;
	uint8_t temp_data = 0; 
	uint16_t line_width = 0; 

	systime_t time;


    while(1){
    	time = chVTGetSystemTime();
    	bool send_to_computer = true;

    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		for (uint16_t i = 0; i < 2*IMAGE_BUFFER_SIZE; i+=2) 
		{
			temp_data = ((uint8_t)img_buff_ptr[i] & 7) << 3; 
			pixel_data = ((uint8_t)img_buff_ptr[i+1] & 224) >> 5;
			pixel_data |= temp_data; 
			image[i/2] = pixel_data;
		}

		line_width = line_detection(image);

		if(line_width){
			distance_cm = PXTOCM/line_width;
		}

		if(send_to_computer){
			//sends the data buffer of the given size to the computer
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		send_to_computer = !send_to_computer;
		
		chThdSleepUntilWindowed(time, time + MS2ST(10));
		//chprintf((BaseSequentialStream *)&SDU1, "capture␣width␣=␣%u\n", distance_cm); // pk on met %d alors que unsigned int et pk chVTGetSystemTime()-time ???
    }


}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

float get_find_me(void){
	return find_me;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}



	
	












