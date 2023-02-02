#include "camera.h"
#include "arducam.h"

#include "stlogo.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_cdc_if.h"

static uint32_t captureStart;
extern int myprintf(const char *format, ...);

static void camera_get_image();
BaseType_t write_fifo_to_buffer(uint32_t length);
unsigned char cameraReady;

void camera_setup(){

	cameraReady = pdFALSE;
	/**
	 * Detect and initialize the Arduchip interface.
	 * Ensure that the OV5642 is powered on.
	 * Detect and initialize the OV5642 sensor chip.
	 */
	if (   arduchip_detect()
		&& arducam_exit_standby()
		&& ov5642_detect()
	) {

		osDelay(100);

		if (!ov5642_configure()) {
			myprintf("camera_task: ov5642 configure failed\n\r");
			return;
		} else {
			myprintf("camera: setup complete\n\r");
			cameraReady = pdTRUE;
			osDelay(100);
		}
	} else {
		myprintf("camera: setup failed\n\r");
	}
}

/**
 * Capture an image from the camera.
 */
void camera_initiate_capture(){

	uint8_t done = 0;

	myprintf("camera: initiate capture\n\r");

	if (!cameraReady) {
		myprintf("camera: set up camera before capture\n\r");
	}

	/* Initiate an image capture. */
	if (!arduchip_start_capture()) {
		myprintf("camera: initiate capture failed\n\r");
		return;
	}

	/* wait for capture to be done */
	captureStart = (uint32_t)xTaskGetTickCount();
	while(!arduchip_capture_done(&done) || !done){

		if ((xTaskGetTickCount() - captureStart) >= CAPTURE_TIMEOUT) {
			myprintf("camera: capture timeout\n\r");
			return;
		}
	}

	myprintf("camera: capture complete\n\r");

	camera_get_image();

	return;

}

void camera_get_image(){

	/* Determine the FIFO buffer length. */
	uint32_t length = 0;
	if (arduchip_fifo_length(&length) == pdTRUE) {
		myprintf("camera: captured jpeg image -> %lu bytes\n\r", length);
		write_fifo_to_buffer(length);
	} else {
		myprintf("camera: get fifo length failed\n\r");
	}

	return;
}

uint8_t fifoBuffer[BURST_READ_LENGTH];
#define MAX_PIC_SIZE 64000
#define BITMAP_SIZE 3600
uint8_t pic_buffer[MAX_PIC_SIZE];
uint8_t bitmap[BITMAP_SIZE];

int pic_index = 0;

static void get_pixel(int* pDst, const uint8_t *pSrc, int luma_only, int num_comps)
{
   int r, g, b;
   if (num_comps == 1)
   {
      r = g = b = pSrc[0];
   }
   else if (luma_only)
   {
      const int YR = 19595, YG = 38470, YB = 7471;
      r = g = b = (pSrc[0] * YR + pSrc[1] * YG + pSrc[2] * YB + 32768) / 65536;
   }
   else
   {
      r = pSrc[0]; g = pSrc[1]; b = pSrc[2];
   }
   pDst[0] = r; pDst[1] = g; pDst[2] = b;
}

// https://helloacm.com
inline float
BilinearInterpolation(float q11, float q12, float q21, float q22, float x1, float x2, float y1, float y2, float x, float y)
{
    float x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 - x1;
    y2y1 = y2 - y1;
    x2x = x2 - x;
    y2y = y2 - y;
    yy1 = y - y1;
    xx1 = x - x1;
    return 1.0 / (x2x1 * y2y1) * (
        q11 * x2x * y2y +
        q21 * xx1 * y2y +
        q12 * x2x * yy1 +
        q22 * xx1 * yy1
    );
}

static void display_bitmap(const uint8_t * image, int width, int height, int comps, int start_x, int start_y, int scale)
{

	int a[3];

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			get_pixel(a, image + (y * width + x) * comps, 0, comps);
			uint32_t argb = (0xFF << 24) | (a[0] << 16) | (a[1] << 8) | (a[0]);
			for (int yy = 0; yy < scale; yy++) {
				for (int xx = 0; xx < scale; xx++) {
					BSP_LCD_DrawPixel(start_x + x * scale + xx, start_y + y * scale + yy, argb);
				}
			}
		}
	}
	uint32_t old_col = BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawRect(start_x, start_y, width * scale, height * scale);
	BSP_LCD_SetTextColor(old_col);
}

BaseType_t
write_fifo_to_buffer(uint32_t length)
{
#if 0	// display captured image
	/* Write the FIFO contents to disk. */
	uint16_t chunk = 0;

	free(ptr_picture);
	// jpeg pic size
	unsigned int jpeg_size = length*sizeof(uint8_t);
	// allocate memory to store jpeg picture
	if((ptr_picture = malloc(jpeg_size)) == NULL){
		myprintf("camera: ran out of memory\n\r");
	}else{
		myprintf("camera: allocated %d bytes of memory for picture\n\r", malloc_usable_size(ptr_picture));
	}

	for (uint16_t i = 0; length > 0; ++i) {

		chunk = MIN(length, BURST_READ_LENGTH);
		arduchip_burst_read(fifoBuffer, chunk);
		length -= chunk;

		// maybe send the fifo buffer to LabVIEW for displaying ....

	}
#else
	// test image: make sure to build the project with -Og to show this static .bmp image
	// 		project properties -> C/C++ Build -> Settings -> Optimization | Optimize for debugging (-Og)
    BSP_LCD_DrawBitmap(80, 180, (uint8_t *)stlogo);
#endif
    osDelay(500);

	return pdTRUE;
}


