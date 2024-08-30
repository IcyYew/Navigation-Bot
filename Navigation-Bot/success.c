#include "success.h"
void success(void)
{
	oi_t *sensor_data = oi_alloc();
	oi_init(sensor_data);


	unsigned char notes1[16] = {74, 74, 86, 81, 80, 79, 77, 74, 77, 79, 72, 72, 86, 81, 80, 79};
	unsigned char notes2[14] = {77, 74, 77, 79, 71, 71, 86, 81, 80, 79, 77, 74, 77, 79};
	unsigned char notes3[10] = {70, 70, 86, 81, 80, 79, 77, 74, 77, 79};

 	unsigned char duration1[16] = {8, 8, 16, 24, 16, 16, 16, 8, 8, 8, 8, 8, 16, 24, 16, 16};
 	unsigned char duration2[14] = {16, 8, 8, 8, 8, 8, 16, 24, 16, 16, 16, 8, 8, 8};
 	unsigned char duration3[10] = {8, 8, 16, 24, 16, 16, 16, 8, 8, 8};

	oi_loadSong(0, 16, notes1, duration1);
	oi_loadSong(1, 14, notes2, duration2);
	oi_loadSong(2, 10, notes3, duration3);

	oi_play_song(0);
	timer_waitMillis(3425);


    oi_play_song(1);
    timer_waitMillis(2725);

    oi_play_song(2);

	oi_free(sensor_data);


}
