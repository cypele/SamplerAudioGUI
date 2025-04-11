#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#ifdef SIMULATOR
    // Kompilacja dla symulatora – pomiń FreeRTOS
#else
    #include "cmsis_os.h" // Tylko dla targetu embedded
	#include "semphr.h"
	#include "queue.h"
    #include "main.h"
	extern SemaphoreHandle_t StartRecordingSemaphoreHandle;
	extern SemaphoreHandle_t StopRecordingSemaphoreHandle;
	extern SemaphoreHandle_t StartSavingSemaphoreHandle;
	extern SemaphoreHandle_t StartPlayingSemaphoreHandle;
	extern QueueHandle_t CommandSDQueueHandle;
	extern QueueHandle_t CommandAudioQueueHandle;


	Audio_SdCard_Command_t StopRecordCommand = CMD_STOP_RECORDING;
	Audio_SdCard_Command_t StartRecordCommand = CMD_START_RECORDING;
	Audio_SdCard_Command_t StartPlayCommand = CMD_START_PLAYING;


	extern "C" {
		void Model::StartRecording()
		{
			xQueueSend(CommandAudioQueueHandle, &StartRecordCommand, portMAX_DELAY);
			xQueueSend(CommandSDQueueHandle, &StartRecordCommand, portMAX_DELAY);


		}
	}


	extern "C" {
		void Model::StopRecording()
		{
			xQueueSend(CommandAudioQueueHandle, &StopRecordCommand, portMAX_DELAY);
			xQueueSend(CommandSDQueueHandle, &StopRecordCommand, portMAX_DELAY);

		}
	}

	extern "C" {
		void Model::StartPlaying()
		{
			xQueueSend(CommandSDQueueHandle, &StartPlayCommand, portMAX_DELAY);
			xQueueSend(CommandAudioQueueHandle, &StartPlayCommand, portMAX_DELAY);

		}
	}



#endif


Model::Model() : modelListener(0)
{

}

void Model::tick()
{

}


