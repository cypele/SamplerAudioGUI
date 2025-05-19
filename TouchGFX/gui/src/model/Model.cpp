#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#ifdef SIMULATOR
    // Kompilacja dla symulatora – pomiń FreeRTOS
#else
    #include "cmsis_os.h" // Tylko dla targetu embedded
    #include "main.h"
	extern osMessageQueueId_t CommandSDQueueHandle;
	extern osMessageQueueId_t CommandAudioQueueHandle;


	Audio_SdCard_Command_t StopRecordCommand = CMD_STOP_RECORDING;
	Audio_SdCard_Command_t StartRecordCommand = CMD_START_RECORDING;
	Audio_SdCard_Command_t StartPlayCommand = CMD_START_PLAYING;


	extern "C" {
		void Model::StartRecording()
		{
		    osMessageQueuePut(CommandAudioQueueHandle, &StartRecordCommand, 0, 0);
		    osMessageQueuePut(CommandSDQueueHandle, &StartRecordCommand, 0, 0);
		}
	}


	extern "C" {
		void Model::StopRecording()
		{
		    osMessageQueuePut(CommandAudioQueueHandle, &StopRecordCommand, 0, 0);
		    osMessageQueuePut(CommandSDQueueHandle, &StopRecordCommand, 0, 0);

		}
	}

	extern "C" {
		void Model::StartPlaying()
		{
		    osMessageQueuePut(CommandAudioQueueHandle, &StartPlayCommand, 0, 0);
		    osMessageQueuePut(CommandSDQueueHandle, &StartPlayCommand, 0, 0);

		}
	}



#endif


Model::Model() : modelListener(0)
{

}

void Model::tick()
{

}


