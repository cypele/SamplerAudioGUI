#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}



#ifdef SIMULATOR
    // Kompilacja dla symulatora – pomiń FreeRTOS
#else
void Screen1Presenter::swButtonRecordSemaphoreGive()
{
	model->StartRecording();
}


void Screen1Presenter::swButtonStopSemaphoreGive()
{
	model->StopRecording();
}

void Screen1Presenter::swButtonPlaySemaphoreGive()
{
	model->StartPlaying();
}

#endif
