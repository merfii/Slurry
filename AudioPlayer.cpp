#include <QSharedPointer>
#include <QTimer>
#include <QSoundEffect>

#include "AudioPlayer.h"

QSharedPointer<AudioPlayer> AudioPlayer::pInst;

AudioPlayer::AudioPlayer()
{
	//³õÊ¼»¯ÉùÒô
	sound2 = new QSoundEffect;
	sound2->setSource(QUrl::fromLocalFile("Sounds/Doorbell.wav"));

	sound2->setLoopCount(5);//(QSoundEffect::Infinite);
	//sound1.setVolume(0.25f);

	alarm = new QSoundEffect;
	alarm->setSource(QUrl::fromLocalFile("Sounds/Alarm.wav"));

	nuclear = new QSoundEffect;
	nuclear->setSource(QUrl::fromLocalFile("Sounds/RedAlert.wav"));
	nuclear->setVolume(0.2);

	cameraCap = new QSoundEffect;
	cameraCap->setSource(QUrl::fromLocalFile("Sounds/CameraCapture.wav"));

	slowSteady = new QSoundEffect;
	slowSteady->setSource(QUrl::fromLocalFile("Sounds/SlowAndStead.wav"));
}

AudioPlayer::~AudioPlayer()
{
	delete sound2;
	delete alarm;
	delete nuclear;
	delete cameraCap;
	delete slowSteady;
}

AudioPlayer* AudioPlayer::GetInstance()
{
	if (pInst.isNull())
	{
		pInst = QSharedPointer<AudioPlayer>(new AudioPlayer);
	}
	return pInst.data();
}

void AudioPlayer::play2()
{
	sound2->play();
}

void AudioPlayer::playCalibrationWarning()
{
	nuclear->play();
}

void AudioPlayer::playScanWarning()
{
	nuclear->play();
}

void AudioPlayer::playAlarm()
{
	alarm->play();
}

void AudioPlayer::playCameraCapture()
{
	cameraCap->play();
}

void AudioPlayer::playSlowAndSteady(bool play)
{
	if (play)
	{
		slowSteadyCount = 0;
		QTimer::singleShot(2000, this, SLOT(playSlowAndSteadyCycle()));
	}
	else
	{
		slowSteadyCount = 10;
	}
}

void AudioPlayer::playSlowAndSteadyCycle()
{
	slowSteady->play();
	if (slowSteadyCount++ < 10)
		QTimer::singleShot(3000 + slowSteadyCount * 1000, this, SLOT(playSlowAndSteadyCycle()));

}