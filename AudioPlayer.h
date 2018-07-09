                 #pragma once
#include <QObject>

class QSoundEffect;
class AudioPlayer :	public QObject
{
	Q_OBJECT

public:
	static AudioPlayer* GetInstance();
	~AudioPlayer();
	void playCalibrationWarning();
	void playScanWarning();
	void playAlarm();
	void playCameraCapture();
	void play1();
	void play2();
	void playSlowAndSteady(bool play);

private slots:
	void playSlowAndSteadyCycle();
private:
	AudioPlayer();
	AudioPlayer(const AudioPlayer &);	//Not implemented
	void operator=(const AudioPlayer &);	//Not implemented

	static QSharedPointer<AudioPlayer> pInst;
	QSoundEffect *alarm, *sound2, *nuclear, *cameraCap, *slowSteady;

	int slowSteadyCount;
};

