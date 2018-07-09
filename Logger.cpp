#include <QMessageBox>
#include <QTimer>
#include <QDateTime>
#include <QMutex>
#include <QFile>
#include <QTextStream>
#include "widget/QConsole.h"
#include "GlobalShared.h"

#include "Slurry.h"

#include "Logger.h"

static void MessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg);

QMutex* Logger::mutex;
QFile* Logger::file;
QTextStream* Logger::text_stream;
QSharedPointer<Logger> Logger::pInst;

Logger::Logger()
{
	mutex = new QMutex;
	file = new QFile("Log.log");
	//if (!file->open(QIODevice::ReadWrite | QIODevice::Append | QIODevice::Text))
	if (!file->open(QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text))
	{
		return;
	}

	text_stream = new QTextStream(file);
	qInstallMessageHandler(MessageHandler);
}

Logger::~Logger()
{
	text_stream->flush();
	delete text_stream;
	file->close();
	delete file;
	delete mutex;
}


Logger* Logger::GetInstance()
{
	if (pInst.isNull())
	{
		pInst = QSharedPointer<Logger>(new Logger);
	}
	return pInst.data();
}

void Logger::PrintLog(const QString &msg)
{
	if (GlobalShared::slurry != nullptr)
	{
		GetInstance();
		//从text栏输出 使用信号槽是为了跨线程
		QMetaObject::invokeMethod(GlobalShared::slurry->ui.logText, "displayMessage", Qt::QueuedConnection, Q_ARG(QString, msg));
		//emit slurry->printLog(msg);
	}
}

void Logger::WriteLog(const QString &msg)
{
	GetInstance();
	QMutexLocker locker(mutex);

	QString current_date_time = QDateTime::currentDateTime().toString("MM-dd hh:mm:ss zzz");
	QString current_date = QString("%1").arg(current_date_time);
	QString message = QString("[%1] %2").arg(current_date).arg(msg);

	(*text_stream) << message << endl;
	file->flush();
}


void Logger::Debug(const QString &msg)
{
	qDebug() << msg;
}


void Logger::WarnAndQuit(const QString &msg)
{
	//警告对话框
	QMessageBox::warning(GlobalShared::slurry, QStringLiteral("错误"), msg);
	//主程序退出
	QTimer::singleShot(0, QApplication::instance(), SLOT(quit()));
}

void Logger::messageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	mutex->lock();
	//text的内容暂时没有用到
	QString text;
	switch (type)
	{
	case QtDebugMsg:
		text = QStringLiteral("Debug");
		break;

	case QtWarningMsg:
		text = QStringLiteral("Warning");
		break;

	case QtCriticalMsg:
		text = QStringLiteral("Critical");
		break;

	case QtFatalMsg:
		text = QStringLiteral("Fatal");
	}

	QString context_info = QString("%1 Line %2").arg(QString(context.file)).arg(context.line);
	QString current_date_time = QDateTime::currentDateTime().toString("MM-dd hh:mm:ss zzz");
	QString current_date = QString("%1").arg(current_date_time);
	QString message = QString("[%1] %2 [%3] %4").arg(current_date).arg(msg).arg(context_info);// .arg(text);

	//写入日志文件
	(*text_stream) << message << endl;
	file->flush();

	//Console输出
	fprintf(stderr, "%s\n", msg.toLatin1().constData());
	fflush(stderr);

	mutex->unlock();
}


static void MessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	Logger::messageHandler(type, context, msg);
}

