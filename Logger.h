#pragma once

#include <QSharedPointer>
#include <QString>

class QMutex;
class QFile;
class QTextStream;

class Logger
{

public:
	virtual ~Logger();

	static void PrintLog(const QString &msg);	//仅从text栏滚动显示
	static void WriteLog(const QString &msg);	//仅写入日志文件
	static void Debug(const QString &msg);	//在控制台输出
	static void WarnAndQuit(const QString &msg); //关键文件错误 警告并退出

	//qDebug() qWarning() qCritical() qFatal() 

	static void messageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg);

protected:
	Logger();

private:
	Logger(const Logger&);
	void operator = (const Logger &);

	static Logger *GetInstance();
	static QMutex *mutex;
	static QFile *file;
	static QTextStream *text_stream;
	static QSharedPointer<Logger> pInst;
};

//写入日志文件和 Cosole控制台
