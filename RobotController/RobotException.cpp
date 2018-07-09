#include <stdio.h>
#include "RobotException.h"


RobotMoveException::RobotMoveException()
{
	;
}

RobotMoveException::RobotMoveException(int code)
{
	this->code = code;
}

RobotMoveException::RobotMoveException(const int code, const char* format, ...)
{
	this->code = code;
	msg = std::string(format);
}

const char* RobotMoveException::what() const
{
	return msg.c_str();
}




RobotException::RobotException(int id)
	: code(id), message()
{
}

RobotException::RobotException(int id, const std::string & msg)
	: code(id), message(msg)
{
}

RobotException::RobotException(int id, const char* format, ...)
	: code(id), message()
{
	va_list args;
	va_start(args, format);
	setMessage(format, args);
	va_end(args);
}

RobotException::RobotException(const int id, const char* format, va_list args)
	: code(id), message()
{
	setMessage(format, args);
}



void RobotException::setMessage(const char* format, va_list args)
{
	char buffer[1024];
	vsnprintf(buffer, 1024, format, args);
	std::string msg(buffer);
	message = msg;
}

const char *RobotException::getMessage(void) const
{
	return (this->message).c_str();
}

const std::string &RobotException::getStringMessage(void) const
{
	return this->message;
}

int
RobotException::getCode(void)
{
	return this->code;
}

/*!
Overloading of the what() method of std::exception to return the RobotException
message.

\return pointer on the array of  \e char related to the error string.
*/
const char* RobotException::what() const throw()
{
	return (this->message).c_str();
}


 std::ostream &
operator << (std::ostream & os, const RobotException & error)
{
	os << "Error [" << error.code << "]:\t" << error.message << std::endl;

	return os;
}
