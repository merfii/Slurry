#ifndef RobotException_H
#define RobotException_H

/* Classes standards. */
#include <iostream>                /* Classe std::ostream.    */
#include <string>                  /* Classe string.     */
#include <stdarg.h>


class RobotMoveException : public std::exception
{

public:
	RobotMoveException();
	RobotMoveException(const int code);
	RobotMoveException(const int code, const std::string & msg);
	RobotMoveException(const int code, const char* format, ...);

	virtual ~RobotMoveException() throw() {}
	int code;
	const char *what() const throw();

private:
	std::string msg;
};


/*!
   \class RobotExcetion
   \ingroup group_core_debug
   \brief error that can be emited by ViSP classes.

   This class inherites from the standard std::exception contained in the C++
   STL.
   It is therefore possible to catch RobotExcetion with any other derivative of
   std::exception in the same catch.
 */
class RobotException : public std::exception
{
  protected:

    //! Contains the error code, see the errorCodeEnum table for details.
    int code;

    //! Contains an error message (can be empty)
    std::string message;

    //! Set the message container
    void setMessage(const char* format, va_list args);

    //!  forbid the empty constructor (protected)
    RobotException(): code(notInitialized), message("") { };

  public:

    enum generalExceptionEnum
    {
      memoryAllocationError,
      memoryFreeError,
      functionNotImplementedError,
      ioError,
      cannotUseConstructorError,
      notImplementedError,
      divideByZeroError,
      dimensionError,
      fatalError,
      badValue, /*!< Used to indicate that a value is not in the allowed range. */
      notInitialized /*!< Used to indicate that a parameter is not initialized. */
    } ;

    RobotException (const int code, const char* format, va_list args);
    RobotException (const int code, const char* format, ...);
    RobotException (const int code, const std::string & msg);
    RobotException (const int code);

    virtual ~RobotException() throw() {}
    //! Send the object code.
    int getCode (void);

    //! Send a reference (constant) related the error message (can be empty).
    const std::string &getStringMessage (void) const;
    //! send a pointer on the array of  \e char related to the error string.
    //! Cannot be  \e NULL.
    const char *getMessage (void) const;

    //! Print the error structure.
    friend  std::ostream & operator << (std::ostream & os, const RobotException & art);

    const char* what () const throw();
};

#endif 