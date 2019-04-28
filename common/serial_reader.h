// USB-reader
#include <string>
#include "termios.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>

/* baudrate settings are defined in <asm/termbits.h>, which is
   included by <termios.h> */
#define BAUDRATE B9600  // Change as needed, keep B

/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyO1" //Beaglebone Black serial port

#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define FALSE 0
#define TRUE 1

class Serial_reader {
    private:
    



    
    static std::string getSerialData(char* serial_device_path) {
        //  /dev/cu.usbserial
        // http://bradsmc.blogspot.com/2013/11/c-code-to-read-gps-data-via-serial-on.html

        struct termios oldtio, newtio;
        int fd, c, res;
        char buf[255];
        // Load the pin configuration
        
        /* Open modem device for reading and writing and not as controlling tty
        because we don't want to get killed if linenoise sends CTRL-C. */
        fd = open(serial_device_path, O_RDWR | O_NOCTTY );
        if (fd < 0) { perror(serial_device_path); exit(-1); }

        bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

        /* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
        CRTSCTS : output hardware flow control (only used if the cable has
                    all necessary lines. See sect. 7 of Serial-HOWTO)
        CS8     : 8n1 (8bit,no parity,1 stopbit)
        CLOCAL  : local connection, no modem contol
        CREAD   : enable receiving characters */
        newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

        /* IGNPAR  : ignore bytes with parity errors
        otherwise make device raw (no other input processing) */
        newtio.c_iflag = IGNPAR;

        /*  Raw output  */
        newtio.c_oflag = 0;

        /* ICANON  : enable canonical input
        disable all echo functionality, and don't send signals to calling program */
        newtio.c_lflag = ICANON;
        /* now clean the modem line and activate the settings for the port */
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd,TCSANOW,&newtio);
        // NMEA command to ouput all sentences
        // Note that this code & format values in manual are hexadecimal
        //write(fd, "$PTNLSNM,273F,01*27\r\n", 21);
        /* terminal settings done, now handle input*/
        int ii = 0;
        std::string serialString;
        while (ii <22) {    
            /*  read blocks program execution until a line terminating character is
                input, even if more than 255 chars are input. If the number
                of characters read is smaller than the number of chars available,
                subsequent reads will return the remaining chars. res will be set
                to the actual number of characters actually read */
            res = read(fd, buf, 255);
            buf[res] = 0;             /* set end of string, so we can printf */
            //printf("%s", buf, res);
            ii++;
            serialString += buf;
        }
        std::cout << serialString << std::endl;
        tcsetattr(fd, TCSANOW, &oldtio);
        close(fd);
        return serialString;
                
    }


    public:
    static std::string readSerialUSB() {
        return getSerialData("/dev/cu.usbmodem1D11401");
    }

};