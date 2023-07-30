/**
 * Control communication interface to send and receive messages over any
 * serial device.
 *
 * Author: Shawn Hymel
 * Date: July 29, 2023
 * License: 0BSD (https://opensource.org/license/0bsd/)
 */

#include <Arduino.h>
#include <ArduinoJson.h>

class Control {
  public:

    static const int OK = 0;
    static const int ERROR = -1;

    /**
     * Constructor
     */
    Control() {
    }

    /**
     * Destructor
     */
    ~Control() {
      delete[] buf;
    }

    /**
     * Initialize serial object
     */
    template <typename T>
    int init(T &ser, int baud = 115200, unsigned int max_buf_size = 512) {

      // Assign our serial object
      stream = &ser;

      // Allocate buffer
      buf_size = max_buf_size;
      buf = new char[buf_size];

      return OK;
    }

    /**
     * Wait for full frame (this is blocking while the message is read)
     */
    int receive() {

      int num_bytes_received = 0;

      // Make sure we have a serial object and a buffer
      if ((stream == nullptr) || (buf == nullptr)) {
        return ERROR;
      }

      // Check for messages
      if (stream->available() > 0) {
        
        // Read entire message
        num_bytes_received = stream->readBytesUntil(CONTROL_EOF, buf, buf_size);

      }

      return num_bytes_received;
    }

    /**
     * Receive action command, parse actions into floating point array.
     */
    int receive_actions(float *actions_out, int max_actions) {
      
      int num_actions = 0;
      int num_bytes_received = 0;

      // Receive raw bytes
      num_bytes_received = receive();
      if (num_bytes_received > 0) {

        // Check for JSON { } start and end characters
        if ((buf[0] == '{') and (buf[num_bytes_received - 1] == '}')) {

          // TEST: echo rest of buffer
          for (int i = 1; i < num_bytes_received - 1; i++) {
            stream->print(buf[i]);
          }
          stream->print('\n');

        } else {
          num_bytes_received = ERROR;
        }
      }

      return num_bytes_received;
    }

  private:
    Stream *stream;
    unsigned int buf_size = 0;
    char *buf = nullptr;
    static const int CONTROL_EOF = '\n';
};