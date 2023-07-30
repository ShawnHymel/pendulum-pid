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

class ControlComms {
  public:

    static const int OK = 0;
    static const int ERROR = -1;

    typedef enum {
      DEBUG_NONE = 0,
      DEBUG_ERROR,
      DEBUG_WARN,
      DEBUG_INFO
    } DebugLevel;

    /**
     * Constructor
     */
    ControlComms() {
    }

    /**
     * Destructor
     */
    ~ControlComms() {
    }

    /**
     * Initialize serial object
     */
    template <typename T>
    int init(T &ser, int baud = 115200, DebugLevel debug_level = DEBUG_NONE) {

      // Assign our serial object
      stream = &ser;

      // Assign debugging
      debug = debug_level;

      return OK;
    }

    /**
     * Receive action command, parse actions into floating point array.
     */
    int receive_actions(float *actions_out, int max_actions) {

      DeserializationError err = DeserializationError::Ok;
      StaticJsonDocument<200> doc;
      int retcode = OK; // TODO: enum retcode

      // Return early if nothing in receive buffer
      if (stream->available() <= 0) {
        return OK;
      }

      // Attempt to parse JSON
      err = deserializeJson(doc, *stream);
      if (err.code() == DeserializationError::Ok) {

        // TODO: do parsing stuff
        if (doc.containsKey(rx_key)) {
          double test = doc[rx_key];
          stream->println(test);
        } else {
          retcode = ERROR;
          if (debug >= DEBUG_ERROR) {
            stream->print("JSON Error: key '");
            stream->print(rx_key);
            stream->println("' not found");
          }
        }
      } else {
        retcode = ERROR;
      }

      // Flush receive buffer to avoid reading any extra '\r' or '\n' next time
      while (isspace(Serial.peek())) {
        Serial.read();
      }

      // Debug JSON parsing
      if (debug >= DEBUG_ERROR) {
        if (err.code() != DeserializationError::Ok) {
          stream->print("JSON Error: ");
          stream->println(err.f_str());
        }
      }
      
      return retcode;
    }

  private:
    Stream *stream;
    DebugLevel debug = DEBUG_NONE;
    static constexpr char rx_key[] = "actions";
};