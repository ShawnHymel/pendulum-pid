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

    typedef enum {
      DEBUG_NONE = 0,
      DEBUG_ERROR,
      DEBUG_WARN,
      DEBUG_INFO
    } DebugLevel;

    typedef enum {
      OK = 0,
      ERROR
    } StatusCode;

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
    int init(T &ser,
             int baud = 115200, 
             DebugLevel debug_level = DEBUG_NONE) {

      // Assign our serial object
      stream = &ser;

      // Assign debugging
      debug = debug_level;

      return OK;
    }

    /**
     * Receive action command, parse actions into floating point array.
     */
    template <unsigned int num_actions>
    StatusCode receive_actions(float *actions_out) {

      DeserializationError err = DeserializationError::Ok;
      StatusCode retcode = OK;

      // Figure out size of receive doc capacity
      constexpr unsigned int rx_doc_capacity = 
        JSON_OBJECT_SIZE(1) + JSON_ARRAY_SIZE(num_actions + 1);
      StaticJsonDocument<rx_doc_capacity> doc;

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

      // We'll get some invalid straggle chars over Serial, so ignore them
      } else {
        switch (err.code()) {
          case DeserializationError::EmptyInput:
          case DeserializationError::IncompleteInput:
          case DeserializationError::InvalidInput:
            break;
          default:
            retcode = ERROR;
            break;
        }
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
    size_t rx_doc_capacity = 0;
    DebugLevel debug = DEBUG_NONE;
    static constexpr char rx_key[] = "actions";
};